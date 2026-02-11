import pyrealsense2 as rs
import open3d as o3d
import numpy as np
import json, time, cv2, base64
from mcap.writer import Writer
from scipy.spatial.transform import Rotation as R

def main():
    # 1. Hardware Reset
    ctx = rs.context()
    for dev in ctx.query_devices(): dev.hardware_reset()
    time.sleep(3)

    output_file = "recording.mcap"
    ans_identity = o3d.core.Tensor(np.eye(4), dtype=o3d.core.float32)

    f = open(output_file, "wb")
    writer = Writer(f, chunk_size=4*1024*1024)
    writer.start()

    try:
        # --- Strict Schemas (Explicitly defining base64 encoding) ---
        tf_sid = writer.register_schema(name="foxglove.FrameTransform", encoding="jsonschema", data=json.dumps({"type":"object"}).encode())
        img_sid = writer.register_schema(name="foxglove.CompressedImage", encoding="jsonschema", data=json.dumps({
            "type": "object", "properties": {"data": {"type": "string", "contentEncoding": "base64"}}
        }).encode())
        depth_sid = writer.register_schema(name="foxglove.RawImage", encoding="jsonschema", data=json.dumps({
            "type": "object", "properties": {
                "data": {"type": "string", "contentEncoding": "base64"},
                "step": {"type": "integer"}, "width": {"type": "integer"}, "height": {"type": "integer"}
            }
        }).encode())

        tf_chan = writer.register_channel(topic="/tf", message_encoding="json", schema_id=tf_sid)
        rgb_chan = writer.register_channel(topic="/cam_rgb/compressed", message_encoding="json", schema_id=img_sid)
        depth_chan = writer.register_channel(topic="/cam_depth/raw", message_encoding="json", schema_id=depth_sid)

        # --- RealSense Setup ---
        pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        config.enable_stream(rs.stream.color, 640, 480, rs.format.rgb8, 30)
        config.enable_stream(rs.stream.accel)
        config.enable_stream(rs.stream.gyro)

        profile = pipeline.start(config)
        align = rs.align(rs.stream.color)
        
        # Get Sensor Transformation (IMU to Camera)
        accel_stream = profile.get_stream(rs.stream.accel)
        color_stream = profile.get_stream(rs.stream.color)
        imu_to_color = accel_stream.get_extrinsics_to(color_stream)
        imu_rot_mat = np.array(imu_to_color.rotation).reshape(3,3)

        intr = color_stream.as_video_stream_profile().get_intrinsics()
        intrinsic_t = o3d.core.Tensor([[intr.fx, 0, intr.ppx], [0, intr.fy, intr.ppy], [0, 0, 1]], dtype=o3d.core.float32)

        # Calibration
        print("⏳ Calibrating... HOLD STILL.")
        accel_samples = []
        for _ in range(100):
            fs = pipeline.wait_for_frames()
            acc_f = fs.first_or_default(rs.stream.accel)
            if acc_f:
                d = acc_f.as_motion_frame().get_motion_data()
                accel_samples.append([d.x, d.y, d.z])
        
        avg_accel = np.mean(accel_samples, axis=0)
        gravity_norm = avg_accel / np.linalg.norm(avg_accel)
        target = np.array([0, 0, 1])
        v = np.cross(gravity_norm, target); c = np.dot(gravity_norm, target); s = np.linalg.norm(v)
        kmat = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
        rot_matrix = np.eye(3) + kmat + kmat.dot(kmat) * ((1 - c) / (s**2))

        # State
        global_pos = np.zeros(3)
        global_rot = R.from_matrix(rot_matrix)
        last_gyro_ts = None
        prev_rgbd = None

        print("🔴 RECORDING. 6-DOF Motion Enabled + Buffer Fix.")

        while True:
            frames = pipeline.wait_for_frames()
            now_ns = time.time_ns()
            ts_obj = {"sec": int(now_ns // 1e9), "nsec": int(now_ns % 1e9)}

            # 1. Gyro
            gyro_f = frames.first_or_default(rs.stream.gyro)
            if gyro_f:
                ts = gyro_f.get_timestamp()
                if last_gyro_ts is not None:
                    g_dt = (ts - last_gyro_ts) / 1000.0
                    g = gyro_f.as_motion_frame().get_motion_data()
                    global_rot = global_rot * R.from_rotvec(np.array([g.x, g.y, g.z]) * g_dt)
                last_gyro_ts = ts

            # 2. Accel (Mapped to Camera Space)
            accel_f = frames.first_or_default(rs.stream.accel)
            acc_mag = 0.0
            lin_acc_cam = np.zeros(3)
            if accel_f:
                a = accel_f.as_motion_frame().get_motion_data()
                lin_acc_cam = imu_rot_mat @ (np.array([a.x, a.y, a.z]) - avg_accel)
                acc_mag = np.linalg.norm(lin_acc_cam)

            # 3. Vision
            aligned = align.process(frames)
            c_f, d_f = aligned.get_color_frame(), aligned.get_depth_frame()
            if not c_f or not d_f: continue
            c_np, d_np = np.asanyarray(c_f.get_data()), np.asanyarray(d_f.get_data())
            
            # VO Logic
            curr_rgbd = o3d.t.geometry.RGBDImage(o3d.t.geometry.Image(o3d.core.Tensor(c_np)), o3d.t.geometry.Image(o3d.core.Tensor(d_np)))
            if prev_rgbd is not None:
                res = o3d.t.pipelines.odometry.rgbd_odometry_multi_scale(prev_rgbd, curr_rgbd, intrinsic_t, ans_identity, 1000.0, 3.0)
                vo_t = res.transformation.cpu().numpy()[:3, 3]
                vo_mag = np.linalg.norm(vo_t)
                if vo_mag > 0.001:
                    u_vo = vo_t / vo_mag
                    u_acc = lin_acc_cam / acc_mag if acc_mag > 0.05 else np.zeros(3)
                    # Absolute correlation for multi-direction support
                    if acc_mag > 0.15 and np.abs(np.dot(u_vo, u_acc)) > 0.2:
                        global_pos += global_rot.as_matrix() @ vo_t
            prev_rgbd = curr_rgbd

            # 4. EXPORT (The DataView Fix)
            quat = global_rot.as_quat()
            
            # TF
            writer.add_message(tf_chan, log_time=now_ns, publish_time=now_ns, data=json.dumps({
                "timestamp": ts_obj, "parent_frame_id": "world", "child_frame_id": "camera",
                "translation": {"x": float(global_pos[0]), "y": float(global_pos[1]), "z": -float(global_pos[2])},
                "rotation": {"x": quat[0], "y": quat[1], "z": quat[2], "w": quat[3]}
            }).encode("utf-8"))

            # RGB
            _, rgb_buf = cv2.imencode(".jpg", cv2.cvtColor(c_np, cv2.COLOR_RGB2BGR))
            writer.add_message(rgb_chan, log_time=now_ns, publish_time=now_ns, data=json.dumps({
                "timestamp": ts_obj, "frame_id": "camera", "format": "image/jpeg",
                "data": base64.b64encode(rgb_buf).decode("utf-8")
            }).encode("utf-8"))

            # DEPTH - The "Bulletproof" Buffer
            # Force conversion to uint16 and standard C-order
            depth_array = np.ascontiguousarray(d_np, dtype=np.uint16)
            depth_base64 = base64.b64encode(depth_array.data).decode("utf-8")
            
            writer.add_message(depth_chan, log_time=now_ns, publish_time=now_ns, data=json.dumps({
                "timestamp": ts_obj, 
                "frame_id": "camera", 
                "width": 640, 
                "height": 480, 
                "encoding": "16UC1", 
                "step": 1280, 
                "data": depth_base64
            }).encode("utf-8"))

            print(f"XYZ: {global_pos[0]:.2f}, {global_pos[1]:.2f}, {global_pos[2]:.2f} | Accel: {acc_mag:.2f}", end='\r')

    except KeyboardInterrupt: pass
    finally:
        writer.finish()
        f.close()
        pipeline.stop()
if __name__=="__main__":
    main()
