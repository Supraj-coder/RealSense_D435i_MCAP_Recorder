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
        # --- Schemas ---
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
        
        # Sensor Transformation
        accel_stream = profile.get_stream(rs.stream.accel)
        color_stream = profile.get_stream(rs.stream.color)
        imu_to_color = accel_stream.get_extrinsics_to(color_stream)
        imu_rot_mat = np.array(imu_to_color.rotation).reshape(3,3)

        intr = color_stream.as_video_stream_profile().get_intrinsics()
        intrinsic_t = o3d.core.Tensor([[intr.fx, 0, intr.ppx], [0, intr.fy, intr.ppy], [0, 0, 1]], dtype=o3d.core.float32)

        # --- Enhanced Calibration ---
        print("⏳ Calibrating gravity vector... HOLD STILL.")
        accel_samples = []
        while len(accel_samples) < 100:
            fs = pipeline.wait_for_frames()
            acc_f = fs.first_or_default(rs.stream.accel)
            if acc_f:
                d = acc_f.as_motion_frame().get_motion_data()
                accel_samples.append([d.x, d.y, d.z])
        
        avg_accel = np.mean(accel_samples, axis=0)
        gravity_norm = avg_accel / np.linalg.norm(avg_accel)
        
        # Initial Orientation Align with Gravity
        target = np.array([0, 0, 1])
        v = np.cross(gravity_norm, target)
        c = np.dot(gravity_norm, target)
        s = np.linalg.norm(v)
        kmat = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
        rot_matrix = np.eye(3) + kmat + kmat.dot(kmat) * ((1 - c) / (s**2 + 1e-8))

        # State Variables
        global_pos = np.zeros(3)
        global_rot = R.from_matrix(rot_matrix)
        last_gyro_ts = None
        prev_rgbd = None
        
        # Smoothing filters
        lin_acc_filt = np.zeros(3)
        alpha = 0.8  # LPF weight (higher = smoother, more lag)

        print("🔴 RECORDING. Accuracy Filters Active.")

        while True:
            frames = pipeline.wait_for_frames()
            now_ns = time.time_ns()
            ts_obj = {"sec": int(now_ns // 1e9), "nsec": int(now_ns % 1e9)}

            # 1. High-Frequency Gyro Update
            gyro_f = frames.first_or_default(rs.stream.gyro)
            if gyro_f:
                ts = gyro_f.get_timestamp()
                if last_gyro_ts is not None:
                    g_dt = (ts - last_gyro_ts) / 1000.0
                    g = gyro_f.as_motion_frame().get_motion_data()
                    # Small angle approximation for better stability
                    global_rot = global_rot * R.from_rotvec(np.array([-g.x, g.y, g.z]) * g_dt)
                last_gyro_ts = ts

            # 2. Accel Filtering (Moving Average)
            accel_f = frames.first_or_default(rs.stream.accel)
            if accel_f:
                a = accel_f.as_motion_frame().get_motion_data()
                raw_acc = imu_rot_mat @ (np.array([a.x, a.y, a.z]) - avg_accel)
                lin_acc_filt = alpha * lin_acc_filt + (1 - alpha) * raw_acc
            
            acc_mag = np.linalg.norm(lin_acc_filt)

            # 3. Vision Odometry with Confidence Gating
            aligned = align.process(frames)
            c_f, d_f = aligned.get_color_frame(), aligned.get_depth_frame()
            if not c_f or not d_f: continue
            
            c_np, d_np = np.asanyarray(c_f.get_data()), np.asanyarray(d_f.get_data())
            curr_rgbd = o3d.t.geometry.RGBDImage(o3d.t.geometry.Image(o3d.core.Tensor(c_np)), o3d.t.geometry.Image(o3d.core.Tensor(d_np)))
            
            if prev_rgbd is not None:
                try:
                    res = o3d.t.pipelines.odometry.rgbd_odometry_multi_scale(
                        prev_rgbd, curr_rgbd, intrinsic_t, ans_identity, 
                        depth_scale=1000.0, depth_max=3.0
                    )
                    
                    if res.fitness > 0.85:
                        vo_t = res.transformation.cpu().numpy()[:3, 3] # Movement in camera coords
                        vo_mag = np.linalg.norm(vo_t)

                        if vo_mag > 0.001:
                            u_vo = vo_t / vo_mag
                            acc_mag = np.linalg.norm(lin_acc_filt)
                            u_acc = lin_acc_filt / acc_mag if acc_mag > 0.1 else np.zeros(3)

                            # Use ABSOLUTE correlation to support both directions on the same axis
                            # np.abs(dot) > 0.2 means the vectors are aligned OR anti-aligned (same line of action)
                            correlation = np.abs(np.dot(u_vo, u_acc))

                            # Only apply motion if physical force is felt on the same axis as vision
                            if acc_mag > 0.20 and correlation > 0.2:
                                global_pos += global_rot.as_matrix() @ vo_t
                            else:
                                # Stationary/Noise cleanup: allow tiny high-confidence movements 
                                # but damp them heavily to prevent the "ghost drift" you saw earlier
                                if res.fitness > 0.95:
                                    global_pos += global_rot.as_matrix() @ (vo_t * 0.05)
                except RuntimeError as e:
                    # This catches the "Singular 6x6" error and prevents a crash
                    if "Singular 6x6" in str(e):
                        print("\n⚠️ Tracking Lost: Featureless area detected. Hold still...")
                    else:
                        print(f"\n❌ VO Error: {e}")
            prev_rgbd = curr_rgbd

            # 4. EXPORT
            quat = global_rot.as_quat()
            writer.add_message(tf_chan, log_time=now_ns, publish_time=now_ns, data=json.dumps({
                "timestamp": ts_obj, "parent_frame_id": "world", "child_frame_id": "camera",
                "translation": {"x": float(global_pos[0]), "y": float(global_pos[1]), "z": -float(global_pos[2])},
                "rotation": {"x": quat[0], "y": quat[1], "z": quat[2], "w": quat[3]}
            }).encode("utf-8"))

            _, rgb_buf = cv2.imencode(".jpg", cv2.cvtColor(c_np, cv2.COLOR_RGB2BGR))
            writer.add_message(rgb_chan, log_time=now_ns, publish_time=now_ns, data=json.dumps({
                "timestamp": ts_obj, "frame_id": "camera", "format": "image/jpeg",
                "data": base64.b64encode(rgb_buf).decode("utf-8")
            }).encode("utf-8"))

            depth_array = np.ascontiguousarray(d_np, dtype=np.uint16)
            writer.add_message(depth_chan, log_time=now_ns, publish_time=now_ns, data=json.dumps({
                "timestamp": ts_obj, "frame_id": "camera", "width": 640, "height": 480, 
                "encoding": "16UC1", "step": 1280, "data": base64.b64encode(depth_array.data).decode("utf-8")
            }).encode("utf-8"))

            print(f"XYZ: {global_pos[0]:.2f}, {global_pos[1]:.2f}, {global_pos[2]:.2f} | IMU Conf: {'HIGH' if acc_mag > 0.2 else 'LOW '}", end='\r')

    except KeyboardInterrupt: pass
    finally:
        writer.finish()
        f.close()
        pipeline.stop()
        print(f"✅ Recording saved to: {output_file}")
        print("🏁 Done.")

if __name__=="__main__":
    main()
