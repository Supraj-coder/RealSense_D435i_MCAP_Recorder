import pyrealsense2 as rs
import open3d as o3d
import numpy as np
import json, time, cv2, base64, signal, sys
from mcap.writer import Writer
from scipy.spatial.transform import Rotation as R

running = True
writer = None
f = None
pipeline = None

def signal_handler(sig, frame):
    global running
    if not running: return
    running = False
    print("\n\n🛑 STOP SIGNAL RECEIVED. Finalizing MCAP...")

signal.signal(signal.SIGINT, signal_handler)

def main():
    global writer, f, pipeline, running
    
    ctx = rs.context()
    for dev in ctx.query_devices(): dev.hardware_reset()
    time.sleep(2)

    output_file = "realsense_recording.mcap"
    ans_identity = o3d.core.Tensor(np.eye(4), dtype=o3d.core.float32)

    f = open(output_file, "wb")
    writer = Writer(f, chunk_size=8*1024*1024)
    writer.start()

    try:
        # --- Schemas ---
        base64_schema = {"type": "object", "properties": {"data": {"type": "string", "contentEncoding": "base64"}}}
        tf_sid = writer.register_schema(name="foxglove.FrameTransform", encoding="jsonschema", data=json.dumps({"type":"object"}).encode())
        img_sid = writer.register_schema(name="foxglove.CompressedImage", encoding="jsonschema", data=json.dumps(base64_schema).encode())
        depth_sid = writer.register_schema(name="foxglove.RawImage", encoding="jsonschema", data=json.dumps(base64_schema).encode())

        tf_chan = writer.register_channel(topic="/tf", message_encoding="json", schema_id=tf_sid)
        rgb_chan = writer.register_channel(topic="/cam_rgb/compressed", message_encoding="json", schema_id=img_sid)
        depth_chan = writer.register_channel(topic="/cam_depth/raw", message_encoding="json", schema_id=depth_sid)

        pipeline = rs.pipeline()
        config = rs.config()
        W, H = 640, 480
        config.enable_stream(rs.stream.depth, W, H, rs.format.z16, 30)
        config.enable_stream(rs.stream.color, W, H, rs.format.rgb8, 30)
        config.enable_stream(rs.stream.gyro)
        config.enable_stream(rs.stream.accel)

        profile = pipeline.start(config)
        align = rs.align(rs.stream.color)
        
        d_scale = profile.get_device().first_depth_sensor().get_depth_scale()
        intr = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
        intrinsic_t = o3d.core.Tensor([[intr.fx, 0, intr.ppx], [0, intr.fy, intr.ppy], [0, 0, 1]], dtype=o3d.core.float32)

        global_pos = np.zeros(3)
        global_rot = R.from_matrix(np.eye(3))
        last_imu_ts = None
        prev_rgbd = None
        
        # --- IMU Validation Parameters ---
        # Accel values are in m/s^2. 9.8 is gravity. 
        # We look for "Jerk" or significant changes to validate movement.
        accel_threshold = 0.25 # Ignore VO if linear accel is < 0.25 m/s^2
        alpha_vo = 0.3         # Smoothing for VO

        print(f"🔴 RECORDING... IMU-Validation Active.")

        while running:
            frames = pipeline.wait_for_frames(timeout_ms=1000)
            if not frames: continue
            
            now_ns = time.time_ns()
            ts_obj = {"sec": int(now_ns // 1e9), "nsec": int(now_ns % 1e9)}

            # 1. IMU Fusion (Gyro for Rotation, Accel for Validation)
            gyro_f = frames.first_or_default(rs.stream.gyro)
            accel_f = frames.first_or_default(rs.stream.accel)
            
            movement_validated = False
            if gyro_f and accel_f:
                ts = gyro_f.get_timestamp()
                if last_imu_ts:
                    dt = (ts - last_imu_ts) / 1000.0
                    g_data = gyro_f.as_motion_frame().get_motion_data()
                    a_data = accel_f.as_motion_frame().get_motion_data()
                    
                    # Update Rotation
                    global_rot = global_rot * R.from_rotvec(np.array([g_data.x, g_data.y, g_data.z]) * dt)
                    
                    # Accel Validation: Remove estimated gravity (rough approx)
                    # and check if the residual force is high enough to be "real" motion
                    accel_vec = np.array([a_data.x, a_data.y, a_data.z])
                    # Magnitude of acceleration minus Earth's gravity
                    linear_accel_mag = abs(np.linalg.norm(accel_vec) - 9.806)
                    
                    if linear_accel_mag > accel_threshold:
                        movement_validated = True
                
                last_imu_ts = ts

            # 2. Visual Odometry
            aligned = align.process(frames)
            c_f, d_f = aligned.get_color_frame(), aligned.get_depth_frame()
            if not c_f or not d_f: continue
            
            c_np, d_np = np.asanyarray(c_f.get_data()), np.asanyarray(d_f.get_data())
            curr_rgbd = o3d.t.geometry.RGBDImage(
                o3d.t.geometry.Image(o3d.core.Tensor(c_np)),
                o3d.t.geometry.Image(o3d.core.Tensor(d_np))
            )

            if prev_rgbd is not None:
                try:
                    res = o3d.t.pipelines.odometry.rgbd_odometry_multi_scale(
                        prev_rgbd, curr_rgbd, intrinsic_t, ans_identity, 1.0/d_scale, 3.0
                    )
                    
                    if res.fitness > 0.8:
                        vo_t = res.transformation.cpu().numpy()[:3, 3]
                        
                        # THE VALIDATION STEP: 
                        # Only apply VO translation if the IMU also felt a force, 
                        # OR if the rotation is significant (camera tilt).
                        if movement_validated or np.linalg.norm(vo_t) > 0.05:
                            global_pos += global_rot.as_matrix() @ (vo_t * alpha_vo)
                        else:
                            # If no force felt, assume "phantom" pixel drift and damp heavily
                            global_pos += global_rot.as_matrix() @ (vo_t * 0.01)
                            
                except: pass
            prev_rgbd = curr_rgbd

            # 3. EXPORT
            quat = global_rot.as_quat()
            writer.add_message(channel_id=tf_chan, log_time=now_ns, publish_time=now_ns, data=json.dumps({
                "timestamp": ts_obj, "parent_frame_id": "world", "child_frame_id": "camera",
                "translation": {"x": float(global_pos[2]), "y": float(global_pos[0]), "z": float(global_pos[1])},
                "rotation": {"x": quat[2], "y": -quat[0], "z": -quat[1], "w": quat[3]}
            }).encode("utf-8"))

            _, rgb_enc = cv2.imencode(".jpg", cv2.cvtColor(c_np, cv2.COLOR_RGB2BGR), [int(cv2.IMWRITE_JPEG_QUALITY), 60])
            writer.add_message(channel_id=rgb_chan, log_time=now_ns, publish_time=now_ns, data=json.dumps({
                "data": base64.b64encode(rgb_enc).decode("utf-8")
            }).encode("utf-8"))

            depth_raw = np.ascontiguousarray(d_np, dtype=np.uint16).tobytes()
            writer.add_message(channel_id=depth_chan, log_time=now_ns, publish_time=now_ns, data=json.dumps({
                "width": W, "height": H, "encoding": "16UC1", "step": W * 2,
                "data": base64.b64encode(depth_raw).decode("utf-8")
            }).encode("utf-8"))

            status = "MOVING" if movement_validated else "STILL "
            print(f"[{status}] XYZ: {global_pos[0]:.3f}, {global_pos[1]:.3f}, {global_pos[2]:.3f}", end='\r')

    except Exception as e:
        print(f"\n❌ Error: {e}")
    finally:
        if pipeline: pipeline.stop()
        if writer: writer.finish()
        if f: f.close()
        print("\n✅ Done.")

if __name__ == "__main__":
    main()
