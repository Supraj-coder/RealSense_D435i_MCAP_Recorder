# RealSense_D435i_MCAP_Recorder
Record data from the Realsense D435i camera into a .MCAP file using ROS2

This guide provides a workflow to install, run, and record synchronized RGB-D data and 3D trajectories from an Intel RealSense D435i into a high-performance MCAP file.

## 1. Prerequisites & Installation:
These commands were tested on a Ubuntu 22.04 system with ROS 2 Humble installed.

<br>  


**A. Register Repositories**

First, we ensure the system can find the latest Intel and ROS 2 packages.
```bash
# Update system and install curl
sudo apt update && sudo apt install -y curl gnupg2 lsb-release

# Register Intel RealSense Repositories
sudo mkdir -p /etc/apt/keyrings

curl -sSf https://librealsense.intel.com/Debian/librealsense.pgp | sudo tee /etc/apt/keyrings/librealsense.pgp > /dev/null

echo "deb [signed-by=/etc/apt/keyrings/librealsense.pgp] https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" | sudo tee /etc/apt/list.d/librealsense.list

sudo apt update
```

<br>  

**B. Install Software Stack**

This installs the camera drivers, SLAM engine, and the MCAP recording plugin.
```bash
sudo apt install -y \
  ros-humble-realsense2-camera \
  ros-humble-rtabmap-ros \
  ros-humble-rosbag2-storage-mcap \
  librealsense2-utils \
  librealsense2-dkms
```

<br>  

**C. Set USB Permissions (udev rules)**

Crucial for allowing the camera to communicate with ROS without root privileges.
```bash
sudo curl https://raw.githubusercontent.com/IntelRealSense/librealsense/master/config/99-realsense-libusb.rules -o /etc/udev/rules.d/99-realsense-libusb.rules
sudo udevadm control --reload-rules && sudo udevadm trigger
```

<br>  

## 2. Recording Workflow

Open three separate terminal windows. You must source ROS 2 in every window: ```source /opt/ros/humble/setup.bash``` or write the sourcing line at the end of the ```~/.bashrc```

<br>  

**Terminal 1: Launch the Camera**

Aligns depth to color and sets a stable resolution for SLAM.

```bash
ros2 launch realsense2_camera rs_launch.py \
  initial_reset:=true \
  align_depth.enable:=true \
  rgb_camera.profile:=640x480x30 \
  depth_module.profile:=640x480x30 \
  enable_sync:=true
```
<br> 

**Terminal 2: Start SLAM (Visual Odometry)**
This generates the 3D trajectory (position and orientation) in the world frame.
```bash
ros2 launch rtabmap_launch rtabmap.launch.py \
    rtabmap_args:="--delete_db_on_start" \
    rgb_topic:=/camera/camera/color/image_raw \
    depth_topic:=/camera/camera/aligned_depth_to_color/image_raw \
    camera_info_topic:=/camera/camera/color/camera_info \
    frame_id:=camera_link \
    approx_sync:=true \
    wait_imu_to_init:=false \
    rtabmap_viz:=true
```

<br>  

**Terminal 3: Record to MCAP**
Saves all data to a compressed MCAP file to prevent frame drops.
```bash
ros2 bag record -s mcap \
  --storage-config-file <(echo "compression: 'Zstd'") \
  /camera/camera/color/image_raw \
  /camera/camera/aligned_depth_to_color/image_raw \
  /camera/camera/color/camera_info \
  /rtabmap/odom \
  /tf \
  /tf_static \
  -o my_realsense_session
```
Stop recording with ```Ctrl+C```.

<br>  

## 3. Data Inspection & Playback
The data is saved as a folder named ```my_realsense_session```.

<br>  

To check message counts and health:
```bash
ros2 bag info my_realsense_session
```
To play back the data: You can replay this session exactly as it happened without the camera plugged in.
```bash
ros2 bag play my_realsense_session
```
##

## Important Troubleshooting

1. **USB 3.0 Requirement:** The D435i requires a high-bandwidth USB 3.0 port. Use ```rs-enumerate-devices``` to confirm "USB Type" is 3.2.
2. **SLAM Tracking:** If the ```/rtabmap/odom``` topic stops sending data (Lost Tracking), move the camera slowly in a well-lit area with plenty of visual features (avoid white walls).
3. **Hardware Sync:** If frames are out of sync, ensure ```enable_sync:=true``` is set in Terminal 1.
