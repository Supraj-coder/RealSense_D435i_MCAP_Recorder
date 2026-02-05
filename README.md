# RealSense_D435i_MCAP_Recorder
Record data from the Realsense D435i camera into a .MCAP file using ROS2

This guide provides a workflow to install, run, and record synchronized RGB-D data and 3D trajectories from an Intel RealSense D435i into a high-performance MCAP file.

<br>

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
  librealsense2-dkms \
  ros-humble-cv-bridge \
  ros-humble-image-transport-plugins
```

**Note on ```librealsense2-dkms``` Errors**

While installing dependencies, you may encounter an error message regarding ```librealsense2-dkms``` (e.g., ```exit status 10``` or ```BUILD_EXCLUSIVE``` directive).

+ **When to Ignore:** If the installation finishes with an error but you can still run realsense-viewer and see the camera feed, you can safely ignore the error. Modern Ubuntu kernels (especially 6.2+) often have built-in support for RealSense cameras, making the DKMS patch redundant.
+ **The libuvc Workaround:** If your camera is not detected at all and the DKMS installation continues to fail, use the **libuvc backend**. This method bypasses the Linux kernel entirely and works via standard USB protocols.

<br>  

**Using the libuvc Backend**
If you cannot get the standard drivers to work, you can build ```librealsense``` from source using the provided Intel script:

```bash
# Clone the repository
git clone https://github.com/IntelRealSense/librealsense.git
cd librealsense/scripts

# Run the libuvc installation script
./libuvc_installation.sh
```
This script will compile the SDK in a mode that does not require kernel patches, ma
king it compatible with almost any Linux kernel version.

##

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

**Terminal 2: Start SLAM (Visual Odometry) and Line Verification**

This generates the 3D trajectory (position and orientation) in the world frame.
```bash
ros2 launch rtabmap_launch rtabmap.launch.py \
    rtabmap_args:="--delete_db_on_start" \
    rgb_topic:=/camera/camera/color/image_raw \
    depth_topic:=/camera/camera/aligned_depth_to_color/image_raw \
    camera_info_topic:=/camera/camera/color/camera_info \
    frame_id:=camera_link \
    approx_sync:=true \
    approx_sync_max_interval:=0.05 \
    wait_imu_to_init:=false \
    rtabmap_viz:=true
```
When you run the command above, a window titled RTAB-Map Viz will open.
- **Green flashes:** Indicate a successful "Loop Closure" (the system recognizes a place it has seen before).
- **3D Map:** You should see a point cloud forming as you move. If the cloud stops updating, the recording will be missing trajectory data.
- **Odometry:** If the screen turns RED, it means "Tracking Lost." Stop moving and point the camera back at a previous location to recover.

<br>  

**Terminal 3: Record to MCAP**
Saves all data to a compressed MCAP file to prevent frame drops.

+ For ```.bash``` terminals.

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

+ For ```.zsh``` terminals.

```bash
# First, create the config file
echo "compression: 'Zstd'" > mcap_config.yaml

# Then run the record command
ros2 bag record -s mcap \
  --storage-config-file mcap_config.yaml \
  /camera/camera/color/image_raw \
  /camera/camera/aligned_depth_to_color/image_raw \
  /camera/camera/color/camera_info \
  /camera/camera/imu \
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

**Terminal 1:**
To check message counts and health:
```bash
ros2 bag info my_realsense_session
```
To start the visualzer (without the camera):
```bash
ros2 launch rtabmap_launch rtabmap.launch.py \
    use_sim_time:=true \
    rgb_topic:=/camera/camera/color/image_raw \
    depth_topic:=/camera/camera/aligned_depth_to_color/image_raw \
    camera_info_topic:=/camera/camera/color/camera_info \
    rtabmap_viz:=true
```
**Terminal 2:**
To play back the data: You can replay this session exactly as it happened without the camera plugged in.
```bash
ros2 bag play my_realsense_session
```
##

## Important Troubleshooting

1. **USB 3.0 Requirement:** The D435i requires a high-bandwidth USB 3.0 port. Use ```rs-enumerate-devices``` to confirm "USB Type" is 3.2.
2. When ```ros2: command not found``` error is encountered, source the ROS 2 installation by executing ```source /opt/ros/humble/setup.bash```.
3. **SLAM Tracking:** Visual Odometry (SLAM) requires distinct 'features' to track. Avoid pointing the camera at blank white walls or dark corners, as this will cause the ```/rtabmap/odom``` topic to stop publishing.
4. **Hardware Sync:** If frames are out of sync, ensure ```enable_sync:=true``` is set in Terminal 1.
5. **Note on Performance:** If your computer lags while recording, you can set ```rtabmap_viz:=false``` in Step 2 Terminal 2 to save CPU power.

## To Include IMU data:

**Update the commands of step 2 as shown below**

+ Terminal 1

```bash
ros2 launch realsense2_camera rs_launch.py \
  initial_reset:=true \
  align_depth.enable:=true \
  enable_accel:=true \
  enable_gyro:=true \
  unite_imu_method:=2 \
  rgb_camera.profile:=640x480x30 \
  depth_module.profile:=640x480x30 \
  enable_sync:=true
```

+ Terminal 2
Set ```wait_imu_to_init:=true``` to ensure that the SLAM orientation is initialized by gravity, making the map "upright."

```bash
ros2 launch rtabmap_launch rtabmap.launch.py \
    rtabmap_args:="--delete_db_on_start" \
    rgb_topic:=/camera/camera/color/image_raw \
    depth_topic:=/camera/camera/aligned_depth_to_color/image_raw \
    camera_info_topic:=/camera/camera/color/camera_info \
    frame_id:=camera_link \
    approx_sync:=true \
    approx_sync_max_interval:=0.05 \
    wait_imu_to_init:=true \
    rtabmap_viz:=true
```

+ Terminal 3

```bash
  ros2 bag record -s mcap \
  --storage-config-file <(echo "compression: 'Zstd'") \
  /camera/camera/color/image_raw \
  /camera/camera/aligned_depth_to_color/image_raw \
  /camera/camera/color/camera_info \
  /camera/camera/imu \
  /rtabmap/odom \
  /tf \
  /tf_static \
  -o my_realsense_session
```

**Note**
If any issues are encountered regarding "Topic not found", then verify the required topic name using the ```bash ros2 topic list``` command.
