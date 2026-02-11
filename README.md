# RealSense D435i MCAP Recorder

A recording utility for the Intel RealSense D435i that captures RGB-D data and camera trajectory (6-DOF) into MCAP files. This tool uses IMU-validated Visual Odometry to ensure spatial accuracy and minimal drift.

----

## Prerequisites

**Hardware Requirements**
* **Camera:** Intel RealSense **D435i** (The 'i' model is required for IMU/Motion data).
* **Cable and Port:** **USB 3.0 Supporting Cable and port**. Standard USB 2.0 cables and port lack the bandwidth to handle simultaneous RGB-D and IMU data streams.

**Software Requirements**
This project uses the **`uv`** package manager for fast, reproducible environments.
* **Install `uv`**:
    * **macOS/Linux:** `curl -LsSf https://astral.sh/uv/install.sh | sh`

----

## Run the Script
Execute the recorder using uv:
```bash
uv run main.py
```
+ **Calibration:** The script calibrates the IMU. Make sure the camera is still until the calibration is done.
+ **Recording:** The script will output real-time `[STATUS] XYZ` coordinates in the terminal.
+ **Stop:** Press `Ctrl+C` to stop the pipeline and safely finalize the MCAP file.
+ **Storage:** The MCAP flie will be saved in the location of the script.

----

## Data Visualization
The generated `recording.mcap` can be opened directly in [Foxglove Studio](https://app.foxglove.dev/).

**Recommended Layout:**

+ **Image Panel:** Set to `/cam_rgb/compressed` (RGB image data).

+ **Image Panel:** Set to `/cam_depth/raw` (Depth data).

+ **3D Panel:** Ensure the `/tf` frame is active to see the physical path of the camera.

+ **Raw Message Panel:** Set to `/tf` to get the raw data of the camera pose in 3D space. (Used for debugging purpose)
