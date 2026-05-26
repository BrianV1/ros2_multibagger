# realsense_capture

A ROS 2 package for capturing synchronized depth images and point clouds from an Intel RealSense camera into a ROS bag on-demand. This tool is designed for efficient sensor ground truth curation workflows, where operators need to selectively save high-quality depth and 3D scene data instead of continuously recording large datasets.

## Overview

`realsense_capture` subscribes to:

- `/camera/camera/depth/image_rect_raw`
- `/camera/camera/depth/color/points`

When the user presses `ENTER`, the node captures:

1. One depth image frame
2. One point cloud frame

Both messages are written into the same ROS bag with their original timestamps preserved.

The node then waits for the next manual capture request.

Each session automatically creates a timestamped bag directory:

```bash
bag_YYYY-MM-DD_HH-MM-SS
```

Example:

```bash
bag_2026-05-26_18-42-15
```

---

# Use Case: Sensor Ground Truth Curation

This package is useful for building curated datasets for:

- Depth estimation validation
- SLAM benchmarking
- Point cloud registration
- Sensor fusion experiments
- Robotics perception pipelines
- Machine learning dataset generation
- RealSense calibration and testing

Instead of recording long continuous bags and manually extracting useful frames later, this tool allows an operator to:

- Position the robot or camera
- Wait for a stable scene
- Press `ENTER`
- Save exactly one aligned depth + point cloud sample

This dramatically reduces:

- Storage requirements
- Dataset cleanup time
- Post-processing overhead

while improving dataset quality and labeling consistency.

---

# Features

- On-demand capture using keyboard input
- Saves depth image and point cloud together
- Multi-threaded ROS 2 executor
- Thread-safe capture logic using atomic flags
- Automatic rosbag creation
- Timestamp preservation
- Clean shutdown handling
- Separate callback groups for concurrent subscriptions

---

# Dependencies

## ROS 2 Dependencies

This package requires:

- `rclcpp`
- `sensor_msgs`
- `rosbag2_cpp`
- `ament_cmake`

## Hardware / Driver Dependencies

Typically used with:

- Intel RealSense cameras
- `realsense2_camera` ROS 2 driver

Example supported devices:

- D435
- D455
- L515

---

# Installation

## 1. Create a ROS 2 Workspace

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

## 2. Clone the Package

```bash
git clone <your-repository-url>
```

## 3. Install Dependencies

From the workspace root:

```bash
cd ~/ros2_ws

rosdep install --from-paths src --ignore-src -r -y
```

## 4. Build the Package

```bash
colcon build --packages-select realsense_capture
```

## 5. Source the Workspace

```bash
source install/setup.bash
```

---

# Running the Node

## Start the RealSense Driver

Example:

```bash
ros2 launch realsense2_camera rs_launch.py
```

Verify topics:

```bash
ros2 topic list
```

Expected topics:

```bash
/camera/camera/depth/image_rect_raw
/camera/camera/depth/color/points
```

---

## Run the Capture Node

```bash
ros2 run realsense_capture capture
```

You should see:

```bash
Press ENTER to capture depth + cloud into the same bag.
```

---

# Capturing Data

Press:

```bash
ENTER
```

The node will:

1. Capture the next available depth frame
2. Capture the next available point cloud
3. Write both into the rosbag
4. Wait for another capture request

Example output:

```bash
Capture requested
Depth captured
Point cloud captured
Capture complete (waiting for next key)
```

---

# ROS Bag Output

Generated bags contain:

```bash
/camera/camera/depth/image_rect_raw
/camera/camera/depth/color/points
```

You can inspect the bag with:

```bash
ros2 bag info <bag_name>
```

Replay with:

```bash
ros2 bag play <bag_name>
```

---

# Architecture

## Multi-threaded Execution

The node uses a `MultiThreadedExecutor` with two mutually exclusive callback groups:

- Depth image callback group
- Point cloud callback group

This allows both subscriptions to operate concurrently while remaining thread-safe.

---

## Capture Logic

The workflow is controlled with atomic state flags:

| Flag | Purpose |
|---|---|
| `capture_requested` | Indicates a pending capture |
| `depth_written` | Prevents duplicate depth writes |
| `cloud_written` | Prevents duplicate cloud writes |

Once both frames are written:

```cpp
capture_requested = false;
```

and the node waits for the next keyboard event.

---

# Package Structure

```text
realsense_capture/
├── CMakeLists.txt
├── package.xml
└── src/
    └── capture.cpp
```

---

# Example Applications

## Dataset Collection

Capture sparse, high-quality samples for:

- Semantic segmentation
- Depth completion
- 3D reconstruction

## Robotics Experiments

Collect reproducible sensor states during:

- Navigation tests
- Manipulation tasks
- Calibration procedures

## Ground Truth Generation

Use manually curated captures as:

- Benchmark datasets
- Validation frames
- Reference sensor measurements

---

# Known Limitations

- Captures are not hardware synchronized
- The next received messages after pressing ENTER are stored
- No RGB image capture currently
- No metadata export
- Assumes RealSense topic naming conventions

---

# Possible Improvements

Potential future extensions:

- RGB image capture
- Approximate time synchronization
- Service-based triggering
- GUI capture interface
- Automatic annotation support
- Capture metadata logging
- Frame indexing
- Compression support

---

# Build Files

## CMakeLists.txt

Uses:

- `ament_cmake`
- `rclcpp`
- `sensor_msgs`
- `rosbag2_cpp`

Build target:

```cmake
add_executable(capture src/capture.cpp)
```

---

## package.xml

Defines ROS 2 package metadata and dependencies required for compilation and runtime.

---

# License

Add your license information in:

- `package.xml`
- Source file headers

Example:

```xml
<license>MIT</license>
```

---

# Maintainer

Brian Viner

Email:

```text
brian.viner0624@gmail.com
```
