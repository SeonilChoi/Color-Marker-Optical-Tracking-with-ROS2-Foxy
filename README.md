# Color Marker Optical Tracking with ROS2 Foxy

In this work, we present a method for estimating the position and orientation of an object using an optical tracking technique with color marker spheres, implemented in ROS2 Foxy. We employ a stereo-vision system to perform triangulation.

## Description

### camera_pkg

<p align = "left">
  <img src = "https://github.com/user-attachments/assets/c1d17894-cace-4382-adc5-472386a12e4b" width = 750 />
</p>

The **camera_publisher** publishes images to the topic **namespace/image/data_raw**. The geometry of camera is visualized using the **static_camera_tf2_broadcaster**.

***

### optical_marker_tracking

<p align = "left">
  <img src = "https://github.com/user-attachments/assets/c1d17894-cace-4382-adc5-472386a12e4b" width = 750 />
</p>

The **marker_detector** detects the markers in the image based on a color. The keypoints of the markers are matched using **marker_matcher**, and matched points are triangulated with the **marker_triangulator**. The **marker_tracker** tracks the points in the world coordinate system by seaching for the index of the marker the using predefined geometry of the markers. 

***

### optical_tracking_msgs

#### KeyPoint
```
geometry_msgs/Point point  # float64 x; float64 y; float64 z
float32[] descriptor
```

#### KeyPointMultiArray
```
builtin_interfaces/Time stamp
optical_tracking_msgs/KeyPoint[] data
```

## Usage

### Build
```
colcon build --packages-select optical_tracking_msgs --cmake-args -DCMAKE_BUILD_TYPE=Release
colcon build --packages-select camera_pkg optical_marker_tracking --cmake-args -DCMAKE_BUILD_TYPE=Release
```

### Run
```
ros2 launch optical_marker_tracking optical_marker_tracking.launch.py
```

or

```
ros2 launch optical_marker_tracking view_optical_marker_tracking.launch.py
```
