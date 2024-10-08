# Color Marker Optical Tracking with ROS2 Foxy

<p align = "left">
  <img src = "https://github.com/user-attachments/assets/7e47dee8-fbfb-4c9b-be00-4a9c8930e251" width = 750 />
</p>

In this work, we present a method for estimating the position and orientation of an object using an optical tracking technique with color marker spheres, implemented in ROS2 Foxy. We employ a stereo-vision system to perform triangulation.

## Packages

### camera_pkg

<p align = "left">
  <img src = "https://github.com/user-attachments/assets/a46333dd-937b-41c6-abb7-91b843bae777" width = 750 />
</p>

+ The **camera_publisher** publishes images to the topic **namespace/image/data_raw**. The geometry of camera is visualized using the **static_camera_tf2_broadcaster**.

***

### optical_marker_tracking

<p align = "left">
  <img src = "https://github.com/user-attachments/assets/9b648e03-74c9-40ee-8b44-0e8ca32ea488" width = 750 />
</p>

+ The **marker_detector** detects the markers in the image based on a color. The keypoints of the markers are matched using **marker_matcher**, and matched points are triangulated with the **marker_triangulator**. The **marker_tracker** tracks the points in the world coordinate system by seaching for the index of the marker the using predefined geometry of the markers. As shown above, the object is visualized using the **marker_tf2_broadcaster** and **object_tf2_broadcaster**.

***

### optical_tracking_msgs

#### PointMultiArray
```
builtin_interfaces/Time stamp
geometry_msgs/Point[] data
```

## Usage

### Build
```
colcon build --packages-select optical_tracking_msgs --cmake-args -DCMAKE_BUILD_TYPE=Release
colcon build --packages-select camera_pkg optical_marker_tracking --cmake-args -DCMAKE_BUILD_TYPE=Release
```

### Run
```
ros2 launch camera_pkg camera_publish.launch.py
ros2 launch optical_marker_tracking optical_marker_tracking.launch.py
```

or

```
ros2 launch camera_pkg view_camera_publish.launch.py
ros2 launch optical_marker_tracking view_optical_marker_tracking.launch.py
```
