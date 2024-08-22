# Color Marker Optical Tracking with ROS2 Foxy

In this work, we present a method for estimating the position and orientation of an object using an optical tracking technique with color marker spheres, implemented in ROS2 Foxy. We employ a stereo-vision system to perform triangulation.

## Description

### camera_pkg

<p align = "left">
  <img src = "https://github.com/user-attachments/assets/c1d17894-cace-4382-adc5-472386a12e4b" width = 750 />
</p>

The **camera_publisher** publishes images to the topic **namespace/image/data_raw**. The geometry of camera is visualized using the **static_camera_tf2_broadcaster**.
