## Vision-Based Object Detection and Control in a ROS2 Gazebo Factory Conveyor System

In this project, a camera-integrated simulation pipeline has been developed within the ROS2 Jazzy environment using the Gazebo simulator and SDF-based world modeling to represent a factory conveyor (bent) system. The system enables real-time image streaming through a ROS2–Gazebo camera bridge, forming a foundation for advanced Computer Vision applications.

Building upon this infrastructure, the project incorporates Object Detection and a fusion of classical Image Processing techniques to analyze visual data from the simulated environment. These include grayscale transformation, region-of-interest (ROI) extraction, contour-based analysis, and feature-based filtering to enhance detection robustness. The integration of these methods allows for efficient detection, localization, and tracking of objects moving along the conveyor.

The architecture is designed to support scalable perception pipelines, where traditional image processing is combined with learning-based approaches (e.g., deep learning models such as YOLO) for improved accuracy and adaptability. The Gazebo environment is structured to be extensible, enabling future enhancements such as multi-object tracking, color-based classification, and robotic manipulation integration based on visual feedback.



## Running ROS2 Gazebo Simulation

```md
cd ~/ros2_ws

export GZ_VERSION=harmonic

source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash

gz sim ~/ros2_ws/src/factory_bent_project/my_factory_worlds/worlds/factory_conveyor.sdf
```

## ROS Gazebo Bridge

```md
cd ~/ros2_ws

export GZ_VERSION=harmonic

source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash

gz sim ~/ros2_ws/src/factory_bent_project/my_factory_worlds/worlds/factory_conveyor.sdf
```



## Run Camera Viewer 
```md
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash

python3 ~/ros2_ws/src/factory_bent_project/camera_viewer.py
```


## Run Object Detection 
```md
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash

python3 ~/ros2_ws/src/factory_bent_project/object_detection.py
```

## Gazebo GUI Fix

```md
sudo ufw allow in proto udp to 224.0.0.0/4
sudo ufw allow in proto udp from 224.0.0.0/4
```



## Screenshots of the Factory

<img width="1868" height="1048" alt="Screenshot from 2026-03-17 08-47-06" src="https://github.com/user-attachments/assets/f62bf372-6cb5-4005-8c1d-8128b0afe54f" />
<img width="1274" height="799" alt="Screenshot from 2026-03-17 08-20-26" src="https://github.com/user-attachments/assets/769a72d2-ca1a-497b-8227-0810296b7914" />

## Screenshots of the Object Detection

<img width="1268" height="568" alt="Screenshot from 2026-04-04 19-53-16" src="https://github.com/user-attachments/assets/c454b2ad-ccf7-4c57-bdb3-c2d0a14f06e9" />
<img width="1176" height="801" alt="Screenshot from 2026-04-04 19-57-24" src="https://github.com/user-attachments/assets/b719ec3f-31e1-4a27-94e3-29e40d180858" />
<img width="1531" height="899" alt="Screenshot from 2026-04-04 19-57-36" src="https://github.com/user-attachments/assets/068fad69-fd53-4c8d-a866-eb724353e733" />


