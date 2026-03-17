# ROS2_Projects


## 1-)Factory Bent System Environment with Camera Integration

Factory Bent System focusing on development a realistic simulation environment in ROS2 and Gazebo to support computer vision–based object detection and counting in a factory setting. The system models a conveyor belt where various non-cylindrical items such as boxes, cartons, and packaged goods move at a consistent speed, independent of their physical properties like size or mass. A carefully designed conveyor structure, including side rails and support elements, ensures stable motion, while friction and velocity parameters are tuned to prevent irregular behavior. At the end of the conveyor, objects transition into a lower-positioned collection box, where they fall under gravity and come to rest, mimicking real-world industrial workflows. The environment also integrates an overhead RGB camera that captures images of the moving objects, enabling further processing through ROS2-based perception pipelines. This setup serves as a foundation for tasks such as object detection, tracking, and counting, which are essential in automated manufacturing and logistics systems. By combining physics-based simulation with perception capabilities, the project provides a scalable and flexible platform for experimenting with computer vision algorithms in a controlled yet realistic environment.


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


## Gazebo GUI Fix

```md
sudo ufw allow in proto udp to 224.0.0.0/4
sudo ufw allow in proto udp from 224.0.0.0/4
```



## Gazebo Environment

<img width="1868" height="1048" alt="Screenshot from 2026-03-17 08-47-06" src="https://github.com/user-attachments/assets/f62bf372-6cb5-4005-8c1d-8128b0afe54f" />
<img width="1274" height="799" alt="Screenshot from 2026-03-17 08-20-26" src="https://github.com/user-attachments/assets/769a72d2-ca1a-497b-8227-0810296b7914" />



## 2-)Factory Bent System Environment with Camera Integration
In this project Camera integration in ROS2 Jazzy environment using Gazebo simulator
and ".sdf" format packages for simulating factory bent system has been developed and 
camera bridge integration in the bent envrionment has been provided and it is ready 
for Computer Vision Application development and the Gazebo environment is being planned
to extended and improved for further bent simulations.

## Gazebo Environment


<img width="1246" height="589" alt="Screenshot from 2026-03-16 06-36-09" src="https://github.com/user-attachments/assets/70231df4-557b-4ec7-8401-feaf62b30b7c" />







