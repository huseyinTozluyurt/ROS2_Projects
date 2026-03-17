# Factory Bent System Environment with Camera Integration
In this project Camera integration in ROS2 Jazzy environment using Gazebo simulator
and ".sdf" format packages for simulating factory bent system has been developed and 
camera bridge integration in the bent envrionment has been provided and it is ready 
for Computer Vision Application development and the Gazebo environment is being planned
to extended and improved for further bent simulations.


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



## Screenshots

<img width="1868" height="1048" alt="Screenshot from 2026-03-17 08-47-06" src="https://github.com/user-attachments/assets/f62bf372-6cb5-4005-8c1d-8128b0afe54f" />
<img width="1274" height="799" alt="Screenshot from 2026-03-17 08-20-26" src="https://github.com/user-attachments/assets/769a72d2-ca1a-497b-8227-0810296b7914" />


