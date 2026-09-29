#!/bin/sh

echo "Creating new world package..."

WORLD_NAME=$1
USER_NAME=$2
USER_EMAIL=$3

if [ -z "${1}" ]; then
  echo "\e[33mWarning:\e[0m world name not found, using default world name"
  WORLD_NAME="test"
fi


if [ -z "${2}" ]; then
  echo "\e[33mWarning:\e[0m user name not found, using default user name"
  USER_NAME="user"
fi

if [ -z "${3}" ]; then
  echo "\e[33mWarning:\e[0m user email not found, using default user email"
  USER_EMAIL="user@todo.todo"
fi

echo "-------------------------------"
echo "World name: $WORLD_NAME"
echo "User name: $USER_NAME"
echo "User email: $USER_EMAIL"
echo "-------------------------------"


create_cmakelist()
{
echo "Creating CMakeLists.txt"

touch CMakeLists.txt

cat <<END >CMakeLists.txt
cmake_minimum_required(VERSION 3.0.2)
project(${WORLD_NAME}_world)

find_package(catkin REQUIRED COMPONENTS
    gazebo_ros
)

catkin_package(
)

include_directories(
)

END
}


create_package()
{

echo "Creating package.xml"

touch package.xml

cat <<END >package.xml
<?xml version="1.0"?>
<package format="2">
  <name>${WORLD_NAME}_world</name>
  <version>0.0.0</version>
  <description>${WORLD_NAME}_world package</description>

  <maintainer email="${USER_EMAIL}">$USER_NAME</maintainer>

  <license>TODO</license>

  <buildtool_depend>catkin</buildtool_depend>

  <build_depend>gazebo_ros</build_depend>
  <build_export_depend>gazebo_ros</build_export_depend>
  <exec_depend>gazebo_ros</exec_depend>

  <export>
    <gazebo_ros gazebo_model_path="\${prefix}/models"/>
    <gazebo_ros gazebo_media_path="\${prefix}/worlds"/>
  </export>

</package>
END
}


create_base_world(){

echo "Creating base world"

touch $WORLD_NAME.world

cat <<END >$WORLD_NAME.world
<sdf version='1.6'>
  <world name='default'>
    <light name='sun' type='directional'>
      <cast_shadows>1</cast_shadows>
      <pose frame=''>0 0 10 0 -0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <attenuation>
        <range>1000</range>
        <constant>0.9</constant>
        <linear>0.01</linear>
        <quadratic>0.001</quadratic>
      </attenuation>
      <direction>-0.5 0.1 -0.9</direction>
    </light>
    <model name='ground_plane'>
      <static>1</static>
      <link name='link'>
        <collision name='collision'>
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
          <surface>
            <contact>
              <collide_bitmask>65535</collide_bitmask>
              <ode/>
            </contact>
            <friction>
              <ode>
                <mu>100</mu>
                <mu2>50</mu2>
              </ode>
              <torsional>
                <ode/>
              </torsional>
            </friction>
            <bounce/>
          </surface>
          <max_contacts>10</max_contacts>
        </collision>
        <visual name='visual'>
          <cast_shadows>0</cast_shadows>
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
          <material>
            <script>
              <uri>file://media/materials/scripts/gazebo.material</uri>
              <name>Gazebo/Grey</name>
            </script>
          </material>
        </visual>
        <self_collide>0</self_collide>
        <enable_wind>0</enable_wind>
        <kinematic>0</kinematic>
      </link>
    </model>
    <gravity>0 0 -9.8</gravity>
    <magnetic_field>6e-06 2.3e-05 -4.2e-05</magnetic_field>
    <atmosphere type='adiabatic'/>
    <physics name='default_physics' default='0' type='ode'>
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
    </physics>
    <scene>
      <ambient>0.4 0.4 0.4 1</ambient>
      <background>0.7 0.7 0.7 1</background>
      <shadows>1</shadows>
    </scene>
    <wind/>
    <spherical_coordinates>
      <surface_model>EARTH_WGS84</surface_model>
      <latitude_deg>0</latitude_deg>
      <longitude_deg>0</longitude_deg>
      <elevation>0</elevation>
      <heading_deg>0</heading_deg>
    </spherical_coordinates>
    <state world_name='default'>
      <sim_time>11 186000000</sim_time>
      <real_time>11 218305265</real_time>
      <wall_time>1614772560 970811890</wall_time>
      <iterations>11186</iterations>
      <model name='ground_plane'>
        <pose frame=''>0 0 0 0 -0 0</pose>
        <scale>1 1 1</scale>
        <link name='link'>
          <pose frame=''>0 0 0 0 -0 0</pose>
          <velocity>0 0 0 0 -0 0</velocity>
          <acceleration>0 0 0 0 -0 0</acceleration>
          <wrench>0 0 0 0 -0 0</wrench>
        </link>
      </model>
      <light name='sun'>
        <pose frame=''>0 0 10 0 -0 0</pose>
      </light>
    </state>
    <gui fullscreen='0'>
      <camera name='user_camera'>
        <pose frame=''>5 -5 2 0 0.275643 2.35619</pose>
        <view_controller>orbit</view_controller>
        <projection_type>perspective</projection_type>
      </camera>
    </gui>
  </world>
</sdf>
END
}

create_launch(){

echo "Creating launch file"

touch ${WORLD_NAME}_world.launch

cat <<END >${WORLD_NAME}_world.launch
<?xml version="1.0"?>
<launch>

    <arg name="gazebo_world" default="${WORLD_NAME}.world" />
    <arg name="gazebo_gui"   default="true"               />	

    <include file="\$(find gazebo_ros)/launch/empty_world.launch">
        <arg name="world_name"    value="\$(arg gazebo_world)" /> 
        <arg name="debug"         value="false"               />
        <arg name="paused"        value="false"               />
        <arg name="use_sim_time"  value="true"                />
        <arg name="headless"      value="false"               /> 
        <arg name="gui"           value="\$(arg gazebo_gui)"   /> 
    </include>

</launch>
END
}



mkdir ${WORLD_NAME}_world
cd ${WORLD_NAME}_world
create_cmakelist
create_package
mkdir worlds
cd worlds
create_base_world
cd ..
mkdir launch
cd launch
create_launch
cd ..
mkdir models

