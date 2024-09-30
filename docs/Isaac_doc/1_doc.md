# Installation

[Reference](https://docs.omniverse.nvidia.com/isaacsim/latest/installation/install_ros.html)

## Install ROS 2

Navigate to [ROS 2 Installation Documentation](../ROS_doc/1_doc.md) to install ROS 2.

ROS 2 can be sourced before Isaac Sim is run, allowing Isaac Sim ROS2 bridge to load the ROS2 libraries of your system.

##  Enabling the ROS Bridge Extension

Create a file named `fastdds.xml` under `~/.ros/`:
```xml
<?xml version="1.0" encoding="UTF-8" ?>

<license>Copyright (c) 2022-2024, NVIDIA CORPORATION.  All rights reserved.
NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto.  Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.</license>

<profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles" >
    <transport_descriptors>
        <transport_descriptor>
            <transport_id>UdpTransport</transport_id>
            <type>UDPv4</type>
        </transport_descriptor>
    </transport_descriptors>

    <participant profile_name="udp_transport_profile" is_default_profile="true">
        <rtps>
            <userTransports>
                <transport_id>UdpTransport</transport_id>
            </userTransports>
            <useBuiltinTransports>false</useBuiltinTransports>
        </rtps>
    </participant>
</profiles>
```

Run `export FASTRTPS_DEFAULT_PROFILES_FILE=~/.ros/fastdds.xml` in the terminals that will use ROS 2 functions . You must also set it under ~Extra Args" when launching Isaac Sim from the Nucleus Launcher.

Source your ROS 2 installation and workspace before launching Isaac Sim.

### Enable Extension

Omniverse-launcher > Library > Isaac Sim > Window > Extensions > search `ros*_bridge` and enable one.

## Next Step

To start using Omniverse Isaac Sim with ROS 2, complete the ROS 2 Tutorial series starting with [URDF Import: Turtlebot](2_doc.md)


<!-- ### Setup the Isaac Sim ROS Workspaces

Install Rocker

```bash
sudo apt-get install python3-rocker
```

Start the ROS container using Rocker.

```bash
rocker --nvidia --x11 --privileged --network host  --name <container name> osrf/ros:humble-desktop-full-jammy
``` -->

<!-- Clone the Isaac Sim ROS Workspace Repository from https://github.com/isaac-sim/IsaacSim-ros_workspaces.

```bash
git clone https://github.com/isaac-sim/IsaacSim-ros_workspaces.git
cd IsaacSim-ros_workspaces
cd humble_ws
```

Set the environment variable for `fastdds.xml`. **You must also set it under “Extra Args” when launching Isaac Sim from the NVIDIA Omniverse Launcher.** 
```bash
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/user1/Desktop/isaac_ros_git/IsaacSim-ros_workspaces/humble_ws/fastdds.xml
``` -->

<!-- Running ROS2 Bridge using Cyclone DDS: -->

<!-- Follow [Cyclone DDS Installation](../ROS_doc/Cyclone_DDS.md) -->

<!-- Before Running Isaac Sim, make sure to set the `RMW_IMPLEMENTATION` environment variable as shown below. 
```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

Source your native ROS 2 workspace (if not sourced yet).

```bash
source /opt/ros/humble/setup.bash
```

Install additional packages.

```bash
# For rosdep install command
sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
# For colcon build command
sudo apt install python3-colcon-common-extensions
```

Resolve any package dependencies from the root of the ROS 2 workspace.

```bash
# cd humble_ws
rosdep install -i --from-path src --rosdistro humble -y
```

Install Docker. Check if docker is installed.

```bash
docker --version
```

Install Docker if it is not installed yet.

```bash
sudo apt update
```

```bash
sudo apt install apt-transport-https ca-certificates curl software-properties-common
curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo apt-key add -
sudo add-apt-repository "deb [arch=amd64] https://download.docker.com/linux/ubuntu $(lsb_release -cs) stable"
```

```bash
sudo apt update
```

```bash
apt-cache policy docker-ce
sudo apt install docker-ce
```

```bash
sudo usermod -aG docker ${USER}
su - ${USER}
```

```bash
docker --version
```

Build the workspace.

```bash
colcon build

# or for docker:

cd ..
./build_humble.sh
``` -->