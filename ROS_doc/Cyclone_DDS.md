# Cyclone DDS Installation

## Have rosdep Installed

```bash
# if you are using rosdep with ROS
sudo apt-get install python3-rosdep
# if you are using rosdep outside of ROS
pip install rosdep
```

### rosdep operation
```bash
sudo rosdep init
rosdep update
```

Finally, we can run `rosdep install` to install dependencies. Typically, this is run over a workspace with many packages in a single call to install all dependencies. A call for that would appear as the following, if in the root of the workspace with directory `src` containing source code.

```bash
rosdep install --from-paths src -y --ignore-src
```

## Install Packages

Install from ROS 2 apt repository

```bash
sudo apt install ros-humble-rmw-cyclonedds-cpp
```

Switch from other rmw to rmw_cyclonedds by specifying the environment variable.`

```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

