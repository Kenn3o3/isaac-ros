# URDF Import: Turtlebot

[Reference](https://docs.omniverse.nvidia.com/isaacsim/latest/ros2_tutorials/tutorial_ros2_turtlebot.html#urdf-import-turtlebot)

## Importing TurtleBot URDF

```bash
git clone -b humble-devel https://github.com/ROBOTIS-GIT/turtlebot3.git turtlebot3
```

The urdf for Turtlebot3 Burger is located in `turtlebot3/turtlebot3_description/urdf/turtlebot3_burger.urdf`

Launch **Isaac Sim**

In **Content** tab inside Isaac Sim:

Drag `localhost/NVIDIA/Assets/Isaac/4.0/Isaac/Environments/Simple_Room/simple_room.usd` to the space

Isaac Utils > Workflows > URDF Importer:

- Unselect `Fix Base Link`
- Set **Joint Drive Type** to `Velocity`
- In **Import File** choose the `turtlebot3_burger.urdf` file
- Select **Output Directory** to `Desktop`
- Click **Import**

## Tune the Robot

The URDF importer automatically imports material, physical, and joint properties whenever it is available and have matching categories in Omniverse Isaac Sim. 

In cases there are no available or matching categories, or if the units are different between the two systems, what gets automatically filled in may not be accurate and changes the robot’s behavior. Here are some properties that can be tuned to correct the robot’s behavior.

**Frictional Properties**

- If your robot’s wheels are slipping, try changing the friction coefficients of the wheels and potentially the ground as well following steps 3.4.2 in Add Simple Objects

**Physical Properties**

- If no explicit mass or inertial properties are given, the physics engine will estimate them from the geometry mesh. To update the mass and inertial properties, find the prim that contains the rigid body for the given link (You can verify this by finding “Physics > Rigid Body” under its property tab). If it already has a “Mass” category under its Physics property tab, modify them accordingly. If there isn’t already a “Mass” category, you can add it by clicking on the `+Add` button on top of the Propery tab, and select “Physics > Mass”.

**Joint Properties**

- If your robot is oscillating at the joint or moving too slow, take a look at the stiffness and damping parameters for the joints. High stiffness makes the joints snap faster and harder to the desire target, and higher damping smoothes but also slows down the joint’s movement to target. For pure position drives, set relatively high stiffness and low damping. For velocity drives, stiffness must be set to zero with a non-zero damping.

## Next Step

[Driving TurtleBot via ROS2 messages](3_doc.md)

## Previous Step

[Installation](1_doc.md)