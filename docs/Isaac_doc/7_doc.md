# RTX Lidar Sensors

First we need to add a Lidar sensor to the robot. Go to Create -> Isaac -> Sensors -> RTX Lidar -> Rotating.

import turtlebot and simple room based on this [tutorial](2_doc.md)

drag the Lidar prim under /World/turtlebot3_burger/base_scan. Zero out any displacement in the Transform fields inside the Property tab. The Lidar prim should now be overlapping with the scanning unit of the robot.

![alt text](img/10.png)

- **On Playback Tick**: This is the node responsible for triggering all the other nodes once Play is pressed.

- **ROS2 Context Node**: ROS2 uses DDS for its middleware communication. DDS uses Domain ID to allow for different logical networks operate independently even though they share a physical network. ROS 2 nodes on the same domain can freely discover and send messages to each other, while ROS 2 nodes on different domains cannot. ROS2 context node creates a context with a given Domain ID. It is set to 0 by default. If Use Domain ID Env Var is checked, it will import the `ROS_DOMAIN_ID` from the environment in which you launched the current instance of Isaac Sim.

- **Isaac Run One Simulation Frame**: This is the node to running the create render product pipeline once at the start to improve performance.

- **Isaac Create Render Product**: In the input camera target prim select the RTX Lidar created in step 2.

- **ROS2 RTX Lidar Helper**: This node will handle publishing of the laser scan message from the RTX Lidar. The input render product is obtained from the output of Isaac Create Render Product in step b.

- If you wish to also publish point cloud data, add another ROS2 RTX Lidar Helper node, and under input type select `point_cloud` and change the topic name to point_cloud. This node will handle publishing the point cloud from the RTX Lidar. The input render product is obtained from the output of Isaac Create Render Product in step b.

![alt text](img/14.png)
![alt text](img/11.png)
![alt text](img/12.png)


For RViZ visualization:

![alt text](img/13.png)

- Run RViZ2 (`rviz2`) in a sourced terminal.

- `ros2 param set /rviz use_sim_time true`

- The fixed frame name in Isaac Sim for the RTX Lidar is set to sim_lidar, update the Rviz side accordingly.

- Add LaserScan visualization and set topic to /scan

- Add PointCloud2 visualization and set topic to /point_cloud

## Next Step

[ROS2 Transform Trees and Odometry](8_doc.md)

## Previous Step

[ROS2 Clock](6_doc.md)