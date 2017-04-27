# Industrial MoveIt


#### Build
- Build the workspace:
  - Cd into the catkin workspace directory and type the following command:
```
catkin build
```

#### Unit Test
- Run all of industrial_moveit unit tests:
  - Cd into the catkin workspace directory and type the following command:
```
catkin run_tests 
```
- Run the stomp_core unit tests:
```
catkin run_tests stomp_core
```


#### Stomp Moveit Demo
- Run the demo
  - Run the demo.launch file
  ```
  roslaunch stomp_test_kr210_moveit_config demo.launch
  ```

  - In the Rviz Motion Planning planel, select **rail_start_pose** from the dropdwown menu under "Select Start State" and click **Update**
  - In Rviz, select **rail_end_pose** from the dropdwown menu under "Select Goal State" and click **Update**  
  - Click **Plan** in order to generate a motion plan.

#### Configure Stomp
- Locate the the stomp planner configuration file
  - **roscd** into the **stomp_test_kr210_moveit_config** package and locate the "stomp_config.yaml" file under the config directory
- Rerun demo.launch file and plan once again to see how the changes affect the planner's behavior. 

========================================================================  
[ROS-Industrial]: http://www.ros.org/wiki/Industrial  
[ROS wiki]: http://ros.org/wiki/industrial_moveit  
[ConstrainedIK]: http://wiki.ros.org/constrained_ik
[Stomp MoveIt!]: http://wiki.ros.org/stomp_moveit

