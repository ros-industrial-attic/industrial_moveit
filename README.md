# Industrial MoveIt

#### Prerequisites
- The latest version of `industrial_moveit` uses [OpenVDB](http://www.openvdb.org/) to generate space-efficient distance fields used for robot navigation (but not currently collision checking). 
  - Due to bugs in the mainline version of OpenVDB found in Ubuntu's trusty repository, we need to use a [PPA](https://launchpad.net/~levi-armstrong/+archive/ubuntu/openvdb)

```
sudo add-apt-repository ppa:levi-armstrong/openvdb
sudo apt-get update

sudo apt-get install libopenvdb-dev libopenvdb-tools
```


#### Stomp Core
- Build the workspace:
  - Cd into the catkin workspace directory and type the following command:
```
catkin_make --jobs=4
```

- Run Unit Test:
  - Cd into the catkin workspace directory and type the following command:
```
catkin_make run_tests_stomp_core_gtest
```
- Inspect the Unit test
  - *roscd* into the **stomp_core** package and go locate the "stomp_3dof.cpp" file inside the **test** directory.


#### Stomp Moveit Demo
- Run the demo
  - Run the demo.launch file
  ```
  roslaunch stomp_test_kr210_moveit_config demo.launch
  ```

  - In the Rviz Motion Planning planel, select **rail_start_pose** from the dropdwown menu under "Select Start State" and click **Update**
  - In Rviz, select **rail_end_pose** from the dropdwown menu under "Select Goal State" and click **Update**  
  - Click **Plan** in order to generate a motion plan.

- Locate the the stomp planner configuration file
  - **roscd** into the **stomp_test_kr210_moveit_config** package and locate the "stomp_config.yaml" file under the config directory

==============================================================================================
[ROS-Industrial][] move it meta-package.  See the [ROS wiki][] page for more information.  
[ROS-Industrial]: http://www.ros.org/wiki/Industrial
[ROS wiki]: http://ros.org/wiki/industrial_moveit

