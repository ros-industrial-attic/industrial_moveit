# Industrial MoveIt

**Note:**: the STOMP packages were recently removed from this repository. They are hosted over at [ros-industrial/stomp_ros](https://github.com/ros-industrial/stomp_ros) now.


## ROS Distro Support

|         | Indigo | Jade | Kinetic |
|:-------:|:------:|:----:|:-------:|
| Branch  | [`indigo-devel`](https://github.com/ros-industrial/industrial_moveit/tree/indigo-devel) | [`indigo-devel`](https://github.com/ros-industrial/industrial_moveit/tree/indigo-devel) | [`kinetic-devel`](https://github.com/ros-industrial/industrial_moveit/tree/kinetic-devel) |
| Status  |  supported | supported |  supported |
| Version | [version](http://repositories.ros.org/status_page/ros_indigo_default.html?q=industrial_moveit) | [version](http://repositories.ros.org/status_page/ros_jade_default.html?q=industrial_moveit) | [version](http://repositories.ros.org/status_page/ros_kinetic_default.html?q=industrial_moveit) |

## Travis - Continuous Integration

Status: [![Build Status](https://travis-ci.com/ros-industrial/industrial_moveit.svg?branch=kinetic-devel)](https://travis-ci.com/ros-industrial/industrial_moveit)

## License

[![License](https://img.shields.io/badge/License-BSD%203--Clause-blue.svg)](https://opensource.org/licenses/BSD-3-Clause)
[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)

## ROS Buildfarm

|         | Indigo Source | Indigo Debian | Jade Source | Jade Debian |  Kinetic Source  |  Kinetic Debian |
|:-------:|:-------------------:|:-------------------:|:-------------------:|:-------------------:|:-------------------:|:-------------------:|
| industrial_moveit | [![not released](http://build.ros.org/buildStatus/icon?job=Isrc_uT__industrial_moveit__ubuntu_trusty__source)](http://build.ros.org/view/Isrc_uT/job/Isrc_uT__industrial_moveit__ubuntu_trusty__source/) | [![not released](http://build.ros.org/buildStatus/icon?job=Ibin_uT64__industrial_moveit__ubuntu_trusty_amd64__binary)](http://build.ros.org/view/Ibin_uT64/job/Ibin_uT64__industrial_moveit__ubuntu_trusty_amd64__binary/) | [![not released](http://build.ros.org/buildStatus/icon?job=Jsrc_uT__industrial_moveit__ubuntu_trusty__source)](http://build.ros.org/view/Jsrc_uT/job/Jsrc_uT__industrial_moveit__ubuntu_trusty__source/) | [![not released](http://build.ros.org/buildStatus/icon?job=Jbin_uT64__industrial_moveit__ubuntu_trusty_amd64__binary)](http://build.ros.org/view/Jbin_uT64/job/Jbin_uT64__industrial_moveit__ubuntu_trusty_amd64__binary/) | [![not released](http://build.ros.org/buildStatus/icon?job=Ksrc_uX__industrial_moveit__ubuntu_xenial__source)](http://build.ros.org/view/Ksrc_uX/job/Ksrc_uX__industrial_moveit__ubuntu_xenial__source/) | [![not released](http://build.ros.org/buildStatus/icon?job=Kbin_uX64__industrial_moveit__ubuntu_xenial_amd64__binary)](http://build.ros.org/view/Kbin_uX64/job/Kbin_uX64__industrial_moveit__ubuntu_xenial_amd64__binary/) |

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
  roslaunch industrial_moveit_test_moveit_config demo.launch
  ```

  - In the Rviz Motion Planning planel, select **rail_start_pose** from the dropdown menu under "Select Start State" and click **Update**
  - In Rviz, select **rail_end_pose** from the dropdown menu under "Select Goal State" and click **Update**  
  - Click **Plan** in order to generate a motion plan.

#### Configure Stomp
- Locate the the stomp planner configuration file
  - **roscd** into the **industrial_moveit_test_moveit_config** package and locate the "stomp_config.yaml" file under the config directory
- Rerun demo.launch file and plan once again to see how the changes affect the planner's behavior.

#### Seeding Stomp
The STOMP planner works through optimization: it starts with a given trajectory, called the ***seed***, and iteratively attempts to improve it. This seed is set:
 1. By default, it is set to the joint interpolated path between the start and end joint configurations.
 2. If you wish, you can set your own seed trajectory.

The `StompPlanner` class works off of the `moveit_msgs/MotionPlanRequest` message type which does not provide an interface for seeds. Until that is added, we bastardize the unused `MotionPlanRequest::trajectory_constraints` field to serve this purpose. Use the `StompPlanner::encodeSeedTrajectory(const trajectory_msgs::JointTrajectory& seed)` static function to do this:

```c++

StompPlanner planner = makeStompPlanner(); // However you initialize
planning_interface::MotionPlanRequest request;

// set your nominal goals, start conditions, etc...

trajectory_msgs::JointTrajectory seed_traj; // Look up your seed traj
request.trajectory_constraints = StompPlanner::encodeSeedTrajectory(seed_traj);

// Call the planning service or the planner itself
planner.setMotionPlanRequest(request)

MotionPlanResponse res;
planner.solve(res);
``` 
There is no current way to set this through the MoveGroupInterface class. 

========================================================================  
[ROS-Industrial]: http://www.ros.org/wiki/Industrial  
[ROS wiki]: http://ros.org/wiki/industrial_moveit  
[ConstrainedIK]: http://wiki.ros.org/constrained_ik
[Stomp MoveIt!]: http://wiki.ros.org/stomp_moveit

