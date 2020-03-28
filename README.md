# ATOM Development

Here’s a guideline for how the development of the Autonomous Transportation On-board Modules (ATOM). Elaborating how the implementation shall take place, how the information shall flow, how the system shall behave in terms of the static and dynamic architecture. Moreover, the design constraints related to the problems/sub-problems on each of the development levels or phases.

In this document, the reference is the previously illustrated layout of the ATOM architecture, which can be found in the above figure. The basic architecture shows implementation of On-board modules in a multi-layered fashion, that enriches the system with the capabilities of the plug-and-play feature; hence allowing extendibility and upgradability one the level of the standalone modules.

Furthermore, the development guide will go through the environment, simulation, implementation and optimization of the software required to build such a complex system, not all the modules will be fully implemented, however, they will be available as interfaces for future development. The document will go as follows, the software environment designed to implement the architecture, the simulation environment developed, the design and development of the modules, and the integration on each level.

## Software Environment

In this section, we’re introducing the software environment design and implementation to illustrate the development in the mentioned architecture. This encourages us to specify some of the functional requirements that this software environment shall meet, which can be viewed in the following list

- Supports multiple build and execution configurations
- Supports multiple simulation environments
- Supports SIL, HIL, MIL and PIL
- Supports hardware and mechanical testing and validation
- Supports multiple frameworks
- Supports multiple networking platforms

Generally speaking, the design and architecture of the software environment it must as well follow the non-functional requirements shown below

- Flexible
- Maintainable
- Reusable

Nevertheless, the whole software environment part is considered a non-functional requirement to our systems, however, it has its own architectural concept that it shall meet in the development process, which will be enlisted shortly.

The build system of the whole project shall allow the system to be configurable in some sense, the system shall support programming in C/C++ and Python, extending the system to support more frameworks and programming paradigms.

Since the architecture states that the on-board modules shall communicate their data independently on whether they’re centralized or distributed, thus, modules require a lot of exchange of data and multiple platforms can support that decentralized configuration.

- Socket programming
- ROS multi-master
- Distributed design pattern(s) over web-sockets
- Vehicle network buses (LIN, CAN, Flexrey and Ethernet)

In this part, the networking platform configuration is necessary, hence using a pre-build configuration will reduce the runtime configuration dramatically and will allow for further extensions if a networking platform is introduced, it will also allow the system to be more dynamically predictable.

On the other hand, framework configuration will allow the system to run on different targets, and thus the simulation environment may be configured as some sort of target(s). Making it more maintainable and capable of enabling and disabling features which embraces the architecture’s concept.

Moreover, the configurable build configuration can be generated through a UI application making it more informative to the user and developer of the system. Also, an approach is to use design patterns in creating and destroying objects, so the system can be more dynamic and flexible during runtime.

## Simulation Environment

The simulation environment is a ROS-based simulation environment using gazebo and rviz for visualization, dynamics and physics of the whole system. Moreover, the system embeds some standard vehicles such as AUDIBOT and DBW_MKZ which are included as a submodule and added to the ROS environment, respectively. This environment basically mimics an actual vehicle with a set of sensors and control APIs.

Control APIs:

- Throttle command
- Brake command
- Steering command
- Gear command
- Turn signal command (DBW_MKZ only)

Sensor set:

- Groundtruth odometry (pose, orientation and twist)
- IMU -- X, Y and Z (Accelerations, Rotational velosities and Orientations) Including gravity
- LaserScan -- **TODO**
- Camera(s) -- **TODO**
- LiDAR -- **TODO**
- Sonar/Ultrasonic -- **TODO**
- Radar -- **TODO**

### ROS-specific info

The section describes the info and APIs implemented using ROS that will allo further communication with the vehicle.

#### Launch

#### Configurations

#### Rviz

#### Topics

#### Messages

#### Services

## Modules Design and Development

## Integration

## Manual

### Launching

| **Package**       | **Launch File**   | **Includes**  | **Functionality** |
|:-------------     |:-----------------:|:-------------:| -----------------:|
| *audibot_gazebo*  | `audibot_named_robot.launch`  | - | Spawns the multiple URDFs (Orange & Blue) of the audibot    |
| *audibot_gazebo*  | `audibot_named_robot_sens.launch` | - | Spawns the multiple URDFs (Orange & Blue) of the audibot with sensors    |
| *audibot_gazebo*  | `audibot_robot.launch`  | - | Spawns a single URDF of the audibot   |
| *audibot_gazebo*  | `audibot_robot_sens.launch`  | - | Spawns a single URDF of the audibot with sensors   |
| *audibot_gazebo*  | `single_vehicle_example.launch`  | - | Launches a single URDF of the audibot along with Gazebo world   |
| *audibot_gazebo*  | `single_vehicle_example_sens.launch`  | - | Launches a single URDF of the audibot with sensors along with Gazebo world   |
| *audibot_gazebo*  | `two_vehicle_example.launch`  | - | Launches the multiple URDFs (Orange & Blue) of the audibot along with Gazebo world   |
| *audibot_gazebo*  | `two_vehicle_example_sens.launch`  | - | Launches the multiple URDFs (Orange & Blue) of the audibot with sensors along with Gazebo world   |
