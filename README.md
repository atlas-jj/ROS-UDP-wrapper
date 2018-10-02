# ROS-UDP-Wrapper

**Designed for[ our live demo](https://www.youtube.com/watch?v=aLcw73dt_Oo " our live demo") in KUKA Innovation Award, Hannover Messe 2018.**

### ROS-UDP-Wrapper can be used as a general wrapper for non-ROS supported robots and sensors.

This repository contains example of:
# KUKA iiwa General Purpose Wrapper for communication in ROS

# Motivation
+ yes, there are existing wrappers.
+ However, we need a flexible structure that can implement our own codes freely.
+ It should support unlimited task types.
+ It should support task status feedback during execution
  + since some tasks may take a super long time by robtot side
  + while some tasks may need to get real-time feedback and adjust parameters continously.
+ It should prevent **Zombie Connections.**
+ It should use UDP other than TCP, but remove the **UDP datagram size limit.**
+ So I decide to design my own wrapper.


# Feature Overview
+ **Auto connection status detection**. Automatically reconnect if lost.
+ **Prevent Zombie UDP connection** by using heart beats to keep connection always online.
+ **No limitation on data package size** (UDP datagram size limits removed.)
+ **Bidirectional command status confirmation**, i.e., it's easy to know when the sent command execution starts in robot side, as well as, when the command execution ends in robot side.
+ Modular design. Seperate **Application**  -- **UDP Conn Thread** -- **Command Hanlder**, make it easy to** program complex tasks** in robot side.
----
# Design
## Command Types
+ Action commands.
+ Status query request commands.

## Bidirectional communication:
![](https://docs.google.com/drawings/d/snXYFKYB8wUKxz2zUzfQW_A/image?parent=e/2PACX-1vQkqbOY70SvV1cIwyy_BNk9uIMPHxMqaHk8rj7xJJIef7wKf72bSKkvI3XSO3PcLTBzsnpqDRdiZpjx&rev=410&h=195&w=624&ac=1)

----
# UDP Data Protocol
### 1. Request Commands:
+ **basic format**: \[command_string]:[command_type]:[data]
+ **command_string**: name of your command, e.g., grasp.
+ **command_type**:
  + 0: status query request command.
  + 1: action command.
  + ... : other user defined command types.
+ **data:** command content.
+ **Example**:
  + grasp: 1 : [grasp position + orientation]
  + the above command represents: Execute Grasping action using given position and orientation.
  
### 2. Response to Commands:
+ **basic format:** \[command_string] : [command_type] : [response_type] : [data]
+ **command_string, command_type:**
  + copy the same content from the request command.
  + for the purpose that the ROS side can differentiate handreds of responses in an asynchronous way.
+ ** response_type:**
  + 0: Response execution status.
    + Status written in [data], e.g., initialiated, processing, done,  etc..
  + 1: Response execution result data. 
    + Results written in [data], e.g., current joint states, etc..
+ **data**: Response content.
+ **Example:**
 + "grasp: 1 : 0 :success!"
 + This response is also published under ROS topic: /rr/response/grasp
    + Topic data is: "grasp: 1 : 0 :success!"

### 3. Design your own commands:
+  Follow the **Request Commands format**: \[command_string]:[command_type]:[data], e.g., grasp: 1 : [grasp position + orientation]
+ Publish the command as **std_msgs/string** under topic name: **/rr/commands**
+ Subscribe to topic: **/rr/response/[command_string]**, e.g., /rr/response/grasp
+ Done. Play time.


----

# ROS_side
### Send commands to robot/sensor side:
+ Design your own commands.
+ send command via topic name: **/rr/commands**, msg type: std_msgs/string.
+ subscribe to topic:  **/rr/response/[command_string]** to get robot side response.
+ yes, each type of command will have its own response topic.
  + they are generated dynamically by ROS_side node.

---


# Robot/sensor side (example codes using KUKA iiwa robot)
### ROS_Server
+ This package contains a UDP_Server_Thread class which creates a new thread and handles all UDP connection & communication processes.

### ROS_CMD_Handler
+ This package includes several command handler classes.
  + Thoses classes can handle different commands.
  + E.g., Grasph_handler class is designed to receive and response to Command Name: grasp.
  
### application
+ This is your main application code running in Robot/sensor side. For example, your are running a robot cooking task, the main entry of your project design is in application package.

### Other robots/sensors NOT using JAVA
+ You can rewrite the JAVA code in other OOP languange, for example, C++.
+ I will also add more examples when I encounter a particular case.

# Heartbeats:
+ yes, heartbeats are important to get rid of **Zombie UDP connection**. 
+ In our demo, when the ROS side sends command 'test'
  + The robot side must send 'echo_test' every 1 second to keep activating the UDP connection.

# Compile:
+ git clone the ROS_side codes.
  + catkin_make --only-pkg-with-deps ros_udp_wrapper
+ git clone the Robot_side codes.
  + import packages and codes into your Sunrise Project.

# Run:
+ Using default IP and port:
 + rosrun udp_ros_wrapper wrapper
+ define your IP and port:
 + rosrun udp_ros_wrapper wrapper 172.31.1.147 31002
+ Robot side: simply run your application.

# TODO
+ auto-reconnect in another thread.
+ Now you must start the UDP server on robot side firstly, then you can start ROS side. This ridiculous order should be removed.
+ Need to find a proper way to stop ROS node other than Ctrl + Z.
