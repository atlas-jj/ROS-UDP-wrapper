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
