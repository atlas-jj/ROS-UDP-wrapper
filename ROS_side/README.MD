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
