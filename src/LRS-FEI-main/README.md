# LRS-FEI

## Setup simulation with LRS-Ubuntu image
1. Open terminator with LRS layout. 
2. In 1st terminal launch gazebo: `gazebo <path_to_world>/fei_lrs_gazebo.world`
3. In 2nd terminal launch ArduPilot SITL: 
```
cd ardupilot/ArduCopter
sim_vehicle.py -f gazebo-iris --console -l 48.15084570555732,17.072729745416016,150,0
```
4. Launch mavros `ros2 run mavros mavros_node --ros-args -p fcu_url:=udp://127.0.0.1:14551@14555`

## Simple takeoff and position control in GUIDED mode

1. Set mode.
2. Arm. 
3. Take off. 
4. Position control by waypoints.

```
ros2 service call /mavros/set_mode mavros_msgs/srv/SetMode "{base_mode: 0, custom_mode: GUIDED}"
ros2 service call /mavros/cmd/arming mavros_msgs/srv/CommandBool "{value: True}"
ros2 service call /mavros/cmd/takeoff mavros_msgs/srv/CommandTOL "{min_pitch: 0, yaw: 90, altitude: 2}"

ros2 topic pub /mavros/setpoint_raw/local mavros_msgs/msg/PositionTarget '{header: {stamp: {sec: 0, nanosec: 0}, frame_id: "",}, coordinate_frame: 1, type_mask: 0, position: {x: 0.0, y: 0.0, z: 0.0}, velocity: {x: 0.0, y: 0.0, z: 0.0}, acceleration_or_force: {x: 0.0, y: 0.0, z: 0.0}, yaw: 0.0, yaw_rate: 0.0}'
```

In the last command you need to set coordinate_frame and type_mask.
Refer to mavlink manual, that will be used in the class often.

https://mavlink.io/en/messages/common.html#POSITION_TARGET_LOCAL_NED

## 2. Task

### Updated path planning

In task 1 we were using the given flood fill algorithm for path planning. In the second task we had to decide between RRT or A* path planning, we decided that we are going to use A*.

#### Theory

https://www.geeksforgeeks.org/a-search-algorithm/

### Circle

We created a python script called: ```circle_move.py```. It is called when a ```circle``` command is called in the during completing tasks. After receiving the circle task, the drone will go forward, and do a circle. The circle is declared that way, that it is got 7 points declared, and the drone will go to all of the points one by one, after completing the full circle, the drone will go back to the original position, where the circle command started, and continues completing task.

### Interrupt

#### Basic information about interrupt

We created a subscriber, which is subscribing to a topic ```/mavros/interrupt```, and the message type is ```std_msgs/msg/Int32```. Until the interrupt command is enabled, the drone will stay in one position, after it receives a continue command, the drone will continue the mission. If it receives a value, which is not declared in the logic (other than 0 and 1) in the terminal we get a message that the interrupt command is incorrect.

Interrupt values: 

 * 0 - Continue
 * 1 - Stop/Interrupt

#### To call interrupt

If you want to call interrupt use the the value: 1 for data, if you want to call continue, then use the value: 0 for data.
```shell
ros2 topic pub /mavros/interrupt std_msgs/msg/Int32 '{data: 1}' -1 
```
