# ur_publisher

[Universal_Robots_ROS2_Driver](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver)

## Setup

1. Install the driver

```shell
sudo apt-get install ros-humble-ur
```

2. Connect the UR control box directly to the remote PC with an ethernet cable. On the remote PC, turn off all network devices except the “wired connection”, e.g. turn off wifi. Open Network Settings and create a new Wired connection with these settings. You may want to name this new connection UR or something similar:

```yaml
IPv4
Manual
Address: 192.168.125.126
Netmask: 255.255.255.0
Gateway: 192.168.125.2
```

3. Verify the connection from the PC with e.g. ping.

```shell
ping 192.168.1.102
```

## Usage

```shell
ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur10 robot_ip:=192.168.125.6
```

