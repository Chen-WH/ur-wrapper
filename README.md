# ur_publisher

[Universal_Robots_ROS2_Driver](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver)
[UR ROS2 Driver Doc](https://docs.universal-robots.com/Universal_Robots_ROS2_Documentation/index.html)

## Setup

1. Install the driver

```shell
sudo apt-get install ros-${ROS_DISTRO}-ur
sudo apt install ros-${ROS_DISTRO}-ros2-control \
                 ros-${ROS_DISTRO}-ros2-controllers \
                 ros-${ROS_DISTRO}-gazebo-ros2-control
```

2. Connect the UR control box directly to the remote PC with an ethernet cable. On the remote PC, turn off all network devices except the “wired connection”, e.g. turn off wifi. Open Network Settings and create a new Wired connection with these settings. You may want to name this new connection UR or something similar:

```yaml
IPv4
Manual
Address: 192.168.125.126
Netmask: 255.255.255.0
Gateway: 192.168.125.1
DNS: 192.168.125.1
```

3. Verify the connection from the PC with e.g. ping.

```shell
ping 192.168.125.6
```

## Usage

1. 开启上位机ros2节点

```shell
ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur10 robot_ip:=192.168.125.6
```

3. 在UR示教器上加载程序并运行

```shell
ros2 action send_goal /scaled_joint_trajectory_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory "{
  trajectory: {
    joint_names: [shoulder_pan_joint, shoulder_lift_joint, elbow_joint, wrist_1_joint, wrist_2_joint, wrist_3_joint],
    points: [{
      positions: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], 
      time_from_start: {sec: 5}
    }]
  }
}"
```

## ToDo

1. joint_states话题下顺序是2-6,最后是1，封装时候调整一下顺序