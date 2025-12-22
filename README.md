# ur-wrapper

## 1 Setup PC

1. Connect the UR control box directly to the remote PC with an ethernet cable. On the remote PC, turn off all network devices except the “wired connection”, e.g. turn off wifi. Open Network Settings and create a new Wired connection with these settings. You may want to name this new connection UR or something similar:

```yaml
IPv4
Manual
Address: 192.168.125.126
Netmask: 255.255.255.0
Gateway: 192.168.125.1
DNS: 192.168.125.1
```

2. Verify the connection from the PC with e.g. ping.

```shell
ping 192.168.125.6
```

**推荐使用Sec 3中的UR_RTDE，方便且高性能**

## 2 Universal_Robots_ROS2_Driver

[GitHub Page](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver)
[Documentation](https://docs.universal-robots.com/Universal_Robots_ROS2_Documentation/index.html)

1. Install the driver

```shell
sudo apt-get install ros-${ROS_DISTRO}-ur
sudo apt install ros-${ROS_DISTRO}-ros2-control \
                 ros-${ROS_DISTRO}-ros2-controllers \
                 ros-${ROS_DISTRO}-gazebo-ros2-control
```

2. 开启上位机ros2节点，UR可以给上位机发话题（可以读到关节角）

```shell
ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur10 robot_ip:=192.168.125.6
```

3. 在UR示教器上加载程序（上位机可以控制UR）并运行

```shell
ros2 action send_goal /scaled_joint_trajectory_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory "{
  trajectory: {
    joint_names: [shoulder_pan_joint, shoulder_lift_joint, elbow_joint, wrist_1_joint, wrist_2_joint, wrist_3_joint],
    points: [{
      positions: [0.0, -1.57, 0.0, -1.57, 0.0, 0.0], 
      time_from_start: {sec: 5}
    }]
  }
}"
```

4. 读取关节角度

```shell
ros2 topic echo /joint_states
```

TIP: joint_states话题下顺序是2-6,最后是1，注意！

## 3 UR_RTDE

[Documentation](https://sdurobotics.gitlab.io/ur_rtde/index.html)

1. Install the library

```shell
sudo add-apt-repository ppa:sdurobotics/ur-rtde
sudo apt-get update
sudo apt install librtde librtde-devhttps://sdurobotics.gitlab.io/ur_rtde/index.html)
```

2. If you only want to the use the Python interface, you can install ur_rtde through pip:

```shell
pip install --user ur_rtde
```

## 4 Universal_Robots_Client_Library

[GitHub Page](https://github.com/UniversalRobots/Universal_Robots_Client_Library)

[Documentation](https://docs.universal-robots.com/Universal_Robots_ROS2_Documentation/doc/ur_client_library/doc/index.html#ur-client-library)

1. Install the driver

```shell
sudo apt install ros-$ROS_DISTRO-ur-client-library
```

2. 编译功能包

```shell
ros2 run ur-wrapper home # 插补到指定关节角
ros2 run ur-wrapper trajectory # 轨迹跟踪，轨迹的规划频率低于125Hz，轨迹点频率125Hz
ros2 run ur-wrapper p2p # 30Hz的点到点任务，使用内置样条规划器
```
