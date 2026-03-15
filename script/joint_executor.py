#!/usr/bin/env python3
import time
from typing import List, Optional, Sequence, Tuple

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory

try:
    import rtde_control
    import rtde_receive
except ImportError as exc:
    raise RuntimeError(
        "Failed to import ur_rtde. Install it with `pip install --user ur_rtde` "
        "or the system package `librtde`/`librtde-dev`."
    ) from exc


TrajectorySample = Tuple[float, List[float]]


class UrRtdeJointExecutor(Node):
    def __init__(self) -> None:
        super().__init__('ur_rtde_joint_executor_node')

        self.robot_ip = str(self.declare_parameter('robot_ip', '192.168.125.6').value)
        self.command_topic = str(self.declare_parameter('command_topic', '/joint_commands').value)
        self.joint_state_topic = str(self.declare_parameter('joint_state_topic', '/joint_states').value)
        self.servo_frequency = float(self.declare_parameter('servo_frequency', 125.0).value)
        self.lookahead_time = float(self.declare_parameter('lookahead_time', 0.1).value)
        self.gain = float(self.declare_parameter('gain', 300.0).value)
        self.velocity = float(self.declare_parameter('velocity', 0.5).value)
        self.acceleration = float(self.declare_parameter('acceleration', 0.5).value)
        self.receive_frequency = float(self.declare_parameter('receive_frequency', 125.0).value)

        self.joint_names = [
            'shoulder_pan_joint',
            'shoulder_lift_joint',
            'elbow_joint',
            'wrist_1_joint',
            'wrist_2_joint',
            'wrist_3_joint',
        ]
        self.n = len(self.joint_names)
        self.dt = 1.0 / self.servo_frequency

        self.trajectory: List[TrajectorySample] = []
        self.trajectory_start_time: Optional[float] = None
        self.last_target: Optional[List[float]] = None

        self.joint_state_pub = self.create_publisher(JointState, self.joint_state_topic, 20)
        self.create_subscription(JointTrajectory, self.command_topic, self.trajectory_callback, 10)

        self.rtde_c = rtde_control.RTDEControlInterface(self.robot_ip)
        self.rtde_r = rtde_receive.RTDEReceiveInterface(self.robot_ip, self.receive_frequency)

        self.control_timer = self.create_timer(self.dt, self.control_loop)

        self.get_logger().info(
            f'UR RTDE executor ready. ip={self.robot_ip}, cmd={self.command_topic}, '
            f'state={self.joint_state_topic}, rate={self.servo_frequency:.1f}Hz'
        )

    def trajectory_callback(self, msg: JointTrajectory) -> None:
        if not msg.points:
            self.get_logger().warn('Received empty trajectory, ignored.')
            return

        name_to_index = {name: idx for idx, name in enumerate(msg.joint_names)}
        if msg.joint_names:
            missing = [name for name in self.joint_names if name not in name_to_index]
            if missing:
                self.get_logger().warn(
                    f'Received trajectory missing joints {missing}, ignored.'
                )
                return

        new_traj: List[TrajectorySample] = []
        for point in msg.points:
            positions = self.extract_positions(point.positions, name_to_index if msg.joint_names else None)
            if positions is None:
                continue
            point_time = float(point.time_from_start.sec) + float(point.time_from_start.nanosec) * 1e-9
            new_traj.append((point_time, positions))

        if not new_traj:
            self.get_logger().warn('Received trajectory without valid joint positions, ignored.')
            return

        new_traj.sort(key=lambda item: item[0])
        self.trajectory = new_traj
        self.trajectory_start_time = time.monotonic()
        self.last_target = list(new_traj[0][1])

        self.get_logger().info(
            f'Received /joint_commands with {len(new_traj)} points over {new_traj[-1][0]:.3f}s.'
        )

    def extract_positions(
        self, positions: Sequence[float], name_to_index: Optional[dict]
    ) -> Optional[List[float]]:
        if name_to_index is None:
            if len(positions) < self.n:
                return None
            return [float(positions[i]) for i in range(self.n)]

        ordered = []
        for joint_name in self.joint_names:
            src_index = name_to_index[joint_name]
            if src_index >= len(positions):
                return None
            ordered.append(float(positions[src_index]))
        return ordered

    def sample_trajectory(self, elapsed: float) -> List[float]:
        if not self.trajectory:
            return self.last_target if self.last_target is not None else self.read_actual_q()

        if elapsed <= self.trajectory[0][0]:
            return list(self.trajectory[0][1])

        if elapsed >= self.trajectory[-1][0]:
            return list(self.trajectory[-1][1])

        for idx in range(1, len(self.trajectory)):
            t1, q1 = self.trajectory[idx]
            if elapsed <= t1:
                t0, q0 = self.trajectory[idx - 1]
                dt = t1 - t0
                if dt <= 1e-9:
                    return list(q1)
                alpha = (elapsed - t0) / dt
                return [
                    float((1.0 - alpha) * q0[j] + alpha * q1[j])
                    for j in range(self.n)
                ]

        return list(self.trajectory[-1][1])

    def read_actual_q(self) -> List[float]:
        return [float(q) for q in self.rtde_r.getActualQ()]

    def publish_joint_state(self) -> List[float]:
        actual_q = self.read_actual_q()
        actual_qd = [float(qd) for qd in self.rtde_r.getActualQd()]
        actual_current = [float(current) for current in self.rtde_r.getActualCurrent()]

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(self.joint_names)
        msg.position = actual_q
        msg.velocity = actual_qd
        msg.effort = actual_current
        self.joint_state_pub.publish(msg)
        return actual_q

    def control_loop(self) -> None:
        try:
            actual_q = self.publish_joint_state()

            if self.trajectory and self.trajectory_start_time is not None:
                elapsed = time.monotonic() - self.trajectory_start_time
                target = self.sample_trajectory(elapsed)
                if elapsed >= self.trajectory[-1][0]:
                    self.last_target = list(target)
                    self.trajectory = []
                    self.trajectory_start_time = None
            else:
                target = self.last_target if self.last_target is not None else actual_q

            self.rtde_c.servoJ(
                target,
                self.velocity,
                self.acceleration,
                self.dt,
                self.lookahead_time,
                self.gain,
            )
            self.last_target = list(target)
        except Exception as exc:  # pylint: disable=broad-except
            self.get_logger().error(f'Control loop failed: {exc}')

    def destroy_node(self) -> bool:
        try:
            self.rtde_c.servoStop()
            self.rtde_c.stopScript()
        except Exception:  # pylint: disable=broad-except
            pass
        return super().destroy_node()


def main() -> None:
    rclpy.init()
    node = UrRtdeJointExecutor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
