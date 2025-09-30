#!/usr/bin/env python3
"""Streaming + replanning example for PassthroughTrajectoryController.

Behavior:
 - Read desired joint targets from terminal (6 floats, space/comma separated).
 - Build a 5s trajectory from current joint positions to target, sampled at controller update_rate (default 125Hz).
 - Send the trajectory as a FollowJointTrajectory goal to the configured controller action (default: /passthrough_trajectory_controller/follow_joint_trajectory).
 - If a new target is entered while a goal is running, cancel the current goal and immediately plan+send the new trajectory.

Notes:
 - This example uses cancel+re-send as the simplest form of replanning with the Passthrough controller
   (the controller rejects new goals while executing, so cancelling is required to replace a running goal).
 - Make sure the controller is running (passthrough or scaled) and action server exists before running.
"""

import threading
import queue
import time
import math
from typing import List

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
from builtin_interfaces.msg import Duration
from controller_manager_msgs.srv import ListControllers


def parse_target_line(line: str) -> List[float]:
	# Accept space- or comma-separated floats
	s = line.strip()
	if not s:
		return []
	for sep in [',', '\t']:
		s = s.replace(sep, ' ')
	parts = s.split()
	vals = []
	for p in parts:
		try:
			vals.append(float(p))
		except ValueError:
			return []
	return vals


class PassthroughStreamer(Node):
	def __init__(self, action_name: str = '/passthrough_trajectory_controller/follow_joint_trajectory',
				 duration: float = 5.0, sample_rate: int = 125):
		super().__init__('passthrough_streamer')
		self._action_name = action_name
		self._action_client = None  # created after selecting active controller
		self._joint_state_sub = self.create_subscription(JointState, '/joint_states', self._joint_state_cb, 10)

		self._last_joint_state = None
		self._joint_names = None
		self._goal_handle = None
		self._goal_lock = threading.Lock()

		self.duration = duration
		self.sample_rate = sample_rate

	def _joint_state_cb(self, msg: JointState):
		# cache latest state and joint names
		self._last_joint_state = msg
		if self._joint_names is None and msg.name:
			self._joint_names = list(msg.name)

	def wait_for_server(self, timeout_sec: float = 5.0) -> bool:
		return self._action_client.wait_for_server(timeout_sec=timeout_sec)

	def wait_for_first_state(self, timeout_sec: float = 5.0) -> bool:
		start = time.time()
		while rclpy.ok() and (time.time() - start) < timeout_sec:
			rclpy.spin_once(self, timeout_sec=0.1)
			if self._last_joint_state is not None:
				return True
		return False

	def build_trajectory(self, target: List[float]) -> JointTrajectory:
		if self._last_joint_state is None:
			raise RuntimeError('No joint state available to build trajectory from')
		current = list(self._last_joint_state.position)
		if len(current) != len(target):
			raise RuntimeError('Target length mismatch with current joint state')

		traj = JointTrajectory()
		if self._joint_names:
			traj.joint_names = list(self._joint_names)
		else:
			# fallback UR default names
			traj.joint_names = [
				'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
				'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'
			]

		total_points = max(2, int(self.duration * self.sample_rate))
		for i in range(total_points):
			t = float(i) / (self.sample_rate)
			alpha = t / self.duration if self.duration > 0 else 1.0
			alpha = max(0.0, min(1.0, alpha))
			pos = [current[j] + alpha * (target[j] - current[j]) for j in range(len(target))]
			pt = JointTrajectoryPoint()
			pt.positions = pos
			pt.velocities = [0.0] * len(pos)
			sec = int(math.floor(t))
			nsec = int((t - sec) * 1e9)
			pt.time_from_start = Duration(sec=sec, nanosec=nsec)
			traj.points.append(pt)

		# ensure final point at duration exactly
		final = JointTrajectoryPoint()
		final.positions = target
		final.velocities = [0.0] * len(target)
		sec = int(math.floor(self.duration))
		nsec = int((self.duration - sec) * 1e9)
		final.time_from_start = Duration(sec=sec, nanosec=nsec)
		traj.points.append(final)

		return traj

	def send_trajectory_goal(self, traj: JointTrajectory) -> bool:
		# Ensure action server present
		if not self.wait_for_server(timeout_sec=5.0):
			self.get_logger().error(f'Action server {self._action_name} not available')
			return False

		goal_msg = FollowJointTrajectory.Goal()
		goal_msg.trajectory = traj

		send_goal_future = self._action_client.send_goal_async(goal_msg)
		rclpy.spin_until_future_complete(self, send_goal_future)
		goal_handle = send_goal_future.result()
		if not goal_handle.accepted:
			self.get_logger().error('Goal rejected by action server')
			return False

		with self._goal_lock:
			self._goal_handle = goal_handle

		self.get_logger().info('Goal accepted and executing')
		# Do not wait for result here - allow replanning/cancel from main loop
		return True

	def cancel_active_goal(self) -> bool:
		with self._goal_lock:
			gh = self._goal_handle
		if gh is None:
			return False
		cancel_future = gh.cancel_goal_async()
		rclpy.spin_until_future_complete(self, cancel_future, timeout_sec=5.0)
		cancel_result = cancel_future.result()
		# After cancel, clear local handle
		with self._goal_lock:
			self._goal_handle = None
		self.get_logger().info('Cancel requested (accepted by server: %s)' % str(bool(cancel_result.accepted)))
		return True

	def _create_action_client(self):
		if self._action_client is None:
			self._action_client = ActionClient(self, FollowJointTrajectory, self._action_name)

	def list_controllers(self, timeout_sec: float = 2.0):
		client = self.create_client(ListControllers, '/controller_manager/list_controllers')
		if not client.wait_for_service(timeout_sec=timeout_sec):
			self.get_logger().warn('controller_manager/list_controllers service not available')
			return None
		req = ListControllers.Request()
		future = client.call_async(req)
		rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_sec)
		if future.result() is None:
			return None
		return future.result().controller

	def select_best_controller(self, timeout_sec: float = 5.0) -> bool:
		"""Choose an active controller: prefer passthrough, fallback to scaled joint trajectory."""
		start = time.time()
		while time.time() - start < timeout_sec and rclpy.ok():
			ctrls = self.list_controllers(timeout_sec=1.0)
			if ctrls is None:
				time.sleep(0.2)
				continue
			names = {c.name: c.state for c in ctrls}
			# prefer passthrough
			if 'passthrough_trajectory_controller' in names and names['passthrough_trajectory_controller'] == 'active':
				self._action_name = '/passthrough_trajectory_controller/follow_joint_trajectory'
				self.get_logger().info('Using passthrough_trajectory_controller (active)')
				self._create_action_client()
				return True
			if 'scaled_joint_trajectory_controller' in names and names['scaled_joint_trajectory_controller'] == 'active':
				self._action_name = '/scaled_joint_trajectory_controller/follow_joint_trajectory'
				self.get_logger().info('Using scaled_joint_trajectory_controller (active)')
				self._create_action_client()
				return True
			# wait and retry
			time.sleep(0.2)
		self.get_logger().error('No suitable active controller found (passthrough or scaled)')
		return False

	def wait_for_controller_active(self, controller_name: str, timeout_sec: float = 5.0) -> bool:
		start = time.time()
		while time.time() - start < timeout_sec and rclpy.ok():
			ctrls = self.list_controllers(timeout_sec=1.0)
			if ctrls is None:
				time.sleep(0.1)
				continue
			for c in ctrls:
				if c.name == controller_name and c.state == 'active':
					return True
			time.sleep(0.1)
		return False


def stdin_reader(q: queue.Queue):
	"""Read lines from stdin and put parsed joint targets into queue."""
	print('Input target joint values as 6 floats separated by space or comma, or "exit" to quit')
	while True:
		try:
			line = input('> ')
		except EOFError:
			break
		if not line:
			continue
		if line.strip().lower() in ['exit', 'quit', 'q']:
			q.put('exit')
			break
		vals = parse_target_line(line)
		if not vals:
			print('Could not parse input. Example: 0.0 -1.0 1.2 -1.0 -1.57 0.0')
			continue
		q.put(vals)


def main():
	rclpy.init()
	node = PassthroughStreamer()

	# queue for inputs
	q = queue.Queue()
	t = threading.Thread(target=stdin_reader, args=(q,), daemon=True)
	t.start()

	try:
		# wait for first joint state so we can build trajectories
		if not node.wait_for_first_state(timeout_sec=5.0):
			node.get_logger().warn('No joint_states received within timeout; using default joint names and zeros')

		# select an active controller (prefer passthrough, fallback to scaled). Exit if none found.
		if not node.select_best_controller(timeout_sec=10.0):
			node.get_logger().error('No active passthrough/scaled controller found; exiting')
			return

		node.get_logger().info('Ready. Enter target joints:')

		running = True
		while rclpy.ok() and running:
			rclpy.spin_once(node, timeout_sec=0.1)
			try:
				item = q.get_nowait()
			except queue.Empty:
				continue

			if item == 'exit':
				running = False
				# cancel active goal if any
				node.cancel_active_goal()
				break

			# item is list of floats (target)
			target = item
			# if target length doesn't match measured joints, try to normalize
			if node._last_joint_state is None:
				node.get_logger().warn('No joint state available, skipping')
				continue
			current_len = len(node._last_joint_state.position)
			if len(target) != current_len:
				node.get_logger().warn(f'Target length {len(target)} != current joint dim {current_len}. Skipping')
				continue

			node.get_logger().info(f'Received new target: {target}. Cancelling active goal (if any) and replanning...')
			# cancel running goal
			node.cancel_active_goal()

			# small sleep to allow controller to process cancel
			time.sleep(0.05)

			# build full 5s trajectory and send as new goal
			try:
				traj = node.build_trajectory(target)
			except Exception as e:
				node.get_logger().error('Failed to build trajectory: %s' % str(e))
				continue

			ok = node.send_trajectory_goal(traj)
			if not ok:
				node.get_logger().error('Failed to send trajectory')
			else:
				node.get_logger().info('Trajectory sent (5s). You can input a new target to replan (it will cancel the current goal).')

		node.get_logger().info('Shutting down')
	finally:
		node.destroy_node()
		rclpy.shutdown()


if __name__ == '__main__':
	main()
