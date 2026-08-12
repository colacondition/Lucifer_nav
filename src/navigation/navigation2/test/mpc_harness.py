# Shared fixtures for ROS-level integration tests.
#
# Why this exists: unit tests only cover pure functions (isHazardous,
# findSafePoint, RouteTracker, ProgressMonitor). FSM transitions, the
# goal_approach_controller on/off sequence, and whether executed_speed_
# actually drives ProgressMonitor are all inside the node -- gtest cannot
# reach them. The creep-at-goal bug was exactly this class: all unit tests
# passed, it only showed on the real robot.
import time

from decision_interfaces.msg import GimbalPosture, GimbalPostureState
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry, Path
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool

# Grid covers x in [-1, 7], y in [-3, 3].  Fits path (0,0)->(5,0) plus
# the full MPC prediction horizon at max speed.
GRID_RES = 0.05
GRID_W = 160
GRID_H = 120
GRID_ORIGIN_X = -1.0
GRID_ORIGIN_Y = -3.0
_FREE_CELLS = [0] * (GRID_W * GRID_H)


def transient_local_qos(depth=1):
    """Match the node's transient_local + reliable topics."""
    return QoSProfile(
        depth=depth,
        history=HistoryPolicy.KEEP_LAST,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        reliability=ReliabilityPolicy.RELIABLE,
    )


class Harness(Node):
    """Drive the MPC node as a black box: publish every input, collect every output."""

    def __init__(self):
        super().__init__('mpc_test_harness')

        # Mutable cost cells; call set_cost_region() before driving.
        self.cells = list(_FREE_CELLS)

        self.path_pub = self.create_publisher(Path, '/plan', 1)
        self.odom_pub = self.create_publisher(Odometry, '/Odometry', 10)
        self.costmap_pub = self.create_publisher(
            OccupancyGrid, '/local_costmap/costmap', transient_local_qos())
        # Chain-end feedback.  The node uses this (not its own published
        # speed) to drive ProgressMonitor -- see feedback.executed_cmd_topic.
        self.executed_cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # 云台收放。默认两条都不发 —— 没有请求时门是开的，其他测试不受影响。
        # 发了请求就必须也发实测姿态，否则节点按「还没收到过回传」停车（这是刻意的保守
        # 默认值，见 control() 里的判据）。
        self.gimbal_posture_pub = self.create_publisher(
            GimbalPosture, '/gimbal_posture', transient_local_qos())
        self.gimbal_posture_state_pub = self.create_publisher(
            GimbalPostureState, '/gimbal_posture_state', transient_local_qos())

        self.approach_enabled = []
        self.create_subscription(
            Bool, '/goal_approach_controller/enabled',
            lambda m: self.approach_enabled.append(m.data), transient_local_qos())

        self.nav_cmds = []
        self.create_subscription(
            Twist, '/cmd_vel_nav_raw', lambda m: self.nav_cmds.append(m), 1)

    def _stamp(self):
        return self.get_clock().now().to_msg()

    def set_cost_region(self, x_lo, x_hi, y_lo, y_hi, value):
        """Set cells whose centre falls inside the world rect to value."""
        for my in range(GRID_H):
            wy = GRID_ORIGIN_Y + (my + 0.5) * GRID_RES
            if not (y_lo <= wy <= y_hi):
                continue
            for mx in range(GRID_W):
                wx = GRID_ORIGIN_X + (mx + 0.5) * GRID_RES
                if x_lo <= wx <= x_hi:
                    self.cells[my * GRID_W + mx] = value

    def publish_inputs(self, robot_xy, goal_xy, executed_speed):
        """
        Publish one full round of inputs.

        Path geometry is identical each round -- this matches the real
        planner's behaviour (publishLastPath resends unchanged paths), so
        the node should recognise it as "path not replaced".
        """
        stamp = self._stamp()

        path = Path()
        path.header.frame_id = 'map'
        path.header.stamp = stamp
        for x, y in ((0.0, 0.0), goal_xy):
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)
        self.path_pub.publish(path)

        odom = Odometry()
        odom.header.frame_id = 'map'
        odom.header.stamp = stamp
        odom.child_frame_id = 'base_link_fake'
        odom.pose.pose.position.x = float(robot_xy[0])
        odom.pose.pose.position.y = float(robot_xy[1])
        odom.pose.pose.orientation.w = 1.0
        self.odom_pub.publish(odom)

        grid = OccupancyGrid()
        grid.header.frame_id = 'map'
        grid.header.stamp = stamp
        grid.info.resolution = GRID_RES
        grid.info.width = GRID_W
        grid.info.height = GRID_H
        grid.info.origin.position.x = GRID_ORIGIN_X
        grid.info.origin.position.y = GRID_ORIGIN_Y
        grid.info.origin.orientation.w = 1.0
        grid.data = list(self.cells)        # copy so later mutations don't alias
        self.costmap_pub.publish(grid)

        executed = Twist()
        executed.linear.x = float(executed_speed)
        self.executed_cmd_pub.publish(executed)

    def set_gimbal(self, requested_lower, measured_lowered=None):
        """
        Publish one gimbal request plus its measured posture.

        measured_lowered=None 表示只发请求、不发回传 —— 用来测「还没收到过回传」的分支。
        两条话题都是 transient_local，发一次就 latch 住，不需要在驱动循环里重复发。
        """
        request = GimbalPosture()
        request.lower = bool(requested_lower)
        self.gimbal_posture_pub.publish(request)

        if measured_lowered is not None:
            state = GimbalPostureState()
            state.lowered = bool(measured_lowered)
            self.gimbal_posture_state_pub.publish(state)

    def drive(self, duration_s, robot_xy, goal_xy, executed_speed, rate_hz=20.0):
        """
        Drive at a fixed rate for duration_s.

        A constant robot_xy simulates a vehicle that is not moving.
        """
        deadline = time.time() + duration_s
        period = 1.0 / rate_hz
        while time.time() < deadline:
            self.publish_inputs(robot_xy, goal_xy, executed_speed)
            rclpy.spin_once(self, timeout_sec=period)

    def closed_loop_drive(self, duration_s, start_xy, goal_xy, rate_hz=30.0):
        """
        Drive with the node's own commands integrated back into position.

        Needed for scenarios where the FSM must observe the robot reaching a
        target (e.g. HAZARD_RECOVERY driving to a safe point, dwelling, then
        exiting back to FOLLOW). Returns the final (x, y) position.
        """
        x, y = float(start_xy[0]), float(start_xy[1])
        period = 1.0 / rate_hz
        deadline = time.time() + duration_s
        while time.time() < deadline:
            # Integrate the most-recently-received nav command (body == world
            # because yaw=0 in all tests).
            cmd = self.nav_cmds[-1] if self.nav_cmds else None
            vx = cmd.linear.x if cmd else 0.0
            vy = cmd.linear.y if cmd else 0.0
            x += vx * period
            y += vy * period
            speed = (vx * vx + vy * vy) ** 0.5
            self.publish_inputs((x, y), goal_xy, speed)
            rclpy.spin_once(self, timeout_sec=period)
        return (x, y)


def mpc_test_params(**overrides):
    """
    Return MPC node params suitable for fast integration tests.

    Timeouts are shortened; ESDF is off (requires /scan); TF is off (read
    pose directly from odometry).
    """
    params = {
        'use_sim_time': False,
        'use_tf_pose': False,
        'target_frame': 'map',
        'path_topic': '/plan',
        'odom_topic': '/Odometry',
        'cmd_vel_topic': '/cmd_vel_nav_raw',
        'control_fps': 30.0,
        'expected_speed': 1.5,
        'goal_tolerance': 0.25,
        'esdf.enable': False,
        'local_safety.costmap_topic': '/local_costmap/costmap',
        'local_safety.costmap_timeout': 1.0,
        'feedback.executed_cmd_topic': '/cmd_vel',
        'feedback.executed_cmd_timeout': 0.5,
        'feedback.override_detect_time': 0.5,
        'progress.min_displacement': 0.15,
        'progress.no_progress_timeout': 10.0,
        'progress.stuck_timeout': 10.0,
        'recovery.enable': True,
        'recovery.suppress_near_goal': 0.35,
        'recovery.reverse_speed': 0.3,
        'recovery.reverse_distance': 0.4,
        'recovery.max_speed': 0.4,
        'recovery.kp': 1.0,
        'recovery.reach_tolerance': 0.12,
        'recovery.dwell_time': 0.4,
        'recovery.max_duration': 10.0,
        'recovery.max_attempts': 3,
        'recovery.veto_recovery_time': 5.0,
    }
    params.update(overrides)
    return params
