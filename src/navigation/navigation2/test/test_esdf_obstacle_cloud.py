#!/usr/bin/env python3
"""
Lock down that RC-ESDF is actually running.

Feed obstacle cloud with a point blocking the path; esdfPathSafe should
return false and MPC enters recovery. Reverse test: comment out the
esdf_map_.query line; test must fail (obstacle ignored, MPC green all the way).
"""
import time
import unittest

from geometry_msgs.msg import PoseStamped, Twist
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
import launch_testing
import launch_testing.actions
from nav_msgs.msg import OccupancyGrid, Odometry, Path
import rclpy
from rclpy.node import Node as RclpyNode
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Bool, Header


def transient_local_qos(depth=1):
    return QoSProfile(
        depth=depth,
        history=HistoryPolicy.KEEP_LAST,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        reliability=ReliabilityPolicy.RELIABLE,
    )


def generate_test_description():
    container = ComposableNodeContainer(
        name='test_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            ComposableNode(
                package='navigation2',
                plugin='navigation2::RmMpcController',
                name='rm_mpc_controller',
                parameters=[{
                    'use_sim_time': False,
                    'use_tf_pose': False,
                    'target_frame': 'map',
                    'robot_base_frame': 'base_link',
                    'path_topic': '/plan',
                    'odom_topic': '/Odometry',
                    'cmd_vel_topic': '/cmd_vel_nav_raw',
                    'control_fps': 30.0,
                    'expected_speed': 1.5,
                    'goal_tolerance': 0.25,
                    'esdf.enable': True,
                    'esdf.robot_length': 0.60,
                    'esdf.robot_width': 0.45,
                    'esdf.safety_margin': 0.04,
                    'esdf.check_steps': 5,
                    'esdf.obstacle_topic': '/segmentation/obstacle',
                    'local_safety.costmap_topic': '/local_costmap/costmap',
                    'local_safety.costmap_timeout': 1.0,
                    'feedback.executed_cmd_topic': '/cmd_vel',
                    'feedback.executed_cmd_timeout': 0.5,
                    'progress.min_displacement': 0.15,
                    'progress.no_progress_timeout': 2.0,
                    'progress.stuck_timeout': 2.0,
                    'recovery.enable': True,
                    'recovery.suppress_near_goal': 0.35,
                    'recovery.reverse_speed': 0.3,
                    'recovery.reverse_distance': 0.4,
                    'recovery.max_speed': 0.4,
                    'recovery.reach_tolerance': 0.12,
                    'recovery.dwell_time': 0.2,
                    'recovery.max_duration': 5.0,
                    'recovery.max_attempts': 2,
                    'recovery.veto_recovery_time': 3.0,
                }],
            ),
        ],
        output='screen',
    )
    return (
        LaunchDescription([
            container,
            launch_testing.actions.ReadyToTest(),
        ]),
        {'container': container},
    )


class TestEsdfObstacle(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = RclpyNode('test_esdf_obstacle')
        self.path_pub = self.node.create_publisher(Path, '/plan', 1)
        self.odom_pub = self.node.create_publisher(Odometry, '/Odometry', 10)
        self.costmap_pub = self.node.create_publisher(
            OccupancyGrid, '/local_costmap/costmap', transient_local_qos())
        self.obstacle_pub = self.node.create_publisher(
            PointCloud2, '/segmentation/obstacle', rclpy.qos.qos_profile_sensor_data)
        self.executed_pub = self.node.create_publisher(Twist, '/cmd_vel', 10)

        self.recovery_entered = []
        self.node.create_subscription(
            Bool, '/goal_approach_controller/enabled',
            lambda m: self.recovery_entered.append(not m.data),
            transient_local_qos())

    def tearDown(self):
        self.node.destroy_node()

    def _stamp(self):
        return self.node.get_clock().now().to_msg()

    def _publish_inputs(self, robot_x, robot_y, goal_x, goal_y, obstacle_x, obstacle_y):
        """发一拍完整输入：路径、里程计、代价地图（全0）、障碍点云（一个点）、执行速度。"""
        stamp = self._stamp()

        path = Path()
        path.header.frame_id = 'map'
        path.header.stamp = stamp
        for x, y in ((0.0, 0.0), (goal_x, goal_y)):
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
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = float(robot_x)
        odom.pose.pose.position.y = float(robot_y)
        odom.pose.pose.orientation.w = 1.0
        self.odom_pub.publish(odom)

        grid = OccupancyGrid()
        grid.header.frame_id = 'map'
        grid.header.stamp = stamp
        grid.info.resolution = 0.05
        grid.info.width = 160
        grid.info.height = 120
        grid.info.origin.position.x = -1.0
        grid.info.origin.position.y = -3.0
        grid.info.origin.orientation.w = 1.0
        grid.data = [0] * (160 * 120)
        self.costmap_pub.publish(grid)

        # 障碍点云：一个点在 base_link 系，MPC 会把它转到 map 再检查。
        # 机器人在 (robot_x, robot_y)，障碍在 (obstacle_x, obstacle_y)，
        # 相对机器人就是 (obstacle_x - robot_x, obstacle_y - robot_y)。
        header = Header(stamp=stamp, frame_id='base_link')
        points = [(obstacle_x - robot_x, obstacle_y - robot_y, 0.0)]
        cloud = point_cloud2.create_cloud_xyz32(header, points)
        self.obstacle_pub.publish(cloud)

        executed = Twist()
        executed.linear.x = 0.5
        self.executed_pub.publish(executed)

    def test_esdf_blocks_when_obstacle_ahead(self):
        """
        机器人在 (0,0)，目标 (5,0)，障碍点在 (1,0)。
        ESDF 应该判不安全，MPC 进恢复。
        """
        self.recovery_entered.clear()
        deadline = time.time() + 3.0
        while time.time() < deadline:
            self._publish_inputs(
                robot_x=0.0, robot_y=0.0,
                goal_x=5.0, goal_y=0.0,
                obstacle_x=1.0, obstacle_y=0.0)
            rclpy.spin_once(self.node, timeout_sec=0.03)

        # recovery_entered 收到 approach_enabled=False 说明进了恢复。
        self.assertTrue(
            any(self.recovery_entered),
            "ESDF 没有阻止 MPC：障碍在预测路径上但 esdfPathSafe 返回 true")

    def test_esdf_allows_when_no_obstacle(self):
        """
        机器人在 (0,0)，目标 (5,0)，障碍点在 (0,3) 旁边。
        ESDF 应该判安全，不进恢复。
        """
        self.recovery_entered.clear()
        deadline = time.time() + 2.0
        while time.time() < deadline:
            self._publish_inputs(
                robot_x=0.0, robot_y=0.0,
                goal_x=5.0, goal_y=0.0,
                obstacle_x=0.0, obstacle_y=3.0)
            rclpy.spin_once(self.node, timeout_sec=0.03)

        # 障碍在一边，不应该触发恢复。
        self.assertFalse(
            any(self.recovery_entered),
            "ESDF 错误阻止了安全路径")


@launch_testing.post_shutdown_test()
class TestProcessOutput(unittest.TestCase):
    def test_exit_code(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
