# 集成测试：sensor_height 必须从 TF（= URDF 外参）里解析出来，不能只信 yaml。
#
# 存在的理由：雷达离地高度以前在仓库里写了三遍且互相矛盾 —— 实车外参 z 填 0.0、
# segmentation_real.yaml 填 0.59、xacro 的 default 填 0.49。分割节点只读 yaml，
# 而代价地图的高度带是相对 base_link 判定的（跟着 TF 走），两边一旦不一致，地面
# 会被整片误判成障碍，或者障碍被整片滤掉，没有任何报错。
#
# 做法：yaml 里故意填一个错得很明显的兜底值，同时用静态 TF 给出真高度，喂一帧
# 点云触发解析，然后断言节点日志里报出的是 TF 的值而不是 yaml 的值。
import math
import os
import struct
import sys
import unittest

import launch
import launch_ros
import launch_testing
import pytest
import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, PointField
from tf2_ros import StaticTransformBroadcaster

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

# TF 里给出的真实安装高度，取实车 measurement_params_real.yaml 的值。
TRUE_SENSOR_HEIGHT = 0.49
# yaml 里的兜底值，故意填成明显不同的数，方便断言到底用了哪个。
WRONG_YAML_SENSOR_HEIGHT = 0.09


@pytest.mark.launch_test
def generate_test_description():
    node = launch_ros.actions.Node(
        package='linefit_ground_segmentation_ros',
        executable='ground_segmentation_node',
        name='ground_segmentation',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'sensor_height': WRONG_YAML_SENSOR_HEIGHT,
            'sensor_height_frame': 'base_link',
            'base_to_ground_z': 0.0,
            'gravity_aligned_frame': '',
            'visualize': False,
            'input_topic': '/test/cloud_in',
            'obstacle_output_topic': '/test/obstacle',
            'ground_output_topic': '/test/ground',
        }],
    )
    return launch.LaunchDescription([
        node, launch_testing.actions.ReadyToTest(),
    ]), {'segmentation': node}


def _make_cloud(frame_id, stamp, points):
    """构造最小的 xyz float32 PointCloud2."""
    cloud = PointCloud2()
    cloud.header.frame_id = frame_id
    cloud.header.stamp = stamp
    cloud.height = 1
    cloud.width = len(points)
    cloud.fields = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
    ]
    cloud.is_bigendian = False
    cloud.point_step = 12
    cloud.row_step = cloud.point_step * cloud.width
    cloud.is_dense = True
    cloud.data = b''.join(struct.pack('<fff', *p) for p in points)
    return cloud


class Harness(Node):
    """发静态 TF + 点云，并订阅输出（节点只在有订阅者时才处理点云）."""

    def __init__(self):
        super().__init__('sensor_height_tf_harness')
        # 节点用 SensorDataQoS（best effort）收发，订阅端必须匹配，否则 QoS 不兼容、
        # 一条消息都收不到，而且节点因为「没有订阅者」直接在回调开头就 return。
        self.cloud_pub = self.create_publisher(
            PointCloud2, '/test/cloud_in', qos_profile_sensor_data)
        self.grounds = []
        self.obstacles = []
        self.create_subscription(
            PointCloud2, '/test/ground', lambda m: self.grounds.append(m),
            qos_profile_sensor_data)
        self.create_subscription(
            PointCloud2, '/test/obstacle', lambda m: self.obstacles.append(m),
            qos_profile_sensor_data)

        # 外参：雷达装在 base_link 上方 TRUE_SENSOR_HEIGHT 处。
        self.tf_static = StaticTransformBroadcaster(self)
        tf = TransformStamped()
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id = 'base_link'
        tf.child_frame_id = 'livox_frame'
        tf.transform.translation.z = TRUE_SENSOR_HEIGHT
        tf.transform.rotation.w = 1.0
        self.tf_static.sendTransform([tf])

    def publish_ground_plane(self):
        # 一圈地面点，位于雷达下方 TRUE_SENSOR_HEIGHT 处。
        points = []
        for i in range(72):
            angle = i * 5.0 * math.pi / 180.0
            for step in range(10):
                d = 0.5 + 0.5 * step
                points.append(
                    (math.cos(angle) * d, math.sin(angle) * d, -TRUE_SENSOR_HEIGHT))
        self.cloud_pub.publish(
            _make_cloud('livox_frame', self.get_clock().now().to_msg(), points))

    def pump(self, seconds=6.0):
        end = self.get_clock().now().nanoseconds + int(seconds * 1e9)
        while self.get_clock().now().nanoseconds < end:
            self.publish_ground_plane()
            rclpy.spin_once(self, timeout_sec=0.1)


class TestSensorHeightFromTf(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.harness = Harness()

    def tearDown(self):
        self.harness.destroy_node()

    def test_sensor_height_comes_from_tf_not_yaml(self, proc_output):
        self.harness.pump()

        # 节点解析成功时会打一行 INFO，带上解析出来的值。断言的是 TF 的 0.490
        # 而不是 yaml 的 0.090 —— 只要有人把解析逻辑去掉、退回只读 yaml，这条就红。
        # 不限定 stream：rclcpp 的日志默认走 stderr。
        proc_output.assertWaitFor(
            f'sensor_height resolved from TF: {TRUE_SENSOR_HEIGHT:.3f}',
            timeout=15)

        # 解析成功后会重建 segmenter，顺手确认它还在正常出点云。
        self.assertTrue(
            self.harness.grounds or self.harness.obstacles,
            '解析 sensor_height 之后节点不再输出点云，说明重建 segmenter 出了问题')


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):

    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(
            proc_info, allowable_exit_codes=[0, -2, -15])
