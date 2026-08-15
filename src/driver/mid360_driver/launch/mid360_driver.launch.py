from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os, glob

def debug_prefix():
    # 仅用于 Debug/ASan 构建的排障启动；不再引用包内不存在的 asan.supp/lsan.supp。
    asan_options = {
        "new_delete_type_mismatch": "0",
        "verify_asan_link_order": "0",
        "detect_odr_violation": "0"
    }
    lsan_options = {
        "detect_leaks": "1"
    }
    asan_env = ":".join([f"{k}={v}" for k, v in asan_options.items()])
    lsan_env = ":".join([f"{k}={v}" for k, v in lsan_options.items()])
    return f"env ASAN_OPTIONS={asan_env} LSAN_OPTIONS={lsan_env}"

def generate_launch_description():
    configs = glob.glob(os.path.join(get_package_share_directory('mid360_driver'), 'config', 'params.yaml'))
    use_asan = LaunchConfiguration('use_asan')
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_asan', default_value='False',
            description='Prefix the node with ASAN/LSAN env options (Debug/ASan builds only).'),
        Node(
            package="mid360_driver",
            executable="mid360_driver_node",
            output="screen",
            emulate_tty=True,
            parameters=configs,
            prefix=PythonExpression([
                "'", debug_prefix(), "' if '", use_asan, "' == 'True' else ''"
            ])
        )
    ])
