import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition


def generate_launch_description():

    use_sim_time = LaunchConfiguration("use_sim_time", default="True")
    namespace = LaunchConfiguration("namespace", default="")
    use_rviz = LaunchConfiguration("use_rviz", default="False")
    use_namespace = LaunchConfiguration("use_namespace", default="False")

    map_dir = LaunchConfiguration(
        "map",
        default=os.path.join(
            get_package_share_directory("robot_navigation"), "maps", "warehouse_with_tiago.yaml"
        ),
    )

    param_dir = LaunchConfiguration(
        "params_file",
        default=os.path.join(
            get_package_share_directory("robot_navigation"), "params", "robot_navigation_params.yaml"
        ),
    )

    nav2_bringup_launch_dir = os.path.join(get_package_share_directory("nav2_bringup"), "launch")

    rviz_config_dir = os.path.join(get_package_share_directory("robot_navigation"), "rviz2", "robot_navigation.rviz")

    return LaunchDescription(
        [
            DeclareLaunchArgument("map", default_value=map_dir, description="Full path to map file to load"),
            DeclareLaunchArgument(
                "params_file", default_value=param_dir, description="Full path to param file to load"
            ),
            DeclareLaunchArgument(
                "use_sim_time", default_value="true", description="Use simulation clock if true"
            ),
            DeclareLaunchArgument(
                "namespace", default_value="", description="Namespace for the nodes"
            ),
            DeclareLaunchArgument(
                "use_namespace", default_value="false", description="Whether to apply the namespace to launched nodes"
            ),
            DeclareLaunchArgument(
                "use_rviz", default_value="false", description="Use Rviz for visualization"
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(nav2_bringup_launch_dir, "rviz_launch.py")),
                launch_arguments={"namespace": namespace, "use_namespace": use_namespace, "rviz_config": rviz_config_dir}.items(),
                condition=IfCondition(use_rviz),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([nav2_bringup_launch_dir, "/bringup_launch.py"]),
                launch_arguments={"map": map_dir, "use_sim_time": use_sim_time, "params_file": param_dir, "namespace": namespace, "use_namespace": use_namespace}.items(),
            ),
        ]
    )