from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    ur_view = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("ur_description"),
                "launch",
                "view_ur.launch.xml",
            ])
        ),
        launch_arguments={"ur_type": "ur10e"}.items(),
    )

    return LaunchDescription([ur_view])
