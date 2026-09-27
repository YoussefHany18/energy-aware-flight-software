from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([

        # ================= CAMERA =================
        Node(
            package='v4l2_camera',
            executable='v4l2_camera_node',
            name='camera',
            parameters=[{
                'image_width': 640,
                'image_height': 480
            }],
            output='screen'
        ),

        # ================= FLIGHT MANAGER =================
        Node(
            package='drone_system',
            executable='flight_manager',
            name='flight_manager',
            output='screen'
        ),

        # ================= ENERGY MONITOR =================
        Node(
            package='drone_system',
            executable='energy_monitor',
            name='energy_monitor',
            output='screen'
        ),

        # ================= ARUCO TRACKER =================
        Node(
            package='drone_system',
            executable='aruco_tracker',
            name='aruco_tracker',
            output='screen'
        ),

        # ================= PRECISION LAND =================
        Node(
            package='drone_system',
            executable='precision_land',
            name='precision_land',
            output='screen'
        ),

        # ================= DOCKING MANAGER =================
        Node(
            package='drone_system',
            executable='docking_manager',
            name='docking_manager',
            output='screen'
        ),

    ])
