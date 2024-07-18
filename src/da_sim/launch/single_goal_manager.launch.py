from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    # Node for cam_to_real
    single_goal_manager_node = Node(
        package='da_sim',
        executable='single_goal_manager',
        name='single_goal_manager',
        output='screen',
        parameters=[{'use_sim_time': True}]  # 添加仿真时间参数
    )

    ld.add_action(single_goal_manager_node)
    
    return ld
