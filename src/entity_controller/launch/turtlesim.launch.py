from launch import LaunchDescription, LaunchContext
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument,OpaqueFunction,LogInfo
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # 声明启动参数
    declare_node_name_prefix_arg = DeclareLaunchArgument(
        'node_name_prefix', default_value='turtlenode')
    declare_nums_for_node_arg = DeclareLaunchArgument(
        'nums_for_node', default_value='2')
    declare_linear_speed_arg = DeclareLaunchArgument(
        'linear_speed', default_value='0.5')
    declare_angular_speed_arg = DeclareLaunchArgument(
        'angular_speed', default_value='0.2')
    
    def run_node_action(context: LaunchContext):
        nums_for_node =  int(context.launch_configurations['nums_for_node'])       
        node_name_prefix = context.launch_configurations['node_name_prefix']       
        nodes = []
        for i in range(nums_for_node):
            node_name = f"{node_name_prefix}_{i + 1}"
            node = Node(
                package='entity_controller',
                executable='turtle_node',
                # name=[LaunchConfiguration('node_name_prefix'),'_', f'{i + 1}'],  
                name=node_name,  
                output='screen',
                parameters=[{
                    'node_name': node_name,
                    'linear_speed': LaunchConfiguration('linear_speed'),
                    'angular_speed': LaunchConfiguration('angular_speed')
                }]
            )
            nodes.append(node)  
        return nodes 
    
    # 创建节点启动列表   
    run_node_action_func = OpaqueFunction(function=run_node_action)

    return LaunchDescription([
        declare_node_name_prefix_arg,
        declare_nums_for_node_arg,
        declare_linear_speed_arg,
        declare_angular_speed_arg,
        run_node_action_func,
        # *nodes
    ])
    
   