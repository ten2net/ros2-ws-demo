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
            log1 = LogInfo(msg=["******************* ", node_name, " *****************"])
            nodes.append(node)  
        return nodes  #[log1,nodes]
    
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
    
    
    # 测试节点多实例启动
    # ros2 launch entity_controller turtlesim.launch.py node_name_prefix:=TN nums_for_node:=2 linear_speed:=0.6 angular_speed:=0.3
   
   
#   <depend>geometry_msgs</depend>
#   <depend>rcl_interfaces</depend>
#   <depend>std_msgs</depend>
#   <depend>std_srvs</depend>
# 为了控制 turtle2，你需要一个新的遥控节点. 不过如果你尝试运行之前的指令，你会发现这个遥控节点也会控制 turtle1.这是因为 turtle_teleop_key 默认会发布到 cmd_vel topic. 所以如果想控制 turtle2，你需要重映射 cmd_vel topic给 turtle2.
#ros2 run turtlesim turtle_teleop_key --ros-args --remap turtle1/cmd_vel:=turtle2/cmd_vel