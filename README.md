ros2 topic pub -1 /joint_command example_interfaces/msg/Float64MultiArray "{data: [0.5, 0.5, 0.5, 0.5, 0.5, 0.5]}"  用来发布一个名字叫joint_command的服务，并且用的接口是example_interfaces/msg/Float64MultiArray



创建自己接口的时候把这几段放入package.xml中，在<test_depend>和<buildtool_depend>之间
  <buildtool_depend>rosidl_default_generators</buildtool_depend>
  <exec_depend>rosidl_default_runtime</exec_depend>
  <member_of_group>rosidl_interface_packages</member_of_group>

创建自己接口的时候把这几段放入Cmake里面
find_package(rosidl_default_generators REQUIRED)   #网站直接无脑复制的
rosidl_generate_interfaces(${PROJECT_NAME}
""
 )
ament_export_dependencies(rosidl_default_runtime)

运行机器人的关节
ros2 topic pub -1 /pose_command my_robot_interfaces/msg/PoseCommand "{x: 0.0, y: -0.7, z: 0.2, roll: 2, pitch: 0.0, yaw: 0.0, cartesian_path: true}"
在使用卡笛尔路径规划时，可以使用坐标x: 0.0, y: -0.7, z: 0.2, roll: 3.14, pitch: 0.0, yaw: 0.0为原始坐标
				x: 0.0, y: -0.7, z: 0.5, roll: 3.14, pitch: 0.0, yaw: 0.0来展示卡笛尔路径
