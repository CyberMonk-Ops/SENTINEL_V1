from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
	return LaunchDescription([
	  Node(package='my_bot_logic', 
	    executable='sensor_node',
	    name='sensor_node'
	  ),

	  Node(
	    package='my_bot_logic',
	    executable='motor_node',
	    name='motor_node'
	  )
	])
