import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
   config = os.path.join(
      get_package_share_directory('rc_car'),
      'params',
      'config.yaml'
      )

   return LaunchDescription([
      Node(
         package='rc_car',
         executable='rc_car_node',
         # namespace='',
         name='rc_car',
         parameters=[config]
      )
   ])