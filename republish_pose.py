import rclpy
from rclpy.node import node

from std_msgs.msg import String

# Subscribe to /mavros/imu/data
# Publish to /mavros/vision_pose/pose (The current position)
# Publish to /mavros/setpoint_position/local (Where you wanna go to)


# Subscribe to /vicon/ROB498_Drone/ROB498_Drone
