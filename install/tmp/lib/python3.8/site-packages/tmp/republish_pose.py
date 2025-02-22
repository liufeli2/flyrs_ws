import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

# vicon (processed)
# Position: x=0.3733230261373949, y=0.4261296018065068, z=0.027844524678396324
# Orientation: x=-0.012163230149627372, y=0.04449411931635693, z=0.9863827407925292, w=0.15786518883277345
# Timestamp: 1740156570.27055376
# Frame ID: ROB498_Drone/ROB498_Drone

# realsense
# header:
#   stamp:
#     sec: 1740157512
#     nanosec: 863638528
#   frame_id: odom_frame
# child_frame_id: camera_pose_frame
# pose:
#   pose:
#     position:
#       x: -0.009451478719711304
#       y: -0.03495466336607933
#       z: -0.00022792984964326024
#     orientation:
#       x: 0.005711567588150501
#       y: 0.0202382430434227
#       z: -0.09548740833997726
#       w: 0.9952085614204407
#   covariance:
#   - 0.1
#   - 0.0
#   - 0.0
#   - 0.0
#   - 0.0
#   - 0.0
#   - 0.0
#   - 0.1
#   - 0.0
#   - 0.0
#   - 0.0
#   - 0.0
#   - 0.0
#   - 0.0
#   - 0.1
#   - 0.0
#   - 0.0
#   - 0.0
#   - 0.0
#   - 0.0
#   - 0.0
#   - 0.001
#   - 0.0
#   - 0.0
#   - 0.0
#   - 0.0
#   - 0.0
#   - 0.0
#   - 0.001
#   - 0.0
#   - 0.0
#   - 0.0
#   - 0.0
#   - 0.0
#   - 0.0
#   - 0.001
# twist:
#   twist:
#     linear:
#       x: 0.007998393104054757
#       y: -0.0102199788249091
#       z: 0.007182180928679173
#     angular:
#       x: -0.010747638842260022
#       y: 0.004091490835956978
#       z: -0.0025083696083936916
#   covariance:
#   - 0.1
#   - 0.0
#   - 0.0
#   - 0.0
#   - 0.0
#   - 0.0
#   - 0.0
#   - 0.1
#   - 0.0
#   - 0.0
#   - 0.0
#   - 0.0
#   - 0.0
#   - 0.0
#   - 0.1
#   - 0.0
#   - 0.0
#   - 0.0
#   - 0.0
#   - 0.0
#   - 0.0
#   - 0.001
#   - 0.0
#   - 0.0
#   - 0.0
#   - 0.0
#   - 0.0
#   - 0.0
#   - 0.001
#   - 0.0
#   - 0.0
#   - 0.0
#   - 0.0
#   - 0.0
#   - 0.0
#   - 0.001


class Vicon(Node):
    def __init__(self):
        super().__init__('vicon')
        
        # Initialize storage variables
        self.position = None
        self.orientation = None
        self.timestamp = None
        self.frame_id = None

        # Subscriber to Vicon pose data
        self.vicon_subscriber = self.create_subscription(PoseStamped, '/vicon/ROB498_Drone/ROB498_Drone', self.vicon_callback, 1)
        # self.vicon_subscriber = self.create_subscription(PoseStamped, '/camera/pose/sample', self.vicon_callback, 1)
        self.get_logger().info('ViconPoseStorage Node Started')

    def vicon_callback(self, msg):
        self.position = msg.pose.position
        self.orientation = msg.pose.orientation
        self.timestamp = msg.header.stamp
        self.frame_id = msg.header.frame_id

        # Print values normally
        print(f"Position: x={self.position.x}, y={self.position.y}, z={self.position.z}")
        print(f"Orientation: x={self.orientation.x}, y={self.orientation.y}, z={self.orientation.z}, w={self.orientation.w}")
        print(f"Timestamp: {self.timestamp.sec}.{self.timestamp.nanosec}")
        print(f"Frame ID: {self.frame_id}")

def main(args=None):
    rclpy.init(args=args)
    node = Vicon()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
