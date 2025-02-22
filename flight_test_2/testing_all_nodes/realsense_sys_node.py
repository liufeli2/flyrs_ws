import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped


class RealSense(Node):
    def __init__(self):
        super().__init__('realsense')
        
        # Initialize storage variables
        self.position = None
        self.orientation = None
        self.timestamp = None
        self.frame_id = None

        # Initialize SENDING storage variables
        self.set_position = None
        self.set_orientation = None

        # Subscriber to RealSense pose data
        self.realsense_subscriber = self.create_subscription(PoseStamped, '/camera/pose/sample', self.realsense_callback, 1)
        self.get_logger().info('Subscribing to RealSense!')
        
        # Statement to end the inits
        self.get_logger().info('Realsense Node All Setup and Started!')


    def realsense_callback(self, msg):
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
    node = RealSense()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
