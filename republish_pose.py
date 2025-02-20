import rclpy
import np
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String


class ImuPosePublisher(Node):
    def __init__(self):
        super().__init__('imu_pose_publisher')

        # Subscriber: Reads IMU data
        self.imu_subscriber = self.create_subscription(
            Imu,
            '/mavros/imu/data',
            self.imu_callback,
            10
        )

        # Publisher: Current position -> /mavros/vision_pose/pose
        self.vision_pose_publisher = self.create_publisher(
            PoseStamped,
            '/mavros/vision_pose/pose',
            10
        )

        # Publisher: Target position -> /mavros/setpoint_position/local
        self.setpoint_publisher = self.create_publisher(
            PoseStamped,
            '/mavros/setpoint_position/local',
            10
        )

        # Initial position (Modify as needed)
        self.current_pose = np.array([0.0, 0.0, 0.0, 0.0])  # x, y, z, theta

        # Target position (Modify as needed)
        self.end_pose = np.array([5.0, 5.0, 5.0, 0.0])  # x, y, z, theta

        # Timer to publish at 10Hz
        self.velocity = 0.2 #m/s
        self.rot_vel = 0.2 #rad/s
        self.timestep = 0.025 #s
        self.look_ahead = 2 #s
        self.timer = self.create_timer(0.1, self.publish_pose)

    def imu_callback(self, msg):
        """Handles incoming IMU data."""
        self.current_pose = 
        self.get_logger().info('Received IMU data')

    def publish_pose(self):
        """Publishes current and target positions."""
        # Create PoseStamped for current position
        current_pose = PoseStamped()
        current_pose.header.stamp = self.get_clock().now().to_msg()
        current_pose.header.frame_id = 'map'
        current_pose.pose.position.x = self.current_position[0]
        current_pose.pose.position.y = self.current_position[1]
        current_pose.pose.position.z = self.current_position[2]
        current_pose.pose.orientation.w = 1.0  # Identity orientation

        # Publish current position
        self.vision_pose_publisher.publish(current_pose)
        self.get_logger().info(f'Published Current Pose: {self.current_position}')

        target_pose = self.traj_flyout(current_pose)
        # Create PoseStamped for target position
        
        # Publish target position
        self.setpoint_publisher.publish(target_pose)
        self.get_logger().info(f'Published Target Pose: {self.target_position}')

    def traj_flyout(self):
        traj = []
        if self.look_ahead*self.velocity < np.linalg.norm(self.current_pose[0:2] - self.end_pose[0:2]):
            direction_world_frame = (self.current_pose[0:2] - self.end_pose[0:2])/np.linalg.norm(self.current_pose[0:2] - self.end_pose[0:2])
            alpha = np.acos(direction_world_frame[0])
            beta = np.acos(direction_world_frame[1])
            gamma = np.acos(direction_world_frame[2])
            
            for step in range(80):
                traj.append([self.current_pose[0] + direction_world_frame[0]*self.velocity*self.timestep,
                             self.current_pose[1] + direction_world_frame[1]*self.velocity*self.timestep,
                             self.current_pose[2] + direction_world_frame[2]*self.velocity*self.timestep, 0])
        elif np.linalg.norm(self.current_pose[0:2] - self.end_pose[0:2]) < 0.1:
            del_ang = self.current_pose[3] - self.end_pose[3]
            for step in range(80):
                if 
                    traj.append()
        else:
            traj = 

        target_pose = PoseStamped()
        target_pose.header = current_pose.header  # Same timestamp
        target_pose.pose.position.x = self.target_position[0]
        target_pose.pose.position.y = self.target_position[1]
        target_pose.pose.position.z = self.target_position[2]
        target_pose.pose.orientation.w = 1.0  # Identity orientation

        return target_pose

        
def main(args=None):
    pass

if __name__ == '__main__':
    main()


# Subscribe to /mavros/imu/data
# Publish to /mavros/vision_pose/pose (The current position)
# Publish to /mavros/setpoint_position/local (Where you wanna go to)


# Subscribe to /vicon/ROB498_Drone/ROB498_Drone
