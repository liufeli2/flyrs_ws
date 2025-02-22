import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty, Trigger

# import the RealSense or Vicon nodes
import realsense_sys_node
import vicon_sys_node

# Setup the node
running_node = RealSense()
# running_node = Vicon()

def initial_position():
    running_node.init_position = running_node.position
    running_node.init_orientation = running_node.orientation

# Callback handlers
def handle_launch():
    print('Launch Requested. Your drone should take off.')
    # Set the position to current pose but 1.5 m higher
    running_node.set_position = running_node.position
    running_node.set_orientation = running_node.orientation
    running_node.set_position.z = running_node.init_position.z + 1.5

def handle_test():
    print('Test Requested. Your drone should perform the required tasks. Recording starts now.')
    running_node.set_position = running_node.position
    running_node.set_orientation = running_node.orientation

def handle_land():
    print('Land Requested. Your drone should land.')
    # Set the position where it started
    running_node.set_position = running_node.init_position
    running_node.set_orientation = running_node.init_orientation

def handle_abort():
    print('Abort Requested. Your drone should land immediately due to safety considerations')
    # Set the position where it started
    running_node.set_position = running_node.init_position
    running_node.set_orientation = running_node.init_orientation

# Service callbacks
def callback_launch(request, response):
    handle_launch()
    return response

def callback_test(request, response):
    handle_test()
    return response

def callback_land(request, response):
    handle_land()
    return response

def callback_abort(request, response):
    handle_abort()
    return response

class CommNode(Node):
    def __init__(self):
        super().__init__('rob498_drone_XX')
        self.srv_launch = self.create_service(Trigger, 'rob498_drone_XX/comm/launch', callback_launch)
        self.srv_test = self.create_service(Trigger, 'rob498_drone_XX/comm/test', callback_test)
        self.srv_land = self.create_service(Trigger, 'rob498_drone_XX/comm/land', callback_land)
        self.srv_abort = self.create_service(Trigger, 'rob498_drone_XX/comm/abort', callback_abort)


def main(args=None):
    rclpy.init(args=args)
    comm_node = CommNode()
    rclpy.spin(running_node)
    initial_position()
    rclpy.spin(comm_node)
    running_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


# def main(args=None):
#     rclpy.init(args=args)
#     node = CommNode()
#     rclpy.spin(node)
#     rclpy.shutdown()

# if __name__ == "__main__":
#     main()