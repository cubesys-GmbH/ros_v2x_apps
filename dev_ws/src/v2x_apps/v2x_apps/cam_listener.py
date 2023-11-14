import rclpy
from rclpy.node import Node

from etsi_its_msgs.msg import CAM


class CamListener(Node):

    def __init__(self):
        super().__init__('cam_listener')
        self.get_logger().info(f'Node "{self.get_name()}" started')
        self.subscription = self.create_subscription(
            CAM,
            '/cam_received',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('I heard CAM: "%s"' % msg)


def main(args=None):
    rclpy.init(args=args)

    cam_listener = CamListener()
    
    rclpy.spin(cam_listener)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    cam_listener.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
