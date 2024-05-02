import rclpy

from rclpy.node import Node
from etsi_its_cam_msgs.msg import CAM


class CamListener(Node):
    def __init__(self):
        super().__init__('cam_listener')
        self.get_logger().info(f'Node "{self.get_name()}" started')
        self.subscription = self.create_subscription(
            CAM,
            '/vanetza/cam_received',
            self.listener_callback,
            10)

    def listener_callback(self, msg: CAM) -> None:
        self.get_logger().info(f"Received CAM from Station Id: {msg.header.station_id.value}")


def main(args=None):
    rclpy.init(args=args)
    cam_listener = CamListener()
    rclpy.spin(cam_listener)
    cam_listener.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
