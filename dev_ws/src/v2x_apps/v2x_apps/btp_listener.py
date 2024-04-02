import rclpy

from rclpy.node import Node
from vanetza_msgs.msg import BtpDataIndication


class BtpListener(Node):
    """
    BtpListener prints the BTP port number of any received BTP packet
    """

    def __init__(self):
        super().__init__("btp_listener")
        self.get_logger().info(f'Node "{self.get_name()}" started')
        self.subscription = self.create_subscription(
            BtpDataIndication, "/vanetza/btp_indication", self.listener_callback, 10
        )

    def listener_callback(self, msg: BtpDataIndication) -> None:
        self.get_logger().info(
            f"received BTP message with {len(msg.data)} bytes on port {msg.destination_port}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = BtpListener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
