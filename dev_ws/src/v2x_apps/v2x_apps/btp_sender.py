import rclpy

from rclpy.node import Node
from vanetza_msgs.msg import TrafficClass
from vanetza_msgs.srv import BtpData


class BtpSender(Node):
    """
    BtpSender transmits a packet every second using Vanetza's BTP service
    """

    def __init__(self):
        super().__init__("btp_sender")
        self.get_logger().info(f'Node "{self.get_name()}" started')
        self.client = self.create_client(BtpData, "/vanetza/btp_request")
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("waiting for BTP request service...")
        self.create_timer(timer_period_sec=1.0, callback=self.trigger)

    def trigger(self):
        self.get_logger().info("BTP data request triggered")
        self.future = self.request_transmission()
        self.future.add_done_callback(self.request_completed)

    def request_transmission(self):
        request = BtpData.Request()
        request.btp_type = BtpData.Request.BTP_TYPE_NON_INTERACTIVE
        request.transport_type = BtpData.Request.TRANSPORT_TYPE_SHB
        request.destination_port = 42  # choose a unique free port number
        request.traffic_class.id = TrafficClass.TC_OTHER
        request.data = b"Your payload"
        return self.client.call_async(request)

    def request_completed(self, future):
        """
        Check BTP service response

        If our request contains invalid data, then the remote service will
        refuse to transmit our BTP packet.
        """
        response = future.result()
        text = (
            "successfully"
            if response.confirm == BtpData.Response.CONFIRM_ACCEPTED
            else f"with failure {response.confirm}"
        )
        self.get_logger().info(f"BTP request completed {text}")


def main(args=None):
    rclpy.init(args=args)
    node = BtpSender()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
