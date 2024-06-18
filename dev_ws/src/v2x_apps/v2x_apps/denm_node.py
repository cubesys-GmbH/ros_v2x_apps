import etsi_its_denm_msgs.msg as denm_msg
import math
import rclpy

from cube_den_msgs.srv import Transmission
from vanetza_msgs.msg import GeoNetArea, TrafficClass, PositionVector
from rclpy.node import Node


class DenmNode(Node):
    def __init__(self):
        super().__init__('denm_node')
        self.get_logger().info(f'{self.get_name()} started')

        # We are interested in our own position so we can set a reasonable destination area
        self.position_vector = None
        self.pos_vector_subscription = self.create_subscription(
            PositionVector, '/its/position_vector', self.position_update, 1
        )

        # Register callback for receiving DENMs
        self.receive_subscription = self.create_subscription(
            denm_msg.DENM, '/its/denm_received', self.receive_callback, 1
        )

        # The DEN service will take care of the fields ItsPduHeader, ActionId and Termination.
        # We simply hand over a prepared DENM to this service via den_request and it will fill
        # these fields before actually sending the final DENM as a BTP packet to the network.
        self.client = self.create_client(Transmission, '/its/den_request')

        # A more sophisticated DEN service will use more appropriate trigger criteria matching
        # the DEN use cases of your application. Here we just send a DENM every 10 seconds.
        self.create_timer(1.0, self.transmit)

    def receive_callback(self, msg: denm_msg.DENM) -> None:
        """Received DENMs are just logged in this example.
        You can implement your own DENM processing here and access any DENM field.
        """
        self.get_logger().info(
            f'Received DENM from Station Id: {msg.header.station_id.value}'
        )
        if msg.denm.situation_is_present:
            self.get_logger().info(f'  Cause code: {msg.denm.situation.event_type.cause_code}')
        if msg.denm.management.termination_is_present:
            self.get_logger().info(f'  Termination: {msg.denm.management.termination}')
        else:
            self.get_logger().info('  No termination present')

    def position_update(self, msg: PositionVector) -> None:
        """Remember last position vector"""
        self.position_vector = msg

    def request_completed(self, future: rclpy.task.Future) -> None:
        result = future.result()
        if result.confirm == Transmission.Response.CONFIRM_ACCEPTED:
            action_id = f'{result.action_id.originating_station_id.value}#{result.action_id.sequence_number.value}'
            self.get_logger().info(f'DEN request fulfilled with ActionId[{action_id}]')
        else:
            reasons = {
                Transmission.Response.CONFIRM_INVALID_REQUEST: 'invalid request',
                Transmission.Response.CONFIRM_INVALID_ACTION_ID: 'invalid ActionId',
                Transmission.Response.CONFIRM_ENCODING_FAILURE: 'ASN.1 encoding failure',
                Transmission.Response.CONFIRM_BTP_FAILURE: 'BTP failure',
            }
            self.get_logger().info(
                f"DENM could not be sent: {reasons.get(result.confirm, 'unknown reason')}"
            )

    def transmit(self) -> None:
        try:
            msg = self.generate_denm()
            future = self.send_request(msg)
            future.add_done_callback(self.request_completed)
        except Exception as e:
            self.get_logger().error(f'DENM transmission failed: {e}')

    def generate_denm(self) -> denm_msg.DENM:
        """Generate a DENM with use-case specific data."""
        msg = denm_msg.DENM()
        # Management container is mandatory.
        # A unique ActionId will be assigned by the DEN service.
        # Its detection and reference time will be set by DEN service if not set by us.
        msg.denm.management.event_position = self.get_reference_position()
        msg.denm.management.station_type.value = denm_msg.StationType.PEDESTRIAN

        # situation container is optional
        msg.denm.situation_is_present = True
        msg.denm.situation.information_quality.value = denm_msg.InformationQuality.LOWEST
        msg.denm.situation.event_type.cause_code.value = denm_msg.CauseCodeType.COLLISION_RISK

        return msg

    def get_reference_position(self) -> denm_msg.ReferencePosition:
        """Reference position is at our own position for this example."""
        pos = denm_msg.ReferencePosition()
        if self.position_vector is None:
            raise RuntimeError('No position vector available')

        pos.latitude.value = (
            int(
                self.position_vector.latitude
                * 1e6
                * denm_msg.Latitude.ONE_MICRODEGREE_NORTH
            )
            if math.isfinite(self.position_vector.latitude)
            else denm_msg.Latitude.UNAVAILABLE
        )
        pos.longitude.value = (
            int(
                self.position_vector.longitude
                * 1e6
                * denm_msg.Longitude.ONE_MICRODEGREE_EAST
            )
            if math.isfinite(self.position_vector.longitude)
            else denm_msg.Longitude.UNAVAILABLE
        )
        pos.position_confidence_ellipse.semi_major_confidence.value = (
            int(
                self.position_vector.semi_major_confidence
                * 1e2
                * denm_msg.SemiAxisLength.ONE_CENTIMETER
            )
            if math.isfinite(self.position_vector.semi_major_confidence)
            else denm_msg.SemiAxisLength.UNAVAILABLE
        )
        pos.position_confidence_ellipse.semi_minor_confidence.value = (
            int(
                self.position_vector.semi_minor_confidence
                * 1e2
                * denm_msg.SemiAxisLength.ONE_CENTIMETER
            )
            if math.isfinite(self.position_vector.semi_minor_confidence)
            else denm_msg.SemiAxisLength.UNAVAILABLE
        )
        pos.altitude.altitude_value.value = (
            int(
                self.position_vector.altitude
                * 1e2
                * denm_msg.AltitudeValue.ONE_CENTIMETER
            )
            if math.isfinite(self.position_vector.altitude)
            else denm_msg.AltitudeValue.UNAVAILABLE
        )
        pos.altitude.altitude_confidence.value = denm_msg.AltitudeConfidence.UNAVAILABLE
        return pos

    def send_request(self, msg: denm_msg.DENM) -> rclpy.Future:
        """Generate a transmission request for a DENM."""
        tx = Transmission.Request()
        # This is a new DEN event (i.e. neither an update nor termination):
        # Thus, the DEN service will generate a new ActionId for this DENM.
        tx.request = Transmission.Request.REQUEST_TRIGGER
        tx.message = msg
        if self.position_vector is not None:
            # Destination area is 500m around our own position.
            # You are free to use other shapes too.
            tx.destination_area.type = GeoNetArea.TYPE_CIRCLE
            tx.destination_area.latitude = self.position_vector.latitude
            tx.destination_area.longitude = self.position_vector.longitude
            tx.destination_area.distance_a = 500.0
        else:
            raise RuntimeError('No position vector available')
        tx.traffic_class.id = TrafficClass.TC_NORMAL_DENM

        return self.client.call_async(tx)


def main(args=None):
    rclpy.init(args=args)
    node = DenmNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
