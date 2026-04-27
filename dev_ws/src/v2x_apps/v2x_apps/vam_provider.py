import math
import rclpy
import etsi_its_vam_ts_msgs.msg as vam_msg

from typing import Optional
from rclpy.node import Node
from vanetza_msgs.msg import PositionVector


class VamValue:
    """Represent a VAM value with scaling and range checking."""

    def __init__(self, value, scale=1.0, unavailable=math.nan):
        self.value = value
        self.scaling_factor = scale
        self.min_value = -math.inf
        self.max_value = math.inf
        self.out_of_range_value = unavailable
        self.unavailable_value = unavailable

    def range(self, min, max, out_of_range=None):
        self.min_value = min
        self.max_value = max
        if out_of_range is not None:
            self.out_of_range_value = out_of_range

    def get(self) -> int:
        if math.isfinite(self.value):
            value = round(self.value * self.scaling_factor)
            if value < self.min_value or value > self.max_value:
                return int(self.out_of_range_value)
            else:
                return int(value)
        else:
            return int(self.unavailable_value)


# ETSI ITS lat/lon are encoded in 0.1-microdegree units (scale 1e7).
class VamLatitudeValue(VamValue):
    def __init__(self, value):
        super().__init__(value, 1e7, vam_msg.Latitude.UNAVAILABLE)
        self.range(vam_msg.Latitude.MIN, vam_msg.Latitude.MAX)


class VamLongitudeValue(VamValue):
    def __init__(self, value):
        super().__init__(value, 1e7, vam_msg.Longitude.UNAVAILABLE)
        self.range(vam_msg.Longitude.MIN, vam_msg.Longitude.MAX)


# Altitude and semi-axis lengths are encoded in centimetres (scale 1e2).
class VamSemiAxisLengthValue(VamValue):
    def __init__(self, value):
        super().__init__(value, 1e2, vam_msg.SemiAxisLength.UNAVAILABLE)
        self.range(
            vam_msg.SemiAxisLength.MIN,
            vam_msg.SemiAxisLength.MAX,
            vam_msg.SemiAxisLength.OUT_OF_RANGE,
        )


class VamAltitudeValue(VamValue):
    def __init__(self, value):
        super().__init__(value, 1e2, vam_msg.AltitudeValue.UNAVAILABLE)
        self.range(vam_msg.AltitudeValue.MIN, vam_msg.AltitudeValue.MAX)


class VamProvider(Node):
    def __init__(self):
        super().__init__('vam_provider')
        self.get_logger().info(f'Node "{self.get_name()}" started')

        # Our reference position
        self.position_vector: Optional[PositionVector] = None
        self.pos_vector_subscription = self.create_subscription(
            PositionVector, '/its/position_vector', self.position_update, 1)

        # Publisher who provides VAM to cube-its
        self.vam_publisher = self.create_publisher(vam_msg.VAM, '/its/vam_provided', 1)

        # Here we just provide a VAM to cube-its every 1 seconds.
        self.create_timer(timer_period_sec=1.0, callback=self.publish)

    def position_update(self, msg: PositionVector) -> None:
        """Remember last position vector."""
        self.position_vector = msg

    def get_reference_position(self) -> vam_msg.ReferencePositionWithConfidence:
        """Return our own position as reference position for this example."""
        if self.position_vector is None:
            raise RuntimeError('No position vector available')

        pv = self.position_vector
        pos = vam_msg.ReferencePositionWithConfidence()
        pos.latitude.value = VamLatitudeValue(pv.latitude).get()
        pos.longitude.value = VamLongitudeValue(pv.longitude).get()
        ellipse = pos.position_confidence_ellipse
        ellipse.semi_major_axis_length.value = (
            VamSemiAxisLengthValue(pv.semi_major_confidence).get())
        ellipse.semi_minor_axis_length.value = (
            VamSemiAxisLengthValue(pv.semi_minor_confidence).get())
        pos.altitude.altitude_value.value = VamAltitudeValue(pv.altitude).get()
        pos.altitude.altitude_confidence.value = vam_msg.AltitudeConfidence.UNAVAILABLE
        return pos

    def generate_vam(self) -> vam_msg.VAM:
        """Generate a VAM with use-case specific data."""
        msg = vam_msg.VAM()
        # Message header will be assigned by the VA service.
        # VRU is a standstill pedestrian
        basic = msg.vam.vam_parameters.basic_container
        basic.station_type.value = vam_msg.TrafficParticipantType.PEDESTRIAN
        basic.reference_position = self.get_reference_position()

        # Assign some values at the mandatory VruHighFrequencyContainer
        vru_hf = vam_msg.VruHighFrequencyContainer()
        vru_hf.speed.speed_value.value = vam_msg.SpeedValue.STANDSTILL
        vru_hf.speed.speed_confidence.value = vam_msg.SpeedConfidence.UNAVAILABLE
        vru_hf.heading.value.value = vam_msg.Wgs84AngleValue.WGS84_NORTH
        vru_hf.heading.confidence.value = vam_msg.Wgs84AngleConfidence.UNAVAILABLE
        accel = vru_hf.longitudinal_acceleration
        accel.longitudinal_acceleration_value.value = (
            vam_msg.LongitudinalAccelerationValue.UNAVAILABLE)
        accel.longitudinal_acceleration_confidence.value = (
            vam_msg.AccelerationConfidence.UNAVAILABLE)
        vru_hf.device_usage_is_present = True
        vru_hf.device_usage.value = vam_msg.VruDeviceUsage.LISTENING_TO_AUDIO
        msg.vam.vam_parameters.vru_high_frequency_container = vru_hf
        return msg

    def publish(self) -> None:
        """Publish on topic."""
        if self.position_vector:
            vam = self.generate_vam()
            self.vam_publisher.publish(vam)
            self.get_logger().info('VAM provided')


def main(args=None):
    rclpy.init(args=args)
    vam_provider = VamProvider()
    rclpy.spin(vam_provider)
    vam_provider.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
