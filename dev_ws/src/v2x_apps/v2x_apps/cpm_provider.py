import math
import rclpy
import etsi_its_cpm_ts_msgs.msg as cpm_msg

from typing import Optional
from rclpy.node import Node
from vanetza_msgs.msg import PositionVector


class CpmValue:
    """ Represents a CPM value with scaling and range checking. """
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


class CpmLatitudeValue(CpmValue):
    def __init__(self, value):
        super().__init__(value, 1e7, cpm_msg.Latitude.UNAVAILABLE)
        self.range(cpm_msg.Latitude.MIN, cpm_msg.Latitude.MAX)


class CpmLongitudeValue(CpmValue):
    def __init__(self, value):
        super().__init__(value, 1e7, cpm_msg.Longitude.UNAVAILABLE)
        self.range(cpm_msg.Longitude.MIN, cpm_msg.Longitude.MAX)


class CpmSemiAxisLengthValue(CpmValue):
    def __init__(self, value):
        super().__init__(value, 1e2, cpm_msg.SemiAxisLength.UNAVAILABLE)
        self.range(cpm_msg.SemiAxisLength.MIN, cpm_msg.SemiAxisLength.MAX, cpm_msg.SemiAxisLength.OUT_OF_RANGE)


class CpmAltitudeValue(CpmValue):
    def __init__(self, value):
        super().__init__(value, 1e2, cpm_msg.AltitudeValue.UNAVAILABLE)
        self.range(cpm_msg.AltitudeValue.MIN, cpm_msg.AltitudeValue.MAX)


class CpmProvider(Node):
    def __init__(self):
        super().__init__('cpm_provider')
        self.get_logger().info(f'Node "{self.get_name()}" started')

        # We are interested in our own position so we can set a reasonable destination area
        self.position_vector: Optional[PositionVector] = None
        self.pos_vector_subscription = self.create_subscription(
            PositionVector, '/its/position_vector', self.position_update, 1)

        # Publisher who provides CPM to cube-its
        self.cpm_publisher = self.create_publisher(
            cpm_msg.CollectivePerceptionMessage, '/its/cpm_provided', 1)

        # Here we just provide a CPM to cube-its every 1 seconds.
        self.create_timer(timer_period_sec=1.0, callback=self.publish)

    def position_update(self, msg: PositionVector) -> None:
        """Remember last position vector"""
        self.position_vector = msg

    def get_reference_position(self) -> cpm_msg.ReferencePosition:
        """Reference position is at our own position for this example."""
        if self.position_vector is None:
            raise RuntimeError('No position vector available')

        # Reference position
        pos = cpm_msg.ReferencePosition()
        pos.latitude.value = CpmLatitudeValue(self.position_vector.latitude).get()
        pos.longitude.value = CpmLongitudeValue(self.position_vector.longitude).get()
        pos.position_confidence_ellipse.semi_major_confidence.value = CpmSemiAxisLengthValue(self.position_vector.semi_major_confidence).get()
        pos.position_confidence_ellipse.semi_minor_confidence.value = CpmSemiAxisLengthValue(self.position_vector.semi_minor_confidence).get()
        pos.altitude.altitude_value.value = CpmAltitudeValue(self.position_vector.altitude).get()
        pos.altitude.altitude_confidence.value = cpm_msg.AltitudeConfidence.UNAVAILABLE
        return pos

    def generate_perceived_object_cpm(self) -> cpm_msg.CollectivePerceptionMessage:
        """Generate a CPM with use-case specific data."""
        
        # Perceived object
        perceived_object = cpm_msg.PerceivedObject()
        perceived_object.measurement_delta_time.value = 1
        perceived_object.position.x_coordinate.value.value = 800
        perceived_object.position.x_coordinate.confidence.value = 1
        perceived_object.position.y_coordinate.value.value = -500
        perceived_object.position.y_coordinate.confidence.value = 1
        perceived_object.angles.z_angle.value.value = 900
        perceived_object.angles.z_angle.confidence.value = 1
        perceived_object.object_dimension_z_is_present = True
        perceived_object.object_dimension_z.value.value = 10
        perceived_object.object_dimension_z.confidence.value = 1
        perceived_object.object_dimension_y_is_present = True
        perceived_object.object_dimension_y.value.value = 20
        perceived_object.object_dimension_y.confidence.value = 1
        perceived_object.object_dimension_x_is_present = True
        perceived_object.object_dimension_x.value.value = 30
        perceived_object.object_dimension_x.confidence.value = 1

        # Container
        container = cpm_msg.WrappedCpmContainer()
        container.container_id.value = 5
        container.container_data.choice.value = 5
        container.container_data.perceived_object_container.number_of_perceived_objects.value = 1
        container.container_data.perceived_object_container.perceived_objects.array = [
            perceived_object]

        cpm = cpm_msg.CollectivePerceptionMessage()
        cpm.payload.management_container.reference_position = self.get_reference_position()
        cpm.payload.cpm_containers.value.array = [container]
        return cpm

    def publish(self) -> None:
        """Publish on topic"""
        if self.position_vector:
            cpm = self.generate_perceived_object_cpm()
            self.cpm_publisher.publish(cpm)
            self.get_logger().info('CPM provided')


def main(args=None):
    rclpy.init(args=args)
    cpm_provider = CpmProvider()
    rclpy.spin(cpm_provider)
    cpm_provider.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
