
import rclpy
import etsi_its_spatem_ts_msgs.msg as spatem_msg

from rclpy.node import Node


class SpatemProvider(Node):
    def __init__(self):
        super().__init__('spatem_provider')
        self.get_logger().info(f'Node "{self.get_name()}" started')

        # Publisher who provides SPATEM to cube-its
        self.spatem_publisher = self.create_publisher(spatem_msg.SPATEM, '/its/spatem_provided', 1)

        # Here we just provide a SPATEM to cube-its every 1 seconds.
        self.timer = self.create_timer(timer_period_sec=1.0, callback=self.publish)


    def generate_spatem(self) -> spatem_msg.SPATEM:
        """Generate a SPATEM with use-case specific data."""
        msg = spatem_msg.SPATEM()
        # MessageId, ProtocolVersion is set by SPATEM facility service
        msg.header.station_id.value = 100
        
        movement_event = spatem_msg.MovementEvent()
        movement_event.event_state.value = movement_event.event_state.PROTECTED_MOVEMENT_ALLOWED
        
        movement_state = spatem_msg.MovementState()
        movement_state.signal_group.value = 2
        movement_state.state_time_speed.array.append(movement_event)
        
        intersection_state = spatem_msg.IntersectionState()
        intersection_state.id.id.value = 1
        status_array = [0] * intersection_state.status.SIZE_BITS
        status_array[intersection_state.status.BIT_INDEX_MANUAL_CONTROL_IS_ENABLED] = 1
        intersection_state.status.value = status_array
        intersection_state.states.array.append(movement_state)
        
        msg.spat.intersections.array.append(intersection_state)

        return msg

    def publish(self) -> None:
        """Publish on topic"""
        spatem = self.generate_spatem()
        self.spatem_publisher.publish(spatem)
        self.get_logger().info('SPATEM provided')


def main(args=None):
    rclpy.init(args=args)
    spatem_provider = SpatemProvider()
    rclpy.spin(spatem_provider)
    spatem_provider.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
