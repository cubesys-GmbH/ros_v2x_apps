# ==============================================================================
# MIT License
#
# Copyright (c) 2025 cubesys GmbH
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
# 
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.
# ==============================================================================

import rclpy
import cube_facility_msgs.msg as facility_msg

from rclpy.node import Node
from cube_facility_msgs.srv import StationaryVehicleRequest


class StationaryVehicleTrigger(Node):
    def __init__(self):
        super().__init__('StVeWaTest')

        # Information quality
        self.information_quality = facility_msg.InformationQuality()
        self.information_quality.value = facility_msg.InformationQuality.INFORMATION_QUALITY_LEVEL_ONE

        # Road type
        self.road_type = facility_msg.RoadType()
        self.road_type.value = facility_msg.RoadType.ROAD_TYPE_UNKNOWN

        # Lane position
        self.lane_position = facility_msg.LanePosition()
        self.lane_position.value = facility_msg.LanePosition.LANE_POSITION_UNKNOWN

        # Stationary Vehicle type
        self.stationary_vehicle_type = facility_msg.StationaryVehicleWarningType()
        self.stationary_vehicle_type.value = facility_msg.StationaryVehicleWarningType.STOPPED_VEHICLE

        # Ignition
        self.ignition = False 

        # Termination
        self.termination = False

        # service client
        self.client = self.create_client(
            StationaryVehicleRequest,
            '/c2c/stationary_vehicle_request')

        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn(
                f'Service {
                    self.client.service_name} not available, waiting again...')
        self.get_logger().info(f'Service {self.client.service_name} available')

    def generate_request(self):
        request = StationaryVehicleRequest.Request()
        request.information_quality.value = self.information_quality.value
        request.road_type.value = self.road_type.value
        request.lane_position.value = self.lane_position.value
        request.stationary_vehicle_type.value = self.stationary_vehicle_type.value
        request.ignition = self.ignition
        request.termination = self.termination
        return request

    def send_request(self):
        request = self.generate_request()
        self.future = self.client.call_async(request)


def main(args=None):
    rclpy.init(args=args)
    node = StationaryVehicleTrigger()
    node.send_request()

    while rclpy.ok():
        rclpy.spin_once(node)
        if node.future.done():
            try:
                response = node.future.result()
                if response.success:
                    node.get_logger().info(f'Request completed successful')
                else:
                    node.get_logger().info(
                        f'Request could not be sent: {response.message}')
            except Exception as e:
                node.get_logger().error(f'Service call failed {e}')
            break

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
