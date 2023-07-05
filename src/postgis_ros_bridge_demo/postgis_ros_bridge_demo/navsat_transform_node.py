# Copyright 2023 AIT - Austrian Institute of Technology GmbH
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import numpy as np

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Quaternion
from visualization_msgs.msg import Marker

from postgis_ros_bridge.geodesic_transform import GeodesicTransform

from scipy.spatial.transform import Rotation


class NavSatTransform(Node):
    """Navsat transform node."""

    def __init__(self):
        super().__init__(node_name='navsat_transform_node')
        self.get_logger().info(f'Starting {self.get_name()}...')

        self.geodesic_transformer = None
        self.init_geodesic_transformer()

        self._sub_gps = self.create_subscription(
            NavSatFix, 'gps/fix', self.gps_callback, 10)

        self._pub_odom = self.create_publisher(Odometry, 'odom', 10)
        self._pub_truck = self.create_publisher(Marker, 'truck', 1)

        self.last_utm_position = np.array([0, 0])
        self.yaw = 0.0

    def init_geodesic_transformer(self):
        """Initialize cartesian transformation."""
        self.declare_parameters(
            namespace="",
            parameters=[
                ('type', 'utm'),
                ('utm_zone', -1),
                ('utm_band', ''),
                ('lon', -1000.0),
                ('lat', -1000.0),
            ],
        )

        self.geodesic_transformer = None

        cartesian_type = self.get_parameter("type").value
        # if no cartesian transform type is set, disable cartesian transform
        if cartesian_type == "":
            return

        utm_zone = self.get_parameter('utm_zone').value
        utm_band = self.get_parameter('utm_band').value
        lon = self.get_parameter('lon').value
        lat = self.get_parameter('lat').value

        if cartesian_type == 'utm':
            if utm_zone > -1:
                if utm_band == '':
                    raise ValueError('utm_band is not set')

                self.geodesic_transformer = GeodesicTransform.to_utm(
                    utm_zone, utm_band, origin_transform=False, origin_lon=lon, origin_lat=lat)
            else:

                if lon == -1000.0 or lat == -1000.0:
                    raise ValueError('lon and lat must be set if no utm_zone/utm_band is set')

                self.geodesic_transformer = GeodesicTransform.to_utm_lonlat(
                    lon, lat, origin_transform=False)

        else:
            raise ValueError(
                f'Type: {cartesian_type} is not supported. Supported: utm'
            )

        self.get_logger().info(f'Initialized cartesian transform: \
                               {str(self.geodesic_transformer)}')

    def gps_callback(self, msg):
        """GPS callback."""
        msg_out = Odometry()
        msg_out.header = msg.header
        msg_out.header.frame_id = 'utm'
        msg_out.child_frame_id = 'truck'

        easting, northing = self.geodesic_transformer.transform_lonlat(msg.longitude, msg.latitude)
        msg_out.pose.pose.position.x = easting
        msg_out.pose.pose.position.y = northing

        # derive heading based on gnss position
        # TODO: looks bad when moving backward
        utm_position = np.array([easting, northing])
        dpos = utm_position - self.last_utm_position
        if np.linalg.norm(dpos) > 0.1:
            rad = np.arctan2(dpos[1], dpos[0])
            degrees = np.int(rad*180/np.pi)
            self.last_utm_position = utm_position
            self.yaw = degrees

        q = Rotation.from_euler('z', self.yaw, degrees=True).as_quat()
        msg_out.pose.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])

        self._pub_odom.publish(msg_out)

        self.publish_truck(msg_out)

    def publish_truck(self, odom_msg):

        marker = Marker(header=odom_msg.header)
        marker.type = Marker.MESH_RESOURCE
        marker.pose.position.x = odom_msg.pose.pose.position.x
        marker.pose.position.y = odom_msg.pose.pose.position.y
        marker.scale.x = 1.0
        marker.scale.y = 1.0
        marker.scale.z = 1.0
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.b = 1.0
        marker.color.g = 0.0

        q = Rotation.from_euler('z', self.yaw+90, degrees=True).as_quat()
        marker.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
        marker.mesh_resource = 'package://postgis_ros_bridge_demo/loader.dae'

        self._pub_truck.publish(marker)


def main(args=None):
    """ROS main function."""
    rclpy.init(args=args)
    postgis_publisher = NavSatTransform()
    rclpy.spin(postgis_publisher)

    postgis_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
