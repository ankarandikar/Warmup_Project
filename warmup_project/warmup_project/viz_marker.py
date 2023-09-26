"""
Rviz Visualization:
Create a marker at position (x=1.0m,y=2.0m) in the Neato's base_link coordinate frame.
"""

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker

class VizMarker(Node):
    def __init__(self):
        super().__init__('viz_marker')
        # Create and publish marker every 0.1 seconds
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.publish_marker)
        self.marker_pub = self.create_publisher(Marker, 'viz_marker', 10)
    
    def publish_marker (self):
        '''
        Create marker to visualize chosen position.
        '''
        marker = Marker()   # Create instance of Marker
        marker.header.frame_id = "base_link"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "my_namespace"
        marker.id = 0

        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = 1.0    # positive x is forward
        marker.pose.position.y = 2.0    # positive y is left
        marker.pose.position.z = 0.0
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.5
        marker.scale.y = 0.5
        marker.scale.z = 0.5
        marker.color.a = 1.0;   # Setting the alpha
        marker.color.r = 1.0
        marker.color.g = 0.4
        marker.color.b = 0.7

        self.marker_pub.publish(marker)     # Publish to Marker message

def main(args=None):
    rclpy.init(args=args)
    node = VizMarker()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()