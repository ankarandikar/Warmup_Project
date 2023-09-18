import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from neato2_interfaces.msg import Bump
from visualization_msgs.msg import Marker
import math

class WallFollower(Node):
    def __init__(self):
        super().__init__('wall_follower')
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.run_loop)
        self.timer = self.create_timer(timer_period, self.publish_marker45)
        self.timer = self.create_timer(timer_period, self.publish_marker90)
        self.timer = self.create_timer(timer_period, self.publish_marker135)
        self.vel_publisher = self.create_publisher(Twist, "cmd_vel", 10)
        self.scan : LaserScan = None
        self.subscriber = self.create_subscription(LaserScan, "scan", self.process_scan, 10)
        self.bump_subscriber = self.create_subscription(Bump, "bump", self.detect_bump, 10)
        self.bumper_active = False
        self.marker45_pub = self.create_publisher(Marker, 'marker_45', 10)
        self.marker90_pub = self.create_publisher(Marker, 'marker_90', 10)
        self.marker135_pub = self.create_publisher(Marker, 'marker_135', 10)
        
    def process_scan(self,message):
        self.scan = message

    def detect_bump(self,vel):
        self.bumper_active = (vel.left_front == 1 or \
                              vel.left_side == 1 or \
                              vel.right_front == 1 or \
                              vel.right_side == 1)

    def run_loop(self):
        vel = Twist()
        if not self.scan:
            return
        dist_45 = self.scan.ranges[45]
        dist_135 = self.scan.ranges[135]
        dist_90 = self.scan.ranges[90]
        if not self.bumper_active:
            vel.linear.x = 0.1
            self.vel_publisher.publish(vel)
            if dist_90 == 0.0:
                dist_90 = 3.0
            if dist_45 > dist_135: #turn right (towards wall)
                vel.angular.z = 0.1*dist_90
                vel.linear.x = 0.1
                self.vel_publisher.publish(vel)
            elif dist_45 < dist_135: #turn left (away from wall)
                vel.angular.z = -0.8/dist_90
                vel.linear.x = 0.1
                self.vel_publisher.publish(vel)
            else: #dist_45 == dist_135 (parallel to wall)
                if dist_90 > 0.5:
                    vel.angular.z = 0.05
                    vel.linear.x = 0.1
                    self.vel_publisher.publish(vel)
                elif dist_90 < 0.5:
                    vel.angular.z = -0.05
                    vel.linear.x = 0.1
                    self.vel_publisher.publish(vel)
                else:
                    vel.angular.z = 0.0
                    vel.linear.x = 0.1
                    self.vel_publisher.publish(vel)
        else:
            vel.linear.x = 0.0
            vel.linear.y = 0.0
            vel.angular.z = 0.0
            self.vel_publisher.publish(vel)

    def publish_marker45(self):
        if not self.scan:
            return
        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "namespace_45"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = self.scan.ranges[45]*math.sin(math.radians(45))
        marker.pose.position.y = self.scan.ranges[45]*math.cos(math.radians(45))
        marker.pose.position.z = 0.0
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        marker.color.a = 1.0; # Don't forget to set the alpha!
        marker.color.r = 1.0
        marker.color.g = 0.4
        marker.color.b = 0.7
        
        self.marker45_pub.publish(marker)

    def publish_marker90(self):
        if not self.scan:
            return
        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "namespace_90"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = 0.0
        # For some reason the distance at 90 always returns 0
        marker.pose.position.y = self.scan.ranges[91]
        marker.pose.position.z = 0.0
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        marker.color.a = 1.0; # Don't forget to set the alpha!
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 1.0
        
        self.marker90_pub.publish(marker)
    
    def publish_marker135(self):
        if not self.scan:
            return
        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "namespace_135"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = self.scan.ranges[135]*math.cos(math.radians(135))
        marker.pose.position.y = self.scan.ranges[135]*math.sin(math.radians(135))
        marker.pose.position.z = 0.0
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        marker.color.a = 1.0; # Don't forget to set the alpha!
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0

        self.marker135_pub.publish(marker)

def main(args=None):
    rclpy.init(args=args)
    node = WallFollower()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()