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
        self.timer = self.create_timer(timer_period, self.publish_marker)
        self.vel_publisher = self.create_publisher(Twist, "cmd_vel", 10)
        self.scan = None
        self.subscriber = self.create_subscription(LaserScan, "scan", self.process_scan, 10)
        self.bump_subscriber = self.create_subscription(Bump, "bump", self.detect_bump, 10)
        self.bumper_active = False
        self.marker_pub = self.create_publisher(Marker, 'viz_marker', 10)
        
    def process_scan(self,message):
        self.scan = message

    def detect_bump(self,vel):
        self.bumper_active = (vel.left_front == 1 or \
                              vel.left_side == 1 or \
                              vel.right_front ==1 or \
                              vel.right_side == 1)

    def publish_marker (self):
        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "my_namespace"
        marker.id = 0

        scan : LaserScan = self.scan
        if not self.scan:
            return
        x_values = []
        y_values = []
        for i,n in enumerate(scan.ranges):  # convert to cartesian
            x_values.append(n*math.cos(math.radians(i)))
            y_values.append(n*math.sin(math.radians(i)))
        marker.type = Marker.POINTS
        marker.action = Marker.ADD
        marker.pose.position.x = x_values    # x is forward, y is right
        marker.pose.position.y = y_values
        marker.pose.position.z = 0.0
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.5
        marker.scale.y = 0.5
        marker.scale.z = 0.5
        marker.color.a = 1.0; # Don't forget to set the alpha!
        marker.color.r = 1.0
        marker.color.g = 0.4
        marker.color.b = 0.7

        self.marker_pub.publish(marker)

    def run_loop(self):
        vel = Twist()
        scan : LaserScan = self.scan
        if not self.scan:
            return
        dist_45 = scan.ranges[46]
        dist_135 = scan.ranges[136]
        dist_90 = scan.ranges[91]
        if not self.bumper_active:
            vel.linear.x = 0.1
            self.vel_publisher.publish(vel)
            if dist_90 == 0.0:
                dist_90 = 3.0
            if dist_45 > dist_135: #turn right (towards wall)
                vel.angular.z = -0.1*dist_90
                vel.linear.x = 0.1
                self.vel_publisher.publish(vel)
            elif dist_45 < dist_135: #turn left (away from wall)
                vel.angular.z = 0.6/dist_90
                vel.linear.x = 0.1
                self.vel_publisher.publish(vel)
            else: #dist_45 == dist_135 (parallel to wall)
                if dist_90 > 0.5:
                    vel.angular.z = -0.05
                    vel.linear.x = 0.1
                    self.vel_publisher.publish(vel)
                elif dist_90 < 0.5:
                    vel.angular.z = 0.05
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

def main(args=None):
    rclpy.init(args=args)
    node = WallFollower()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()