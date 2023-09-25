import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from neato2_interfaces.msg import Bump
from visualization_msgs.msg import Marker
import math
import time

class ObstacleAvoider(Node):
    def __init__(self):
        super().__init__('obstacle_avoider')
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.run_loop)
        self.timer = self.create_timer(timer_period, self.obstacle_viz)
        self.vel_publisher = self.create_publisher(Twist, "cmd_vel", 10)
        self.scan = None
        self.subscriber = self.create_subscription(LaserScan, "scan", self.process_scan, 10)
        self.bump_subscriber = self.create_subscription(Bump, "bump", self.detect_bump, 10)
        self.marker_pub = self.create_publisher(Marker, 'obstacle_viz', 10)
        self.bumper_active = False
        self.obstacle_angle = 0
        self.x_COM = 0
        self.y_COM = 0

    def process_scan(self,message):
        self.scan = message

    def detect_bump(self,vel):
        self.bumper_active = (vel.left_front == 1 or \
                              vel.left_side == 1 or \
                              vel.right_front ==1 or \
                              vel.right_side == 1)
    
    # def process_angles(self):
    #     angles = []
    #     distances = []
    #     x_values = []
    #     y_values = []
    #     scan : LaserScan = self.scan
    #     if not scan:
    #         return
    #     for i,n in enumerate(scan.ranges):
    #         if (n < 5) and (i < 90 or i > 270):
    #             angles.append(i)
    #             distances.append(i)
    #     if len(angles) < 5:
    #         return
    #     for i,n in enumerate(angles):
    #         x_values.append(distances[i]*math.sin(math.radians(n)))
    #         y_values.append(distances[i]*math.cos(math.radians(n)))
    #     self.x_COM = sum(x_values)/len(x_values)
    #     y_COM = sum(y_values)/len(y_values)
    #     if self.x_COM == 0:
    #         return
    #     self.obstacle_angle = math.degrees(math.atan(y_COM/self.x_COM))
    #     print(self.obstacle_angle)

    def process_angles(self):
        angles = []
        distances = []
        x_values = []
        y_values = []
        scan : LaserScan = self.scan
        if not scan:
            return
        for i,n in enumerate(scan.ranges):
            if (n < 1.0) and (i < 45 or i > 315):
                angles.append(i)
                distances.append(n)
        if len(angles) == 0:
            self.obstacle_angle == 0
        else:
            for i,n in enumerate(angles):
                x_values.append(distances[i]*math.cos(math.radians(n)))
                y_values.append(distances[i]*math.sin(math.radians(n)))
            self.x_COM = sum(x_values)/len(x_values)
            self.y_COM = sum(y_values)/len(y_values)
            if self.x_COM == 0:
                return
            self.obstacle_angle = math.degrees(math.atan(self.y_COM/self.x_COM))
        
    def run_loop(self):
        vel = Twist()
        self.process_angles()
        if not self.bumper_active:
            if self.x_COM == 0 and self.y_COM == 0:
                vel.linear.x = 0.1
                vel.angular.z = 0.0
                self.vel_publisher.publish(vel)
            else:
                vel.linear.x = 0.1
                vel.angular.z = -10/self.obstacle_angle
                self.vel_publisher.publish(vel)
        else:
            vel.linear.x = 0.0
            vel.linear.y = 0.0
            vel.angular.z = 0.0
            self.vel_publisher.publish(vel)
    
    def obstacle_viz(self):
        if not self.scan:
            return
        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "my_namespace"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        if self.x_COM == 0 and self.y_COM == 0:
            marker.pose.position.x = 0.5
            marker.pose.position.y = 0.0
            marker.color.r = 0.0
            marker.color.g = 0.5
            marker.color.b = 0.5
        else:
            marker.pose.position.x = 10*self.x_COM
            marker.pose.position.y = 10*self.y_COM
            marker.color.r = 1.0
            marker.color.g = 0.4
            marker.color.b = 0.7
        marker.pose.position.z = 0.0
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        marker.color.a = 1.0; # Don't forget to set the alpha!
        
        self.marker_pub.publish(marker)

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoider()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()