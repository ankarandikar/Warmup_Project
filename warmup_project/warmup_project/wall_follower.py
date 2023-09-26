'''
Wall follower:
Assume robot is near a wall already; robot gets distance readings at 45 and 135 degrees and script compares
them to determine whether it is angled towards or away from the wall. Script uses proportional control to
set an angular velocity for the robot. The goal is to get the robot parallel with the wall from 0.3 m away.
'''

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

        # Call run_loop and marker creation functions every 0.1 seconds
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.run_loop)
        self.timer = self.create_timer(timer_period, self.publish_marker45)
        self.timer = self.create_timer(timer_period, self.publish_marker90)
        self.timer = self.create_timer(timer_period, self.publish_marker135)

        # Subscribe to LaserScan and Bump messages
        self.subscriber = self.create_subscription(LaserScan, "scan", self.process_scan, 10)
        self.scan : LaserScan = None    # Create instance of LaserScan
        self.bump_subscriber = self.create_subscription(Bump, "bump", self.detect_bump, 10)
        self.bumper_active = False  # Initialize bump sensor input as false

        # Publish velocity commands to cmd_vel topic
        self.vel_publisher = self.create_publisher(Twist, "cmd_vel", 10)

        # Publish to Marker
        self.marker45_pub = self.create_publisher(Marker, 'marker_45', 10)
        self.marker90_pub = self.create_publisher(Marker, 'marker_90', 10)
        self.marker135_pub = self.create_publisher(Marker, 'marker_135', 10)
    
    def process_scan(self,message):
        '''
        Assign the LaserScan message to self.scan.
        '''
        self.scan = message

    def detect_bump(self,vel):
        '''
        Detect whether Neato bumpers have been hit.
        '''
        self.bumper_active = (vel.left_front == 1 or \
                              vel.left_side == 1 or \
                              vel.right_front == 1 or \
                              vel.right_side == 1)

    def run_loop(self):
        '''
        Check distance readings at 45, 90, and 135 degrees and change velocities to make Neato parallel to 
        wall at desired distance.
        '''
        vel = Twist()   # Create instance of Twist
        if not self.scan:   # Wait to run code until scan has been received
            return
        
        # Index distances at 45, 90, and 135 degrees
        dist_45 = self.scan.ranges[45]      # units: meters
        dist_135 = self.scan.ranges[135]
        # Sometimes the Neatos return 0.0 at degrees near 90, so I took the averages of the distances at 85 and 95 degrees
        dist_90 = (self.scan.ranges[85]+self.scan.ranges[95])/2

        # Wrap velocity commands in if statement so that Neato stops when bumpers are hit
        if not self.bumper_active:  # If bump sensors are not active
            vel.linear.x = 0.1  # Default to drive forward
            self.vel_publisher.publish(vel)

            if dist_45 > dist_135:  # If Neato is angled away from wall, turn left (towards wall)
                vel.angular.z = 0.1*dist_90     # Angular velocity is proportional to distance from wall (closer=faster, further=slower)
                vel.linear.x = 0.1
                self.vel_publisher.publish(vel)

            elif dist_45 < dist_135:    # If Neato is angled towards wall, turn right (away from wall)
                vel.angular.z = -0.1/dist_90    # Angular velocity is proportional to distance from wall (closer=faster, futher=slower)
                vel.linear.x = 0.1
                self.vel_publisher.publish(vel)
                
            else:   # If dist_45 == dist_135 (parallel to wall)
                # End goal to be parallel to wall, 0.3 m away
                if dist_90 > 0.3:   # If Neato is further than 0.3 m from wall
                    vel.angular.z = 0.05    # Turn towards wall
                    vel.linear.x = 0.1
                    self.vel_publisher.publish(vel)
                elif dist_90 < 0.3: # If Neato is closer than 0.3 m from wall
                    vel.angular.z = -0.05   # Turn away from wall
                    vel.linear.x = 0.1
                    self.vel_publisher.publish(vel)
                else:   # If Neat is exactly 0.3 m from wall
                    vel.angular.z = 0.0     # Don't turn
                    vel.linear.x = 0.1
                    self.vel_publisher.publish(vel)
        
        else:   # If bumpers are hit, stop Neato
            vel.linear.x = 0.0
            vel.linear.y = 0.0
            vel.angular.z = 0.0
            self.vel_publisher.publish(vel)

    def publish_marker45(self):
        '''
        Create marker at 45 degree reading in Neato's base_link coordinate frame
        '''
        if not self.scan:
            return
        marker = Marker()   # create instance of Marker
        marker.header.frame_id = "base_link"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "namespace_135"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        # Convert reading from 45 degrees to cartesian coordinates to plot in base_link frame
        marker.pose.position.x = self.scan.ranges[45]*math.cos(math.radians(45))
        marker.pose.position.y = self.scan.ranges[45]*math.sin(math.radians(45))
        marker.pose.position.z = 0.0
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        marker.color.a = 1.0;   # Setting the alpha
        marker.color.r = 1.0
        marker.color.g = 0.4
        marker.color.b = 0.7

        self.marker135_pub.publish(marker)  # Publish to Marker message

    def publish_marker90(self):
        '''
        Create marker at 90 degree reading in Neato's base_link coordinate frame
        '''
        if not self.scan:
            return
        marker = Marker()   # create instance of Marker
        marker.header.frame_id = "base_link"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "namespace_90"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        # Cartesian conversion is just x = 0 and y = value from 90 degree reading
        marker.pose.position.x = 0.0
        marker.pose.position.y = (self.scan.ranges[85]+self.scan.ranges[95])/2
        marker.pose.position.z = 0.0
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        marker.color.a = 1.0;   # Setting the alpha
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 1.0
        
        self.marker90_pub.publish(marker)   # Publish to Marker message
    
    def publish_marker135(self):
        '''
        Create marker at 135 degree reading in Neato's base_link coordinate frame
        '''
        if not self.scan:
            return
        marker = Marker()   # create instance of Marker
        marker.header.frame_id = "base_link"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "namespace_135"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        # Convert reading from 135 degrees to cartesian coordinates to plot in base_link frame
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
        marker.color.a = 1.0;   # Setting the alpha
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0

        self.marker135_pub.publish(marker)  # Publish to Marker message

def main(args=None):
    rclpy.init(args=args)
    node = WallFollower()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()