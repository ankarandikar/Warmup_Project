'''
Person follower:
Polar laser scan readings from the robot (within a set range of angles) are converted into cartesian coordinates
to find the center of mass of the readings, which is assumed to be a person. The x-coordinate of the center of
mass determines the linear speed of the Neato (it moves faster if the center of mass is closer, and vice versa),
and the y-coordinate determines the angular speed with the goal of getting the Neato angularly aligned with the 
enter of mass. The goal is for the Neato to follow a person (represented by their center of mass) and stop once
it reaches them (through triggering of the bump sensor).
'''

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from neato2_interfaces.msg import Bump
from visualization_msgs.msg import Marker
import math

class PersonFollower(Node):
    def __init__(self):
        super().__init__('person_follower')

        # Call run_loop and visualization function every 0.1 seconds
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.run_loop)
        self.timer = self.create_timer(timer_period, self.person_viz)

        # Subscribe to LaserScan and Bump messages
        self.subscriber = self.create_subscription(LaserScan, "scan", self.process_scan, 10)
        self.scan = None    # Initialize scan variable
        self.bump_subscriber = self.create_subscription(Bump, "bump", self.detect_bump, 10)
        self.bumper_active = False  # Initialize bump sensor input as false

        # Publish to Twist and Marker messages
        self.vel_publisher = self.create_publisher(Twist, "cmd_vel", 10)
        self.marker_pub = self.create_publisher(Marker, 'person_viz', 10)

        # Create variables for x- and y-coordinates of center of mass of laser scans
        self.x_COM = 0
        self.y_COM = 0

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
        Process readings from LaserScan to find the center of mass of all the nearby points and change
        velocity accordingly.
        '''
        vel = Twist()   # Create instance of Twist
        scan : LaserScan = self.scan    # Make variable scan to store self.scan
        if not scan:    # Wait to run code until scan has been received
            return

        # Make arrays for scan processing
        angles = []
        distances = []
        x_values = []
        y_values = []

        # Laser scan processing
        for i,n in enumerate(scan.ranges):
            # Save distance readings under 1.5m and between 0-90 and 270-360 degrees
            if (n < 1.5) and (i < 90 or i > 270):
                angles.append(i)    # Store filtered angle values
                distances.append(n) # Store filtered distance values
        if len(angles) < 5: # Wait until there are enough filtered readings
            return
        for i,n in enumerate(angles):
            # Convert polar readings into cartesian coordinates
            x_values.append(distances[i]*math.cos(math.radians(n)))
            y_values.append(distances[i]*math.sin(math.radians(n)))
        # Calculate x- and y-coordinates of center of mass by averaging all x- and y-values
        self.x_COM = sum(x_values)/len(x_values)
        self.y_COM = sum(y_values)/len(y_values)

        # Velocity commands
        if not self.bumper_active:  # If bump sensors are not active
            # Proportional control of angular and linear velocities
            vel.angular.z = 16.0*self.y_COM
            vel.linear.x = 4.0*self.x_COM
            self.vel_publisher.publish(vel)
        else:   # If bumpers are hit, stop Neato
            vel.linear.x = 0.0
            vel.linear.y = 0.0
            vel.angular.z = 0.0
            self.vel_publisher.publish(vel)

    def person_viz(self):
        '''
        Visualize the center of mass representing the person's location in the Neato coordinate frame.
        '''
        if not self.scan:
            return
        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "my_namespace"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        # Magnify the magnitude of the COM coordinates to make the marker more visible
        marker.pose.position.x = 10.0*self.x_COM
        marker.pose.position.y = 10.0*self.y_COM
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
        
        self.marker_pub.publish(marker) # Publish to Marker message

def main(args=None):
    rclpy.init(args=args)
    node = PersonFollower()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()