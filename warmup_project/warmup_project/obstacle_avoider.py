'''
Obstacle avoider:
Polar laser scan readings from the robot (within a set range of angles) are converted into cartesian coordinates
to find the center of mass of the readings, which is assumed to represent the nearest obstacle. The center of
mass is converted back to polar readings to get the angle of the obstacle. The Neato moves forward by default
until it encounters an obstacle, when the angular velocity is determined by the obstacle angle.
'''

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

        # Call run_loop and visualization function every 0.1 seconds
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.run_loop)
        self.timer = self.create_timer(timer_period, self.obstacle_viz)
        
        # Subscribe to LaserScan and Bump messages
        self.subscriber = self.create_subscription(LaserScan, "scan", self.process_scan, 10)
        self.scan = None    # Initialize scan variable
        self.bump_subscriber = self.create_subscription(Bump, "bump", self.detect_bump, 10)
        self.bumper_active = False  # Initialize bump sensor input as false

        # Publish to Twist and Marker messages
        self.vel_publisher = self.create_publisher(Twist, "cmd_vel", 10)
        self.marker_pub = self.create_publisher(Marker, 'obstacle_viz', 10)
        
        # Create variables for the obstacle angle and x- and y-coordinates of center of mass of laser scans
        self.obstacle_angle = 0
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
                              vel.right_front ==1 or \
                              vel.right_side == 1)

    def process_angles(self):
        '''
        Process readings from LaserScan to find the center of mass of all the nearby points and the angle
        of the closest obstacle.
        '''
        # Make arrays for scan processing
        angles = []
        distances = []
        x_values = []
        y_values = []
        
        scan : LaserScan = self.scan    # Make variable scan to store self.scan
        if not scan:    # Wait to run code until scan has been received
            return
        
        for i,n in enumerate(scan.ranges):
            # Save distance readings under 1.0m and between 0-45 and 315-360 degrees
            if (n < 1.0) and (i < 45 or i > 315):
                angles.append(i)    # Store filtered angle values
                distances.append(n) # Store filtered distance values
        if len(angles) == 0:    # If there are no values in the filtered ranges, set the obstacle angle to 0
            self.obstacle_angle == 0
        else:   # If there are values in the filtered ranges, find the center of mass
            for i,n in enumerate(angles):
                # Convert polar readings into cartesian coordinates
                x_values.append(distances[i]*math.cos(math.radians(n)))
                y_values.append(distances[i]*math.sin(math.radians(n)))
                
            # Calculate x- and y-coordinates of center of mass by averaging all x- and y-values
            self.x_COM = sum(x_values)/len(x_values)
            self.y_COM = sum(y_values)/len(y_values)
            if self.x_COM == 0: # Wait until x_COM is not 0 to be able to calculate the obstacle angle
                return
            
            # Polar conversion of COM to get obstacle angle (-45 to 45 degrees)
            self.obstacle_angle = math.degrees(math.atan(self.y_COM/self.x_COM)) 
        
    def run_loop(self):
        '''
        Set the velocity of the Neato depending on the center of mass and obstacle angle.
        '''
        vel = Twist()   # Create instance of Twist
        self.process_angles()   # Run scan processing function

        # Velocity commands
        if not self.bumper_active:  # If bump sensors are not active
            if self.x_COM == 0 and self.y_COM == 0: # If there is no obstacle in range, go forward
                vel.linear.x = 0.1
                vel.angular.z = 0.0
                self.vel_publisher.publish(vel)
            else:   # If there is an obstacle, set the angular velocity to turn in the opposite direction
                vel.linear.x = 0.1
                vel.angular.z = -10/self.obstacle_angle # Proportional control of the angular velocity
                self.vel_publisher.publish(vel)
        else:   # If bumpers are hit, stop Neato
            vel.linear.x = 0.0
            vel.linear.y = 0.0
            vel.angular.z = 0.0
            self.vel_publisher.publish(vel)
    
    def obstacle_viz(self):
        '''
        Visualize the center of mass representing the nearest obstacle's location in the Neato coordinate frame.
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
        if self.x_COM == 0 and self.y_COM == 0:
            # If there is no obstacle, create a teal marker directly in front of the Neato
            marker.pose.position.x = 0.5
            marker.pose.position.y = 0.0
            marker.color.r = 0.0
            marker.color.g = 0.5
            marker.color.b = 0.5
        else:
            # If there is an obstacle, create a pink marker at the obstacle's center of mass
            # Magnify the magnitude of the COM coordinates to make the marker more visible
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
        marker.color.a = 1.0;   # Setting the alpha
        
        self.marker_pub.publish(marker) # Publish to Marker message

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoider()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()