'''
Finite-State Controller:
The default state of the robot is to follow the wall using the same methodology as the wall_follower
script. When laser scan readings are detected in front of the robot, indicating a person, the state switches
to person following, using the same methodology as the person_follower script.
'''

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from neato2_interfaces.msg import Bump
from visualization_msgs.msg import Marker
import math

class FiniteStateController(Node):
    def __init__(self):
        super().__init__('finite_state_controller')

        # Call run_loop every 0.1 seconds
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.run_loop)

        # Publish velocity commands to cmd_vel topic
        self.vel_publisher = self.create_publisher(Twist, "cmd_vel", 10)

        # Subscribe to LaserScan and Bump messages
        self.subscriber = self.create_subscription(LaserScan, "scan", self.process_scan, 10)
        self.scan : LaserScan = None    # Initialize scan variable
        self.bump_subscriber = self.create_subscription(Bump, "bump", self.detect_bump, 10)
        self.bumper_active = False  # Initialize bump sensor input as false

        # Create variables for the obstacle angle and x- and y-coordinates of center of mass of laser scans
        self.obstacle_angle = 0
        self.x_COM = 0
        self.y_COM = 0
    
    def process_scan(self,message):
        '''
        Assign the LaserScan message to self.scan
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

    def process_angles(self):
        '''
        Isolate laser scan readings relevant to state switching.
        '''
        if not self.scan:   # Wait to run code until scan has been received
            return
        
        # Save readings for angles between 0-20 degrees and 340-360 degrees
        # Readings within this range indicate that there is a person in front of the Neato
        self.person_range = self.scan.ranges[0:20]+self.scan.ranges[340:360]

    def wall_follower(self):
        '''
        Make robot drive parallel to wall.
        '''
        vel = Twist()   # Create instance of Twist
        if not self.scan:   # Wait to run code until scan has been received
            return
        
        # Index distances at 45, 90, and 135 degrees
        dist_45 = self.scan.ranges[45]
        dist_135 = self.scan.ranges[135]
        # Sometimes the Neatos return 0.0 at degrees near 90, so I took the averages of the distances at 85 and 95 degrees
        dist_90 = (self.scan.ranges[85]+self.scan.ranges[95])/2
        if dist_90 == 0:    # Make sure reading at 90 degrees is not 0 to prevent errors when dividing by dist_90 for velocities
            return
        
        # Velocity commands
        if not self.bumper_active:  # If bumpers are not active
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

        else:   # If bumpers are active, stop Neato
            vel.linear.x = 0.0
            vel.linear.y = 0.0
            vel.angular.z = 0.0
            self.vel_publisher.publish(vel)

    def person_following(self):
        '''
        Make robot follow the center of mass of the laser scans, representing a person.
        '''
        vel = Twist()   # Create instance of Twist
        if not self.scan:   # Wait to run code until scan has been received
            return
        
        # Make arrays for scan processing
        angles = []
        distances = []
        x_values = []
        y_values = []

        # Laser scan processing
        for i,n in enumerate(self.scan.ranges):
            # Save distance readings under 1.5m and between 0-40 and 320-360 degrees
            if (n < 1.5) and (i < 40 or i > 320):
                angles.append(i)    # Store filtered angle values
                distances.append(n) # Store filtered distance values
        if len(angles) < 5: # Wait until there are enough filtered readings
            return
        for i,n in enumerate(angles):
            # Convert polar readings into cartesian coordinates
            x_values.append(distances[i]*math.cos(math.radians(n)))
            y_values.append(distances[i]*math.sin(math.radians(n)))
        # Calculate x- and y-coordinates of center of mass by averaging all x- and y-values
        x_COM = sum(x_values)/len(x_values)
        y_COM = sum(y_values)/len(y_values)

        # Velocity commands
        # Proportional control of angular and linear velocities
        vel.angular.z = 16.0*y_COM
        vel.linear.x = 4.0*x_COM
        self.vel_publisher.publish(vel)
    
    def run_loop(self):
        '''
        Switch between states of wall and person following.
        '''
        vel = Twist()   # Create instance of Twist
        if not self.scan:   # Wait to run code until scan has been received
            return
        
        self.process_angles()   # Run scan processing function

        if not self.bumper_active:  # If bump sensors are not active
            if max(self.person_range) != 0 and max(self.person_range) < 0.75:
                # If there is a reading within the set range of angles in front of the robot
                self.person_following()
                print("Person following")
            else:   # If there are no readings in front of the robot
                self.wall_follower()
                print("Wall following")
        else:   # If bumpers are hit, stop Neato
            vel.linear.x = 0.0
            vel.angular.z = 0.0
            self.vel_publisher.publish(vel)

def main(args=None):
    rclpy.init(args=args)
    node = FiniteStateController()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()