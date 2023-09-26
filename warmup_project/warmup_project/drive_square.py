'''
Drive in a Square:
The Neato drives in a square due to time-based velocity commands.
'''

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class DriveSquare(Node):
    def __init__(self):
        super().__init__('drive_square')
        self.start_time = time.time()   # Initialize time when script starts
        
        # Call run_loop every 0.1 seconds
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.run_loop)

        # Publish velocity commands to cmd_vel topic
        self.vel_publisher = self.create_publisher(Twist, "cmd_vel", 10)

        # Set basic linear and angular velocities
        self.linear_vel = 0.2
        self.angular_vel = 0.3
    
    def vel_zero(self):
        '''
        Velocity commands to stop robot.
        '''
        vel = Twist()   # Create instance of Twist
        
        # Set all velocities to 0
        vel.linear.x = 0.0
        vel.linear.y = 0.0
        vel.angular.z = 0.0
        self.vel_publisher.publish(vel)
    
    def drive_straight(self):
        '''
        Velocity commands to make robot drive forward.
        '''
        vel = Twist()   # Create instance of Twist

        # Set only linear velocity
        vel.linear.x = self.linear_vel
        vel.angular.z = 0.0
        self.vel_publisher.publish(vel)
    
    def turn(self):
        '''
        Velocity commands to make robot turn left.
        '''
        vel = Twist()   # Create instance of Twist

        # Set only angular velocity
        vel.linear.x = 0.0
        vel.linear.y = 0.0
        vel.angular.z = self.angular_vel
        self.vel_publisher.publish(vel)
    
    def run_loop(self):
        '''
        Make robot drive in a square.
        '''
        elapsed_time = float(time.time()-self.start_time)   # Keep track of time since script started

        drive_time = 1.0/self.linear_vel    # Time for robot to drive 1 meter forward
        turn_time =  1.67/self.angular_vel  # Time for robot to turn 90 degrees
        # 1.5708 rad (90 deg) did not leave enough time for turn in practice,
        # so 1.67 is experimentally adjusting for the turn

        if elapsed_time < drive_time:   # First straight drive
            self.drive_straight()
        elif elapsed_time < drive_time + turn_time:     # First turn
            self.turn()
        elif elapsed_time < 2*drive_time + turn_time:   # Second straight drive
            self.drive_straight()
        elif elapsed_time < 2*drive_time + 2*turn_time: # Second turn
            self.turn()
        elif elapsed_time < 3*drive_time + 2*turn_time: # Third straight drive
            self.drive_straight()
        elif elapsed_time < 3*drive_time + 3*turn_time: # Third turn
            self.turn()
        elif elapsed_time < 4*drive_time + 3*turn_time: # Fourth straight drive
            self.drive_straight()
        elif elapsed_time < 4*drive_time + 4*turn_time: # Fourth turn
            self.turn()
        else:   # After time for four straight drives and four turns has passed, stop Neato and exit script
            self.vel_zero()
            raise KeyboardInterrupt
        
def main(args=None):
    rclpy.init(args=args)
    node = DriveSquare()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()