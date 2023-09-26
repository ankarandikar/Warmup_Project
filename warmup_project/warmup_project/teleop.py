'''
Robot teleoperator:
Use w, a, x, d keys to move forward, left, back, and right
Use s key to stop
'''

import tty
import select
import sys
import termios
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class Teleop(Node):
    def __init__(self):
        super().__init__('teleop')

        # Call run_loop every 0.1 seconds
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.run_loop)
        self.vel = Twist()  # Create instance of Twist

        # Publish to cmd_vel topic
        self.vel_publisher = self.create_publisher(Twist, "cmd_vel", 10)
    
    def getKey(self):
        '''
        Get keyboard inputs.
        '''
        settings = termios.tcgetattr(sys.stdin)
        key = None
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return key

    def run_loop(self):
        '''
        Check keyboard input from user and change velocity accordingly.
        '''
        key = self.getKey()     # Check which key is pressed
        print(key)  # Print keyboard input
        
        if key == 'w':  # Move forward
            self.vel.linear.x = 1.0     # units: meters/seconds
            self.vel_publisher.publish(self.vel)
        elif key == 'a':    # Turn left
            # Turn for set amount of time
            self.vel.angular.z = 1.0    # units: radians/seconds
            self.vel_publisher.publish(self.vel)
            time.sleep(1.5708)  # Enough time for 90 degree turn
            # Go straight after turning
            self.vel.angular.z = 0.0
            self.vel.linear.x = 1.0
            self.vel_publisher.publish(self.vel)
        elif key == 's':    # Stop
            self.vel.linear.x = 0.0
            self.vel_publisher.publish(self.vel)
        elif key == 'd':    # Turn right
            # Turn for set amount of time
            self.vel.angular.z = -1.0
            self.vel_publisher.publish(self.vel)
            time.sleep(1.5708)  # Enough time for 90 degree turn
            # Go straight after turning
            self.vel.angular.z = 0.0
            self.vel.linear.x = 1.0
            self.vel_publisher.publish(self.vel)
        elif key == 'x':    # Move back
            self.vel.linear.x = -1.0
            self.vel_publisher.publish(self.vel)
        elif key == '\x03':     # Exit script
            self.vel.linear.x = 0.0
            self.vel.angular.z = 0.0
            raise KeyboardInterrupt
        else:   # If any keys without a set output are pressed
            print("Invalid input")

def main(args=None):
    rclpy.init(args=args)
    node = Teleop()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()