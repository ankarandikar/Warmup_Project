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
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.run_loop)
        self.vel = Twist()
        self.vel_publisher = self.create_publisher(Twist, "cmd_vel", 10)
    
    def getKey(self):
        settings = termios.tcgetattr(sys.stdin)
        key = None
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return key

    def run_loop(self):
        key = None
        key = self.getKey()
        print(key)
        if key == 'w':
            self.vel.linear.x = 1.0
            self.vel_publisher.publish(self.vel)
        elif key == 'a':
            self.vel.angular.z = 1.0
            self.vel_publisher.publish(self.vel)
            time.sleep(1.5708)
            self.vel.angular.z = 0.0
            self.vel.linear.x = 1.0
            self.vel_publisher.publish(self.vel)
        elif key == 's':
            self.vel.linear.x = 0.0
            self.vel_publisher.publish(self.vel)
        elif key == 'd':
            self.vel.angular.z = -1.0
            self.vel_publisher.publish(self.vel)
            time.sleep(1.5708)
            self.vel.angular.z = 0.0
            self.vel.linear.x = 1.0
            self.vel_publisher.publish(self.vel)
        elif key == 'x':
            self.vel.linear.x = -1.0
            self.vel_publisher.publish(self.vel)
        elif key == '\x03':
            self.vel.linear.x = 0.0
            self.vel.angular.z = 0.0
            raise KeyboardInterrupt
        else:
            print("Invalid input")

def main(args=None):
    rclpy.init(args=args)
    node = Teleop()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()