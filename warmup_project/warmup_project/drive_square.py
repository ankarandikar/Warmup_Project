import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class DriveSquare(Node):
    def __init__(self):
        super().__init__('drive_square')
        self.start_time = time.time()
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.run_loop)
        self.vel = Twist()
        self.vel_publisher = self.create_publisher(Twist, "cmd_vel", 10)
        #self.elapsed_time = time.time()-self.start_time
        self.linear_vel = 1.0
        self.angular_vel = 1.0
    
    def vel_zero(self):
        self.vel.linear.x = 0.0
        self.vel.linear.y = 0.0
        self.vel.angular.z = 0.0
        self.vel_publisher.publish(self.vel)
    
    def drive_straight(self):
        self.vel.linear.x = self.linear_vel
        self.vel_publisher.publish(self.vel)
    
    def turn(self):
        self.vel.angular.z = self.angular_vel
        self.vel_publisher.publish(self.vel)
    
    def run_loop(self):
        pause = time.sleep(2.0)
        elapsed_time = int(time.time()-self.start_time-pause)
        drive_time = 1.0/self.linear_vel
        turn_time = 1.5708/self.angular_vel
        if elapsed_time < drive_time:
            self.drive_straight()
            self.vel_zero()
        else:
            self.vel_zero()
        """elif elapsed_time < drive_time + turn_time:
            self.turn()
            self.vel_zero()
        elif elapsed_time < 2*drive_time + turn_time:
            self.drive_straight()
            self.vel_zero()
        elif elapsed_time < 2*drive_time + 2*turn_time:
            self.turn()
            self.vel_zero()
        elif elapsed_time < 3*drive_time + 2*turn_time:
            self.drive_straight()
            self.vel_zero()
        elif elapsed_time < 3*drive_time + 3*turn_time:
            self.turn()
            self.vel_zero()
        elif elapsed_time < 4*drive_time + 3*turn_time:
            self.drive_straight()
            self.vel_zero()
        elif elapsed_time < 4*drive_time + 4*turn_time:
            self.turn()
            self.vel_zero()"""
        

def main(args=None):
    rclpy.init(args=args)
    node = DriveSquare()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()