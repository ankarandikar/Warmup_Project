import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import tty
import select
import sys
import termios

class WallFollower(Node):
    def __init__(self):
        super().__init__('wall_follower')
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.run_loop)
        self.vel_publisher = self.create_publisher(Twist, "cmd_vel", 10)
        self.scan = None
        self.subscriber = self.create_subscription(LaserScan, "scan", self.process_scan, 10)

    def process_scan(self,message):
        self.scan = message

    def getKey(self):
        settings = termios.tcgetattr(sys.stdin)
        key = None
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return key

    def run_loop(self):
        key = self.getKey()
        key = 'a'
        vel = Twist()
        scan : LaserScan = self.scan
        #if scan.range_max == 0.0:
        #    raise KeyboardInterrupt
        if not self.scan:
            return
        dist_45 = scan.ranges[46]
        dist_135 = scan.ranges[136]
        dist_90 = scan.ranges[91]
        # elif key == 's':
        #     vel.linear.x = 0.0
        #     self.vel_publisher.publish(vel)
        while True:
            vel.linear.x = 0.1
            self.vel_publisher.publish(vel)
            if dist_90 == 0.0:
                dist_90 = 3.0
            if dist_45 > dist_135: #turn right (towards wall)
                vel.angular.z = -0.2*dist_90
                vel.linear.x = 0.1
                self.vel_publisher.publish(vel)
            elif dist_45 < dist_135: #turn left (away from wall)
                vel.angular.z = 0.2/dist_90
                vel.linear.x = 0.1
                self.vel_publisher.publish(vel)
            else: #dist_45 == dist_135
                if dist_90 > 0.2:
                    vel.angular.z = -0.05
                    vel.linear.x = 0.1
                    self.vel_publisher.publish(vel)
                elif dist_90 < 0.2:
                    vel.angular.z = 0.05
                    vel.linear.x = 0.1
                    self.vel_publisher.publish(vel)
                else:
                    vel.angular.z = 0.0
                    vel.linear.x = 0.1
                    self.vel_publisher.publish(vel)
            if key == 's':
                vel.linear.x = 0.0
                self.vel_publisher.publish(vel)
            if key == '\x03':
                raise KeyboardInterrupt

def main(args=None):
    rclpy.init(args=args)
    node = WallFollower()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()