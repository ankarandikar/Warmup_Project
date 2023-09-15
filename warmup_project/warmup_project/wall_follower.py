import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from neato2_interfaces.msg import Bump

class WallFollower(Node):
    def __init__(self):
        super().__init__('wall_follower')
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.run_loop)
        self.vel_publisher = self.create_publisher(Twist, "cmd_vel", 10)
        self.scan = None
        self.subscriber = self.create_subscription(LaserScan, "scan", self.process_scan, 10)
        self.bump_subscriber = self.create_subscription(Bump, "bump", self.detect_bump, 10)
        self.bumper_active = False
        
    def process_scan(self,message):
        self.scan = message

    def detect_bump(self,vel):
        self.bumper_active = (vel.left_front == 1 or \
                              vel.left_side == 1 or \
                              vel.right_front ==1 or \
                              vel.right_side == 1)

    def run_loop(self):
        vel = Twist()
        scan : LaserScan = self.scan
        if not self.scan:
            return
        dist_45 = scan.ranges[46]
        dist_135 = scan.ranges[136]
        dist_90 = scan.ranges[91]
        if not self.bumper_active:
            vel.linear.x = 0.1
            self.vel_publisher.publish(vel)
            if dist_90 == 0.0:
                dist_90 = 3.0
            if dist_45 > dist_135: #turn right (towards wall)
                vel.angular.z = -0.2*dist_90
                vel.linear.x = 0.1
                self.vel_publisher.publish(vel)
            elif dist_45 < dist_135: #turn left (away from wall)
                vel.angular.z = 0.4/dist_90
                vel.linear.x = 0.1
                self.vel_publisher.publish(vel)
            else: #dist_45 == dist_135 (parallel to wall)
                if dist_90 > 0.5:
                    vel.angular.z = -0.05
                    vel.linear.x = 0.1
                    self.vel_publisher.publish(vel)
                elif dist_90 < 0.5:
                    vel.angular.z = 0.05
                    vel.linear.x = 0.1
                    self.vel_publisher.publish(vel)
                else:
                    vel.angular.z = 0.0
                    vel.linear.x = 0.1
                    self.vel_publisher.publish(vel)
        else:
            vel.linear.x = 0.0
            vel.linear.y = 0.0
            vel.angular.z = 0.0
            self.vel_publisher.publish(vel)

def main(args=None):
    rclpy.init(args=args)
    node = WallFollower()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()