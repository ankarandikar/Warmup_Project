import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from neato2_interfaces.msg import Bump
import math

class ObstacleAvoider(Node):
    def __init__(self):
        super().__init__('obstacle_avoider')
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.run_loop)
        self.vel_publisher = self.create_publisher(Twist, "cmd_vel", 10)
        self.scan = None
        self.subscriber = self.create_subscription(LaserScan, "scan", self.process_scan, 10)
        self.bump_subscriber = self.create_subscription(Bump, "bump", self.detect_bump, 10)
        self.bumper_active = False
        self.scan : LaserScan = []
        self.obstacle_angle = []

    def process_scan(self,message):
        self.scan = message

    def detect_bump(self,vel):
        self.bumper_active = (vel.left_front == 1 or \
                              vel.left_side == 1 or \
                              vel.right_front ==1 or \
                              vel.right_side == 1)
    
    def process_angles(self):
        angles = []
        distances = []
        x_values = []
        y_values = []
        if not self.scan:
            return
        for i,n in enumerate(self.scan.ranges):
            if (n > 5) and (i < 120 or i > 240):
                angles.append(i)
                distances.append(i)
        if len(self.angles) < 5:
            return
        for i,n in enumerate(angles):
            x_values.append(distances[i]*math.cos(math.radians(n)))
            y_values.append(distances[i]*math.sin(math.radians(n)))
        x_COM = sum(x_values)/len(x_values)
        y_COM = sum(y_values)/len(y_values)
        self.obstacle_angle = math.degrees(math.atan(y_COM/x_COM))

        
    def run_loop(self):
        vel = Twist()
        if not self.scan:
            return
        if not self.bumper_active:
            if self.obstacle_angle:
                vel.angular.z = 2*y_COM
                vel.linear.x = 0.2*x_COM
                self.vel_publisher.publish(vel)
        else:
            vel.linear.x = 0.0
            vel.linear.y = 0.0
            vel.angular.z = 0.0
            self.vel_publisher.publish(vel)

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoider()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()