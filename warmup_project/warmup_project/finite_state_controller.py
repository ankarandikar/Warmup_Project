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
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.run_loop)
        self.vel_publisher = self.create_publisher(Twist, "cmd_vel", 10)
        self.subscriber = self.create_subscription(LaserScan, "scan", self.process_scan, 10)
        self.bump_subscriber = self.create_subscription(Bump, "bump", self.detect_bump, 10)
        self.bumper_active = False  # Set bumper input to false
        self.scan : LaserScan = None    # Create instance of LaserScan
        self.obstacle_angle = 0
        self.x_COM = 0
        self.y_COM = 0
        self.angles = []
        self.person_angles = []
    
    def process_scan(self,message):
        self.scan = message

    def detect_bump(self,vel):
        self.bumper_active = (vel.left_front == 1 or \
                              vel.left_side == 1 or \
                              vel.right_front == 1 or \
                              vel.right_side == 1)

    def process_angles(self):
        distances = []
        x_values = []
        y_values = []
        self.angles = []
        if not self.scan:
            return
        for i,n in enumerate(self.scan.ranges):  # filter relevant values
            if (n < 1):
                self.angles.append(i)
                distances.append(n)
        if len(self.angles) < 5:
            return
        for i,n in enumerate(self.angles):
            x_values.append(1)#distances[i]*math.cos(math.radians(n)))
            y_values.append(1)#distances[i]*math.sin(math.radians(n)))
        self.x_COM = sum(x_values)/len(x_values)
        self.y_COM = sum(y_values)/len(y_values)
        if self.y_COM == 0:
            return
        if self.x_COM > 0:
            self.obstacle_angle = math.degrees(math.atan(self.y_COM/self.x_COM))
        else:
            self.obstacle_angle = 180+math.degrees(math.atan(self.y_COM/self.x_COM))
        self.person_angles = self.scan.ranges[0:30]+self.scan.ranges[330:360]

    def wall_follower(self):
        vel = Twist()   # Create instance of Twist
        if not self.scan:   # Wait to run code until scan has been received
            return
        # Index distances at 45, 90, and 135 degrees
        dist_45 = self.scan.ranges[55]
        dist_135 = self.scan.ranges[135]
        # Sometimes the Neatos return 0.0 at degrees near 90, so I took the averages of the distances at 85 and 95 degrees
        dist_90 = (self.scan.ranges[85]+self.scan.ranges[95])/2

        # Wrap velocity commands in if statement so that Neato stops when bumpers are active
        if not self.bumper_active:  # If bumpers are not active
            vel.linear.x = 0.1
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
        vel = Twist()
        angles = []
        distances = []
        x_values = []
        y_values = []
        if not self.scan:
            return
        for i,n in enumerate(self.scan.ranges):  # filter relevant values
            if (n < 1.5) and (i < 30 or i > 330):
                angles.append(i)
                distances.append(n)
        if len(angles) < 5:
            return
        for i,n in enumerate(angles):
            x_values.append(distances[i]*math.cos(math.radians(n)))
            y_values.append(distances[i]*math.sin(math.radians(n)))
        x_COM = sum(x_values)/len(x_values)
        y_COM = sum(y_values)/len(y_values)
        vel.angular.z = 16.0*y_COM
        vel.linear.x = 4.0*x_COM
        self.vel_publisher.publish(vel)
    
    def run_loop(self):
        vel = Twist()
        if not self.scan:
            return
        self.process_angles()
        print(len(self.angles))
        if not self.bumper_active:
            if sum(self.person_angles) != 0:
                self.person_following()
                print("Person following")
            else:
                self.wall_follower()
                print("Wall following")
        else:
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