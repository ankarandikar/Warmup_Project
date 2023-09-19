import rclpy
from rclpy.node import Node

class FiniteStateController(Node):
    def __init__(self):
        super().__init__('finite_state_controller')
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.run_loop)
    
    def process_scan(self,message):
        self.scan = message

    def detect_bump(self,vel):
        self.bumper_active = (vel.left_front == 1 or \
                              vel.left_side == 1 or \
                              vel.right_front == 1 or \
                              vel.right_side == 1)

    def wall_follower(self):
        vel = Twist()
        if not self.scan:
            return
        dist_45 = self.scan.ranges[45]
        dist_135 = self.scan.ranges[135]
        dist_90 = (self.scan.ranges[85]+self.scan.ranges[95])/2
        if not self.bumper_active:
            vel.linear.x = 0.1
            self.vel_publisher.publish(vel)
            if dist_90 == 0.0:
                dist_90 = 3.0
            if dist_45 > dist_135: #turn left (towards wall)
                vel.angular.z = 0.1*dist_90
                vel.linear.x = 0.1
                self.vel_publisher.publish(vel)
            elif dist_45 < dist_135: #turn right (away from wall)
                vel.angular.z = -0.1/dist_90
                vel.linear.x = 0.1
                self.vel_publisher.publish(vel)
            else: #dist_45 == dist_135 (parallel to wall)
                if dist_90 > 0.3:
                    vel.angular.z = 0.05
                    vel.linear.x = 0.1
                    self.vel_publisher.publish(vel)
                elif dist_90 < 0.3:
                    vel.angular.z = -0.05
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

    def person_following(self):
        vel = Twist()
        if not self.scan:
            return
        dist_45 = self.scan.ranges[45]
        dist_135 = self.scan.ranges[135]
        dist_90 = self.scan.ranges[90]
        if not self.bumper_active:
            vel.linear.x = 0.1
            self.vel_publisher.publish(vel)
            if dist_90 == 0.0:
                dist_90 = 3.0
            if dist_45 > dist_135: #turn left (towards wall)
                vel.angular.z = 0.1*dist_90
                vel.linear.x = 0.1
                self.vel_publisher.publish(vel)
            elif dist_45 < dist_135: #turn right (away from wall)
                vel.angular.z = -0.8/dist_90
                vel.linear.x = 0.1
                self.vel_publisher.publish(vel)
            else: #dist_45 == dist_135 (parallel to wall)
                if dist_90 > 0.5:
                    vel.angular.z = 0.05
                    vel.linear.x = 0.1
                    self.vel_publisher.publish(vel)
                elif dist_90 < 0.5:
                    vel.angular.z = -0.05
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
    
    def run_loop(self):
        pass

def main(args=None):
    rclpy.init(args=args)
    node = FiniteStateController()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()