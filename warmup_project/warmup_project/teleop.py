import tty
import select
import sys
import termios

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class Teleop(Node):
    def __init__(self):
        super().__init__('teleop')
    
    def getKey():
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return key

def main(args=None):
    rclpy.init(args=args)
    node = Teleop()
    rclpy.spin(node)

    settings = termios.tcgetattr(sys.stdin)
    key = None

    while key != '\x03':
        key = getKey()
        print(key)

    rclpy.shutdown()

if __name__ == '__main__':
    main()