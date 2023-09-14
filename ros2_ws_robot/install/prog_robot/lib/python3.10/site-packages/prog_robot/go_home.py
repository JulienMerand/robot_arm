""" 
Envoie directement au robot les positions articulaires correspondantes à sa position 'home'.

"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray, Int32
import time,math
from prog_robot.fonctions import ang2Step
from prog_robot.kinematics import inverse_kinematics

class GoHome(Node):
    def __init__(self):

        super().__init__('pub_to_arduino')
        self.publisher_ = self.create_publisher(Int32MultiArray, '/micro_ros_arduino_subscriber', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.sub_robot = self.create_subscription(Int32,'/micro_ros_arduino_node_publisher', self.robot_callback, 10)

        self.pub = Int32MultiArray()
        self.gripper_open = 90
        self.gripper_close = 180

        self.robot_state = -1

        # Position 'home'
        ang_deg = [0,0,0,0,0,0]

        self.step = ang2Step(ang_deg, "deg")
        self.speed = 25
        self.accel = 30

        print("En attente du robot...")

    def robot_callback(self, msg):
        self.robot_state = True
        print('Le robot est prêt...')
    
    def timer_callback(self):
        if self.robot_state:
            self.pub.data = self.step + [(int)(self.gripper_open)] + [(int)(self.speed)] + [(int)(self.accel)]
            self.publisher_.publish(self.pub)
            self.get_logger().info('Publishing: "%s"' % self.pub.data)
            self.robot_state = False

def main(args=None):
    rclpy.init(args=args)

    publisher = GoHome()

    rclpy.spin(publisher)

    publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()