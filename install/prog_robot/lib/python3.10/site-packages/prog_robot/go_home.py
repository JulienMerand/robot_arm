import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray, Int32
import time,math
from prog_robot.fonctions import ang2Step
from prog_robot.kinematics import inverse_kinematics

class MinimalPublisher(Node):

    def __init__(self):

        super().__init__('pub_to_arduino')
        self.publisher_ = self.create_publisher(Int32MultiArray, '/micro_ros_arduino_subscriber', 10)
        # self.sub_arduino = self.create_subscription(Int32,'/micro_ros_arduino_node_publisher', self.arduino_callback, 10)

        self.pub = Int32MultiArray()
        self.gripper_open = 90
        self.gripper_close = 180


        
    # def arduino_callback(self, msg):
        ang_deg = [0,0,0,0,0,0]
        step = ang2Step(ang_deg, "deg")
        speed = 50
        accel = 70
        self.pub.data = step + [(int)(self.gripper_open)] + [(int)(speed)] + [(int)(accel)]
        self.publisher_.publish(self.pub)
        self.get_logger().info('Publishing: "%s"' % self.pub.data)



def main(args=None):
    rclpy.init(args=args)

    publisher = MinimalPublisher()

    rclpy.spin(publisher)

    publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()