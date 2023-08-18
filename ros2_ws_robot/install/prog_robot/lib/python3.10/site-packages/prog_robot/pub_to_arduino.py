import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray, Int32
from sensor_msgs.msg import JointState
import math
from copy import deepcopy
from prog_robot.fonctions import ang2Step

class Pub2Arduino(Node):
    """ Récupère les informations du topic /joint_states et les transmets au robot."""
    def __init__(self):

        super().__init__('pub_to_arduino')
        
        self.publisher_ = self.create_publisher(Int32MultiArray, '/micro_ros_arduino_subscriber', 10)

        self.timer = self.create_timer(0.0001, self.timer_callback)

        self.sub_joint_states = self.create_subscription(JointState,'/joint_states', self.joint_states_callback, 10)
        self.sub_joint_states  # prevent unused variable warning

        self.sub_robot = self.create_subscription(Int32,'/micro_ros_arduino_node_publisher', self.robot_callback, 10)
        self.sub_robot  # prevent unused variable warning


        self.pub = Int32MultiArray()
        self.old_msg = JointState()
        self.all_pub = []

        self.robot_state = False
        self.get_logger().info("Waiting for the robot...")


    def joint_states_callback(self, msg):
        if msg.position != self.old_msg.position:
            ang_rad = [0,0,0,0,0,0]
            ang_rad[0] = msg.position[2]
            ang_rad[1] = msg.position[0]
            ang_rad[2] = msg.position[1]
            ang_rad[3] = msg.position[3]
            ang_rad[4] = msg.position[4]
            ang_rad[5] = msg.position[5]

            step = ang2Step(ang_rad, "rad")
            pos_gripper = (msg.position[6]+math.pi)*180/math.pi
            speed = 50
            accel = 30
            self.pub.data = step + [(int)(pos_gripper)] + [(int)(speed)] + [(int)(accel)]
            
            # self.get_logger().info('Pos data: "%s"' % len(self.pub.data))

            self.all_pub.append(deepcopy(self.pub))
            # self.old = deepcopy(self.pub)
            
            self.get_logger().info('Pos count: "%s"' % len(self.all_pub))

            self.old_msg = msg
        
        
    def robot_callback(self, msg):
        self.robot_state = True
        self.get_logger().info("Robot is ready !")
    
    def timer_callback(self):
        if self.robot_state and self.all_pub != []:
            pub = Int32MultiArray()
            pub = self.all_pub.pop(0)
            self.publisher_.publish(pub)
            self.get_logger().info('Publishing: "%s"' % pub.data)
            self.get_logger().info('Pos count: "%s"' % len(self.all_pub))
            self.robot_state = False
        # else:
        #     self.get_logger().info("Waiting for the robot...")
        


def main(args=None):
    rclpy.init(args=args)

    publisher = Pub2Arduino()

    rclpy.spin(publisher)

    publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()