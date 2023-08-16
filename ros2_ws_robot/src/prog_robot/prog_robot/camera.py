import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


class Camera(Node):

    def __init__(self):
        super().__init__('camera_node')
        self.cam_publisher = self.create_publisher(Image, '/camera/frames', 10)
        self.timer = self.create_timer(0.0333, self.timer_callback)

        self.IP = 'https://192.168.1.16:8080/video'
        self.cap = cv2.VideoCapture(self.IP)
        self.br = CvBridge()
    
    def timer_callback(self):
        ret, frame = self.cap.read()
        if ret == True:
            self.cam_publisher.publish(self.br.cv2_to_imgmsg(frame))
            self.get_logger().info('Publishing video frame')

def main(args=None):
    rclpy.init(args=args)

    publisher = Camera()

    rclpy.spin(publisher)

    publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()