""" 
Récupère le flux vidéo sur le serveur, enlève la distortion crée par la caméra et publie l'image sur le topic /camera/frames.

"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import pickle

class Camera(Node):
    def __init__(self):
        super().__init__('camera_node')
        self.cam_publisher = self.create_publisher(Image, '/camera/frames', 10)
        self.timer = self.create_timer(0.0333, self.timer_callback)

        self.IP = 'https://192.168.1.16:8080/video'
        self.cap = cv2.VideoCapture(self.IP)
        self.br = CvBridge()

        # Get camera parameters
        file_path = '/home/julien/Documents/Python/calibration/parametres_camera.pkl'
        with open(file_path, 'rb') as f:
            parameters = pickle.load(f)
        
        self.extrinsic_matrix = parameters['extrinsic matrix']
        self.intrinsic_matrix = parameters['intrinsic matrix']
        self.dist_coeffs = parameters['distortion']
        self.rot_vecs = parameters['rotation']
        self.trans_vecs = parameters['translation']

        self.get_logger().info('Publishing video frame...')
    
    def timer_callback(self):
        ret, frame = self.cap.read()
        frame = cv2.undistort(frame,self.intrinsic_matrix, self.dist_coeffs)
        if ret == True:
            self.cam_publisher.publish(self.br.cv2_to_imgmsg(frame))

def main(args=None):
    rclpy.init(args=args)

    publisher = Camera()

    rclpy.spin(publisher)

    publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()