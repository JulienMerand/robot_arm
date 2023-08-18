import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Int32MultiArray, Int32
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from tictactoe.game_algorithm import Tictactoe
from tictactoe.vision import Vision_Tictactoe

import pygame
import time
import numpy as np

class NodeTictactoe(Node):

    def __init__(self):

        super().__init__('tictactoe')
        self.publisher_pose = self.create_publisher(Float32MultiArray, '/pose_xyz_speed', 10)
        self.publisher_gripper = self.create_publisher(Int32, '/pose_gripper', 10)
        self.cam_subscriber = self.create_subscription(Image, '/camera/frames', self.update_frame, 10)
        self.br = CvBridge()

        self.frame = None

        self.delay = 1
        self.pose = Float32MultiArray()
        self.gripper = Int32()
        self.speed = (float)(40.0)

        # Init du robot
        self.init_robot()
        time.sleep(self.delay)
        self.open_gripper()
        time.sleep(self.delay)
        
        self.squares_coords = []
        self.green_pieces_coords = []
        self.player = 'ROBOT'

    def update_frame(self, data):
        # Convert ROS Image message to OpenCV image and update self.frame
        self.frame = self.br.imgmsg_to_cv2(data, desired_encoding='passthrough')
        
        
    def init_robot(self):
        self.pose.data = [250.0, 0.0, 350.0, 0.0, 120.0, 0.0] + [self.speed]
        self.publisher_pose.publish(self.pose)

    def robot_ready(self):
        self.pose.data = [250.0, 200.0, 100.0, 0.0, 180.0, 0.0] + [self.speed]
        self.publisher_pose.publish(self.pose)
        
    def moveJ(self, pos):
        for i in range(len(pos)):
            pos[i] = float(pos[i])
        self.pose.data = pos + [self.speed]
        self.publisher_pose.publish(self.pose)

    def open_gripper(self):
        self.gripper.data = (int)(90)
        self.publisher_gripper.publish(self.gripper)

    def close_gripper(self):
        self.gripper.data = (int)(180)
        self.publisher_gripper.publish(self.gripper)

    def move_robot(self, case):
        (i, j) = case
        num_case = 3*j + i
        print(f"[ROBOT] Je bouge sur la case : {num_case}, n'oublies pas d'appuyer sur SPACE lorsque tu as joué.")
        # Prendre la pièce et la déposer sur la case demandée
        [x, y, ori] = self.green_pieces_coords.pop(0)
        self.moveJ([x, y, 50, ori, 180, 0])                     # Approche
        time.sleep(self.delay)
        self.moveJ([x, y, 5, ori, 180, 0])                      # Prise
        time.sleep(self.delay)
        self.close_gripper()                                    # Gripper
        time.sleep(self.delay)
        self.moveJ([x, y, 100, ori, 180, 0])                    # Approche
        time.sleep(self.delay)
        [x_case, y_case, theta] = self.squares_coords[num_case]
        self.moveJ([x_case, y_case, 50, theta, 180, 0])         # Approche
        time.sleep(self.delay)
        self.moveJ([x_case, y_case, 5, theta, 180, 0])          # Prise
        time.sleep(self.delay)
        self.open_gripper()                                     # Gripper
        time.sleep(self.delay)
        self.moveJ([x_case, y_case, 100, theta, 180, 0])        # Approche
        time.sleep(self.delay)
        self.robot_ready()                                      # Ready
        time.sleep(self.delay)



def main(args=None):
    rclpy.init(args=args)

    node = NodeTictactoe()
    rclpy.spin_once(node)

    node.player = input("[MASTER] Qui commence ? 'ROBOT' ou 'HUMAIN' ?")

    # Jeu   
    if node.player == 'ROBOT' or node.player == 'HUMAIN': 

        # Initialisation de pygame
        pygame.init()

        # Début du jeu
        game = Tictactoe(node.player)
        vision = Vision_Tictactoe()
        rclpy.spin_once(node)
        # print("[ROBOT] Je localise mes pieces...")
        node.green_pieces_coords = vision.get_coords_green_pieces(node.frame)
        # print(f"[ROBOT] Je les ai trouvées, elles sont là : {node.green_pieces_coords}.")
        # print("[ROBOT] Je localise le plateau...")
        _, node.squares_coords, _ = vision.get_board(node.frame)
        # print("[ROBOT] OK !")
        print("[MASTER] Debut du jeu...")
        is_robot_first, move = game.start()
        
        pygame.display.flip()
        
        if is_robot_first:
            node.move_robot(move)
        rclpy.spin_once(node)

        # Boucle principale du jeu
        running = True
        end_game = False
        end_game_msg_displayed = False
        while running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                    break
                elif event.type == pygame.KEYDOWN and not end_game:
                    if event.key == pygame.K_SPACE and game.current_player == game.HUMAN:
                        # node.cam_subscriber = self.create_subscription(Image, '/camera/frames', self.update_frame, 1)
                        rclpy.spin_once(node)
                        # print("[ROBOT] Je localise mes pieces...")
                        node.green_pieces_coords = vision.get_coords_green_pieces(node.frame)
                        # print(f"[ROBOT] Je les ai trouvées, elles sont là : {self.green_pieces_coords}, surtout n'y touche pas je ne veux pas avoir à les rechercher.")
                        # print("[ROBOT] Je localise le plateau...")
                        vision_board, node.squares_coords, full_board_img = vision.get_board(node.frame)
                        # print("[ROBOT] OK !")

                        ret0, ret1, (i, j) = game.compare_board(vision_board)
                        if ret0:
                            if ret1:
                                print(f"[MASTER] J'ai noté ton coup, tu as joué en {3*j + i}")
                                bool, move = game.next_move(i, j)
                                # print("[CAMERA] Maintenant j'ai ça : ", game.board)
                                if bool:
                                    node.move_robot(move)
                                    break
                            else:
                                node.move_robot((i, j))
                                break
                        print("[MASTER] Tu n'as pas joué...")

            game.update_board()
            rclpy.spin_once(node)

            # Vérification du résultat
            result = game.check_winner()
            end_game = not game.display_result(result)
            # game.display_result(result)

            # Mise à jour de l'affichage
            pygame.display.flip()
            
            if end_game:
                if not end_game_msg_displayed:
                    print("[MASTER] Fin de la partie. Clique sur la croix de la fenêtre pygame pour quitter.")
                    end_game_msg_displayed = True

            if not running:
                time.sleep(0.5)

        # Fermeture de pygame
        pygame.quit()
    else:
        print("[MASTER] Erreur : Mauvaise saisie.")



    node.destroy_node()
    rclpy.shutdown()


# if __name__ == '__main__':
    # main()