import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Int32MultiArray, Int32
from tictactoe.game_algorithm import Tictactoe
from tictactoe.vision import Vision_Tictactoe

import pygame
import time

class NodeTictactoe(Node):

    def __init__(self):

        super().__init__('tictactoe')
        self.publisher_pose = self.create_publisher(Float32MultiArray, '/pose_xyz_speed', 10)
        self.publisher_gripper = self.create_publisher(Int32, '/pose_gripper', 10)

        self.pose = Float32MultiArray()
        self.gripper = Int32()
        self.speed = (float)(35.0)

        # Init du robot
        self.robot_ready()
        self.open_gripper()
        
        self.player = input("[MASTER] Qui commence ? 'ROBOT' ou 'HUMAN' ? ")
        self.squares_coords = []
        self.green_pieces_coords = []

        # Jeu
        if self.player == 'ROBOT' or self.player == 'HUMAN': 
            self.play()
        else:
            print("Erreur : Mauvaise saisie.")

        
    def robot_ready(self):
        self.pose.data = [225.0, 0.0, 350.0, 0.0, 120.0, 0.0] + [self.speed]
        self.publisher_pose.publish(self.pose)
        # self.moveJ([225.0, 0.0, 350.0, 0.0, 120.0, 0.0])
        time.sleep(0.2)

    def moveJ(self, pos):
        for i in range(len(pos)):
            pos[i] = float(pos[i])
        self.pose.data = pos + [self.speed]
        self.publisher_pose.publish(self.pose)
        time.sleep(0.2)

    def open_gripper(self):
        self.gripper.data = (int)(90)
        self.publisher_gripper.publish(self.gripper)
        time.sleep(0.2)

    def close_gripper(self):
        self.gripper.data = (int)(180)
        self.publisher_gripper.publish(self.gripper)
        time.sleep(0.2)

    def move_robot(self, case):
        (i, j) = case
        num_case = 3*j + i
        print(f"[ROBOT] Je bouge sur la case : {num_case}, n'oublies pas d'appuyer sur SPACE lorsque tu as joué.")
        # Prendre la première pièce
        [x, y, ori] = self.green_pieces_coords.pop(0)
        self.moveJ([x, y, 50, ori, 180, 0]) # Approche
        self.moveJ([x, y, 5, ori, 180, 0]) # Prise
        self.close_gripper()
        self.moveJ([x, y, 100, ori, 180, 0]) # Approche
        [x_case, y_case, theta] = self.squares_coords[num_case]
        self.moveJ([x_case, y_case, 50, theta, 180, 0]) # Approche
        self.moveJ([x_case, y_case, 5, theta, 180, 0]) # Prise
        self.open_gripper()
        self.moveJ([x_case, y_case, 100, theta, 180, 0]) # Approche
        self.robot_ready()

    def play(self):
        
        self.game = Tictactoe(self.player)
        self.vision = Vision_Tictactoe('https://192.168.1.16:8080/video')

        print("[ROBOT] Je localise mes pieces...")
        self.green_pieces_coords = self.vision.get_coords_green_pieces()
        # print(f"[ROBOT] Je les ai trouvées, elles sont là : {self.green_pieces_coords}, surtout n'y touche pas je ne veux pas avoir à les rechercher.")
        print("[ROBOT] Je localise le plateau...")
        _, self.squares_coords, _ = self.vision.get_board()
        print("[ROBOT] OK !")

        print("[MASTER] Debut du jeu...")
        ret, move = self.game.start()
        if ret:
            self.move_robot(move)
        
        # Initialisation de pygame
        pygame.init()

        # Boucle principale du jeu
        running = True
        while running:
            
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                    break
                elif event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_SPACE and self.game.current_player == self.game.HUMAN:
                        print("[ROBOT] Je localise mes pieces...")
                        self.green_pieces_coords = self.vision.get_coords_green_pieces()
                        # print(f"[ROBOT] Je les ai trouvées, elles sont là : {self.green_pieces_coords}, surtout n'y touche pas je ne veux pas avoir à les rechercher.")
                        print("[ROBOT] Je localise le plateau...")
                        vision_board, self.squares_coords, full_board_img = self.vision.get_board()
                        print("[ROBOT] OK !")
                        # print(self.squares_coords)
                        # print("[CAMERA] Je vois ça : ", vision_board)

                        ret0, ret1, (i, j) = self.game.compare_board(vision_board)
                        if ret0:
                            if ret1:
                                print(f"[MASTER] J'ai noté ton coup, tu as joué en {3*j + i}")
                                bool, move = self.game.next_move(i, j)
                                # print("[CAMERA] Maintenant j'ai ça : ", game.board)
                                if bool:
                                    self.move_robot(move)
                                    break
                            else:
                                self.move_robot((i, j))
                                break
                        print("[MASTER] Tu n'as pas joué...")

            self.game.update_board()

            # Vérification du résultat
            result = self.game.check_winner()
            # running = game.display_result(result)
            self.game.display_result(result)

            # Mise à jour de l'affichage
            pygame.display.flip()

            if not running:
                time.sleep(1.5)

        # Fermeture de pygame
        pygame.quit()
        
        



def main(args=None):
    rclpy.init(args=args)

    game = NodeTictactoe()

    rclpy.spin_once(game, timeout_sec=1)

    game.destroy_node()
    rclpy.shutdown()


# if __name__ == '__main__':
    # main()