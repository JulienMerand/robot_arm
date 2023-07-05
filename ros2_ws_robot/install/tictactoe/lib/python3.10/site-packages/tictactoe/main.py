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
        self.publisher_ = self.create_publisher(Float32MultiArray, '/pose_xyz_speed', 10)
        self.publisher_ = self.create_publisher(Int32, '/pose_gripper', 10)

        self.pose = Float32MultiArray()
        self.gripper = Int32()
        self.player = input("[MASTER] Qui commence ? 'ROBOT' ou 'HUMAN' ? ")
        self.squares_coords = []

        self.play()    

    def move_robot(slef, case):
        print(f"[ROBOT] Je bouge sur la case : {case}, n'oublies pas d'appuyer sur SPACE lorsque tu as joué.", )

    def play(self):

        if self.player == 'ROBOT' or self.player == 'HUMAN': 
            game = Tictactoe(self.player)
            vision = Vision_Tictactoe('https://192.168.1.16:8080/video')

            print("[ROBOT] Je localise mes pieces...")
            green_pieces_coords = vision.get_coords_green_pieces()
            print(f"[ROBOT] Je les ai trouvées, elles sont là : {green_pieces_coords}, surtout n'y touche pas je ne veux pas avoir à les rechercher.")
            
            # Initialisation de pygame
            pygame.init()

            print("[MASTER] Debut du jeu...")
            ret, move = game.start()
            if ret:
                self.move_robot(move)

            # Boucle principale du jeu
            running = True
            while running:

                for event in pygame.event.get():
                    if event.type == pygame.QUIT:
                        running = False
                        break
                    elif event.type == pygame.KEYDOWN:
                        if event.key == pygame.K_SPACE and game.current_player == game.HUMAN:
                            
                            vision_board, self.squares_coords, full_board_img = vision.get_board()
                            # print("[CAMERA] Je vois ça : ", vision_board)

                            ret0, ret1, (i, j) = game.compare_board(vision_board)
                            if ret0:
                                if ret1:
                                    print("[GAME] J'ai noté ton coup, tu as joué en ", (i, j))
                                    bool, move = game.next_move(i, j)
                                    # print("[CAMERA] Maintenant j'ai ça : ", game.board)
                                    if bool:
                                        # print('ici')
                                        self.move_robot(move)
                                        break
                                else:
                                    # print('la')
                                    self.move_robot((i, j))
                                    break
                            print("[MASTER] Tu n'as pas joué...")

                game.update_board()

                # Vérification du résultat
                result = game.check_winner()
                # running = game.display_result(result)
                game.display_result(result)

                # Mise à jour de l'affichage
                pygame.display.flip()

                if not running:
                    time.sleep(1.5)

            # Fermeture de pygame
            pygame.quit()


        else:
            print("Erreur : Mauvaise saisie.")


def main(args=None):
    rclpy.init(args=args)

    game = NodeTictactoe()

    rclpy.spin_once(game, timeout_sec=1)

    game.destroy_node()
    rclpy.shutdown()


# if __name__ == '__main__':
    # main()