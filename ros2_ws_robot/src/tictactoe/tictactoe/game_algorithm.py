import pygame
import time

class Tictactoe():
    """ Tictactoe game !
        \nstarter = 'ROBOT' or 'HUMAN' """
    def __init__(self, starter):

        # Définition des couleurs
        self.BLACK = (0, 0, 0)
        self.WHITE = (255, 255, 255)
        self.RED = (255, 0, 0)

        # Définition des dimensions de l'écran
        self.WIDTH = 400
        self.HEIGHT = 400

        # Définition des dimensions des cases
        self.W = self.WIDTH // 3
        self.H = self.HEIGHT // 3

        # Définition des symboles pour le joueur et l'IA
        self.starter = starter
        self.AI = 'X'
        self.HUMAN = 'O'

        # Initialisation du joueur courant
        self.current_player = self.HUMAN

        # Initialisation de la grille
        self.board = [
            ['-', '-', '-'],
            ['-', '-', '-'],
            ['-', '-', '-']
        ]

        # Scores pour l'algorithme Minimax
        self.scores = {
            'X': 10,
            'O': -10,
            'tie': 0
        }

        self.screen = self.setup()

    def start(self):
        ''' Joue le coup du robot si c'est le premier joueur.'''
        if self.starter=='ROBOT':
            print("[MASTER] Le robot commence.")
            # IA qui commence
            move = self.best_move()
            self.current_player = self.HUMAN
            return True, move
        else:
            print("[MASTER] A toi de commencer. N'oublies pas d'appuyer sur la touche SPACE une fois que tu as joué.")
            return False, 0 

    
    def setup(self):
        ''' Initialisation du plateau de jeu '''
        
        # Initialisation de la fenêtre du jeu
        screen = pygame.display.set_mode((self.WIDTH, self.HEIGHT))
        pygame.display.set_caption("Tic Tac Toe")

        # Affichage du plateau de jeu
        screen.fill(self.WHITE)
        pygame.draw.line(screen, self.BLACK, (self.W, 0), (self.W, self.HEIGHT), 4)
        pygame.draw.line(screen, self.BLACK, (self.W * 2, 0), (self.W * 2, self.HEIGHT), 4)
        pygame.draw.line(screen, self.BLACK, (0, self.H), (self.WIDTH, self.H), 4)
        pygame.draw.line(screen, self.BLACK, (0, self.H * 2), (self.WIDTH, self.H * 2), 4)

        return screen

    
    def equals3(self, a, b, c):
        '''Fonction pour vérifier si trois éléments sont égaux'''
        return a == b and b == c and a != '-'

    def check_winner(self):
        ''' Vérifie si il y a un gagnant dans self.board.'''
        winner = None

        # Vérification des lignes horizontales
        for i in range(3):
            if self.equals3(self.board[i][0], self.board[i][1], self.board[i][2]):
                winner = self.board[i][0]

        # Vérification des lignes verticales
        for i in range(3):
            if self.equals3(self.board[0][i], self.board[1][i], self.board[2][i]):
                winner = self.board[0][i]

        # Vérification des diagonales
        if self.equals3(self.board[0][0], self.board[1][1], self.board[2][2]):
            winner = self.board[0][0]
        if self.equals3(self.board[2][0], self.board[1][1], self.board[0][2]):
            winner = self.board[2][0]

        # Comptage des cases vides
        open_spots = 0
        for i in range(3):
            for j in range(3):
                if self.board[i][j] == '-':
                    open_spots += 1

        if winner is None and open_spots == 0:
            return 'tie'
        else:
            return winner

    def display_result(self, result):
        ''' Affiche le vainqueur sur la fenêtre pyame.'''
        if result is not None:
            result_text = ''
            if result == 'tie':
                result_text = 'Tie!'
            else:
                result_text = f'Les {result} ont gagné !'
            
            # Affichage du résultat
            font = pygame.font.Font(None, 32)
            text = font.render(result_text, True, self.RED)
            text_rect = text.get_rect(center=(self.WIDTH // 2, self.HEIGHT // 2))
            self.screen.blit(text, text_rect)
            return False
        
        return True
        

    def best_move(self):
        ''' Choisis le meilleur coup à jouer grâce à l'algorithme "minimax" et met à jour self.board.'''
        # AI fait son coup
        best_score = float('-inf')
        move = None
        for i in range(3):
            for j in range(3):
                # La case est-elle disponible ?
                if self.board[i][j] == '-':
                    self.board[i][j] = self.AI
                    score = self.minimax(self.board, 0, False)
                    self.board[i][j] = '-'
                    if score > best_score:
                        best_score = score
                        move = (i, j)
        self.board[move[0]][move[1]] = self.AI
        # print("Jeu AI : ", move)
        return move


    def minimax(self, board, depth, is_maximizing):
        ''' Calcul le meilleur score du plateau donné.'''
        result = self.check_winner()
        if result is not None:
            return self.scores[result]

        if is_maximizing:
            best_score = float('-inf')
            for i in range(3):
                for j in range(3):
                    if board[i][j] == '-':
                        board[i][j] = self.AI
                        score = self.minimax(board, depth + 1, False)
                        board[i][j] = '-'
                        best_score = max(score, best_score)
            return best_score
        else:
            best_score = float('inf')
            for i in range(3):
                for j in range(3):
                    if board[i][j] == '-':
                        board[i][j] = self.HUMAN
                        score = self.minimax(board, depth + 1, True)
                        board[i][j] = '-'
                        best_score = min(score, best_score)
            return best_score

    def next_move(self, i, j):
        '''Met à jour self.board en fonction du coup demandé par le joueur et effectue le coup de l'IA.'''
        if self.board[i][j] == '-':
            # si le coup est valide
            self.board[i][j] = self.HUMAN
            for i in range(3):
                if '-' in self.board[i]:
                    self.current_player = self.AI
                    move = self.best_move()
                    self.current_player = self.HUMAN
                    return True, move
        return False, 0

    def update_board(self):
        ''' Met à jour la fenêtre pygame en fonction de self.board.'''
        for i in range(3):
            for j in range(3):
                y = self.W * i + self.W // 2
                x = self.H * j + self.H // 2
                spot = self.board[i][j]
                radius = self.W // 4
                if spot == self.HUMAN:
                    pygame.draw.circle(self.screen, self.BLACK, (x, y), radius, 4)
                elif spot == self.AI:
                    pygame.draw.line(self.screen, self.BLACK, (x - radius, y - radius), (x + radius, y + radius), 4)
                    pygame.draw.line(self.screen, self.BLACK, (x + radius, y - radius), (x - radius, y + radius), 4)

    def compare_board(self, vision_board):
        '''
        Renvoie : bool, bool, (i,j) \n
        \t  - Il y a une difference entre le plateau que je vois et celui que je connais,
        \t  - Une pièce 'O' (True) ou 'X' (False) a été jouée,
        \t  - La case jouée (i, j).
        '''
        for i in range(3):
            for j in range(3):
                if self.board[i][j] != vision_board[i][j]:
                    if self.board[i][j] == '-' and vision_board[i][j] == 'O':
                        # self.board[i][j] = vision_board[i][j]
                        # print("[COMPARE_BOARD] Il y a une pièce O en plus.")
                        # print("[COMPARE_BOARD] Je vois ça : ", vision_board)
                        # print("[COMPARE_BOARD] Et j'ai ça : ", self.board)
                        return True, True, (i, j)
                    elif self.board[i][j] == 'X' and vision_board[i][j] == '-':
                        # print("[COMPARE_BOARD] J'ai une pièce X en plus.")
                        # print("[COMPARE_BOARD] Je vois ça : ", vision_board)
                        # print("[COMPARE_BOARD] Et j'ai ça : ", self.board)
                        return True, False, (i, j)
        return False, False, (i, j)




if __name__=="__main__":

    board1 = [
            ['X', '-', '-'],
            ['-', '-', '-'],
            ['-', '-', '-']
        ]
    
    board2 = [
            ['-', 'O', '-'],
            ['-', '-', '-'],
            ['-', '-', '-']
        ]

"""
    # Initialisation de pygame
    pygame.init()

    game = Tictactoe('IA')

    # Boucle principale du jeu
    running = True
    while running:

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
                break
            elif event.type == pygame.MOUSEBUTTONDOWN and game.current_player == game.HUMAN:
                mouse_x, mouse_y = pygame.mouse.get_pos()
                # Calcul de l'indice de la case
                i = mouse_x // game.W
                j = mouse_y // game.H
                if game.next_move(i, j):
                    break

        game.update_board()

        # Vérification du résultat
        result = game.check_winner()
        running = game.display_result(result)

        # Mise à jour de l'affichage
        pygame.display.flip()

        if not running:
            time.sleep(1.5)

    # Fermeture de pygame
    pygame.quit()
"""