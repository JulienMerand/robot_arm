import cv2
import pickle
import numpy as np
from math import sqrt, sin, cos, atan, pi

def print_board(board):
    print("\nGame :\n")
    print(f"\n\t{board[0][0]} | {board[0][1]} | {board[0][2]}")
    print("\t---------")
    print(f"\t{board[1][0]} | {board[1][1]} | {board[1][2]}")
    print("\t---------")
    print(f"\t{board[2][0]} | {board[2][1]} | {board[2][2]}\n")

class Vision_Tictactoe():

    def __init__(self):
        
        # Get camera parameters
        file_path = '/home/julien/Documents/Python/calibration/parametres_camera.pkl'
        with open(file_path, 'rb') as f:
            parameters = pickle.load(f)
        
        self.extrinsic_matrix = parameters['extrinsic matrix']
        self.intrinsic_matrix = parameters['intrinsic matrix']
        self.dist_coeffs = parameters['distortion']
        self.rot_vecs = parameters['rotation']
        self.trans_vecs = parameters['translation']

    def pixel_to_real_coordinates(self, px, py):
        # rx, ry = 0.518*py+64.203, (px - 400)/2
        rx, ry = 0.512*py + 66.744, 0.481*px - 189.26
        # rx, ry = py, px
        return rx, ry

    def analyse_board(self, board_img):

        board = [
            ['-', '-', '-'],
            ['-', '-', '-'],
            ['-', '-', '-']
        ]

        def find_object(mask, img, color):
            places = []
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if(len(contours) > 0):
                for cnt in contours:
                    x,y,w,h = cv2.boundingRect(cnt)
                    if w>30 and h>30:
                        rect = cv2.minAreaRect(cnt)
                        box = np.intp(cv2.boxPoints(rect))
                        middle_x, middle_y = box[0][0] + (box[2][0] - box[0][0])//2, box[0][1] + (box[2][1] - box[0][1])//2

                        peri = cv2.arcLength(cnt, True)
                        approx_poly = cv2.approxPolyDP(cnt, 0.04 * peri, True)
                        approx_poly = np.intp(approx_poly)
                        pts = np.zeros((4,2))
                        for i in range(4):
                            pts[i] = approx_poly[i][0]
                        pts = np.intp(pts)

                        cv2.drawContours(img, [box], 0, color, 2)
                        cv2.circle(img, (middle_x, middle_y), 5, color, -1)

                        i,j = middle_x//W, middle_y//H
                        places.append((j,i))

            return places

        def update_board(places, player):
            for (i,j) in places:
                board[i][j] = player

        img = np.copy(board_img)

        loHSV_RED_1 = np.array([0,110,0])
        hiHSV_RED_1 = np.array([30,255,255])
        loHSV_RED_2 = np.array([90,110,0])
        hiHSV_RED_2 = np.array([255,255,255])

        loHSV_GREEN = np.array([55,65,100])
        hiHSV_GREEN = np.array([70,255,255])

        img_height, img_width = img.shape[:2]

        H, W = img_height//3, img_width//3

        img_HSV = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        img_blur = cv2.blur(img_HSV, (7,7))
        mask_red_1 = cv2.inRange(img_blur, loHSV_RED_1, hiHSV_RED_1)
        mask_red_2 = cv2.inRange(img_blur, loHSV_RED_2, hiHSV_RED_2)
        mask_red = cv2.bitwise_or(mask_red_1, mask_red_2)
        mask_red = cv2.dilate(mask_red, np.ones((5, 5), np.uint8), iterations=1)
        mask_green = cv2.inRange(img_blur, loHSV_GREEN, hiHSV_GREEN)

        place_red = find_object(mask_red, img, (0,0,255))
        place_green = find_object(mask_green, img, (0,255,0))

        update_board(place_red, 'O')
        update_board(place_green, 'X')

        return img, board

    def find_board(self, plate):
        
        img = np.copy(plate)
        img_HSV = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        img_blur = cv2.blur(img_HSV, (7,7))
        mask = cv2.inRange(img_blur, np.array([0,0,190]), np.array([255,255,255]))
        kernel = np.ones((5, 5), np.uint8)
        mask_morph = cv2.dilate(mask, kernel, iterations=1)
        mask_morph = cv2.morphologyEx(mask_morph, cv2.MORPH_CLOSE, kernel, iterations=2)
        contours, _ = cv2.findContours(mask_morph, cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
        if len(contours) > 0:
            cnt = max(contours, key=cv2.contourArea)
            rect = cv2.minAreaRect(cnt)
            box = np.int0(cv2.boxPoints(rect)) # [[89,152], [307,152], [307,358], [89,358]]  coin haut à gauche puis sens horaire
            peri = cv2.arcLength(cnt, True)
            approx_poly = cv2.approxPolyDP(cnt, 0.04 * peri, True)
            approx_poly = np.intp(approx_poly)
            pts = np.zeros((4,2))
            for i in range(4):
                pts[i] = approx_poly[i][0]
            pts1 = np.intp(pts)

            pts2 = np.float32([[0, 0],     [600, 0],
                            [600, 600], [0, 600]])
            
            # Apply Perspective Transform Algorithm
            matrix = cv2.getPerspectiveTransform(np.float32(pts1), pts2)
            result = cv2.warpPerspective(img, matrix, (600, 600))

            x_middle_board, y_middle_board = box[0][0] + (box[2][0] - box[0][0])//2, box[0][1] + (box[2][1] - box[0][1])//2
            img = cv2.drawContours(img,[pts1],0,(0,255,0),2)
            cv2.circle(img, (x_middle_board, y_middle_board), 5, (0,0,0), -1)
            try:
                theta = -(atan((box[1][0] - box[0][0])/(box[1][1] - box[0][1]))%(pi/4))
            except:
                theta = 0.0
            TR_BaseToBoard = np.eye(2)
            Rot_board = np.array(([cos(theta), -sin(theta)],
                                [sin(theta),  cos(theta)]))
            coord_x, coord_y = self.pixel_to_real_coordinates(x_middle_board, y_middle_board)
            Pos_board = np.array([int(coord_x), int(coord_y)])
            TR_BaseToBoard = [Rot_board, Pos_board]       

        return cv2.flip(result, 1), TR_BaseToBoard, round(theta*180/pi,1)

    def show_squares(self, board_img, draw):
        
        squares_coords_board = {0: [], 1: [], 2: [], 3: [], 4: [], 5: [], 6: [], 7: [], 8: []}
        img = np.copy(board_img)

        def get_contour_top_left(contour):
            x,y,w,h = cv2.boundingRect(contour)
            return (x+w//2)//200, (y+h//2)//200

        grey = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        _, thresh = cv2.threshold(grey, 200, 255, cv2.THRESH_BINARY)
        kernel = np.ones((5, 5), np.uint8)
        img_close = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel, iterations=1)
        contours, _ = cv2.findContours(img_close, cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
        contours = sorted(contours, key=lambda x: cv2.contourArea(x))[::-1][:10] # On garde que les 9 plus grands contours
        if len(contours) > 0:
            contours = sorted(contours, key=get_contour_top_left)
            i = 0
            for cnt in contours:
                x, y, w, h = cv2.boundingRect(cnt)
                if (w > 150 and h > 150) and (abs(w-h)<50):
                    cv2.rectangle(draw, (x,y), (x+w,y+h), (255,0,0), 2)
                    x_centre, y_centre = x + w//2, y + h//2
                    cv2.putText(draw, str(i), (x_centre-15, y_centre+15), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (255,0,0), 4)
                    coord_x_board, coord_y_board = (y_centre - 300)*210/600, (x_centre - 300)*210/600
                    squares_coords_board[i] = [coord_x_board, coord_y_board]
                    i += 1
        return draw, squares_coords_board

    def get_board(self, frame):
        # frame = cv2.undistort(frame,self.intrinsic_matrix, self.dist_coeffs)
        # Apply Perspective Transform Algorithm
        pts1 = np.float32([[58, 90],     [429, 90],
                            [34, 512], [452, 512]])
        pts2 = np.float32([[0, 0],     [800, 0],
                            [0, 900], [800, 900]])
        matrix = cv2.getPerspectiveTransform(pts1, pts2)
        wood_plate = cv2.warpPerspective(frame, matrix, (800,900))
        # Find the board and get the transformation matrix from the base to the board and the angle of the board
        board_img, TR_base2board, theta = self.find_board(wood_plate)
        # Show, identify squares and find the pieces to construct the board
        board_analysed, board = self.analyse_board(board_img)
        full_board_img, squares_coords_board = self.show_squares(board_img, board_analysed)
        squares_coords = {0: [], 1: [], 2: [], 3: [], 4: [], 5: [], 6: [], 7: [], 8: []}
        for i in range(9):
            rot, trans = TR_base2board[0], TR_base2board[1]
            squares_coords[i] = list(np.round(np.dot(np.linalg.inv(rot), np.transpose(squares_coords_board[i] + trans)),2)) + [theta]       
        return board, squares_coords, full_board_img


    def find_green_pieces(self, plate):
        coords = []
        img = np.copy(plate)
        img_HSV = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        img_blur = cv2.blur(img_HSV, (7,7))
        loHSV_GREEN = np.array([55,65,100])
        hiHSV_GREEN = np.array([70,255,255])
        mask = cv2.inRange(img_blur, loHSV_GREEN, hiHSV_GREEN)
        kernel = np.ones((5, 5), np.uint8)
        mask_morph = cv2.dilate(mask, kernel, iterations=1)
        mask_morph = cv2.morphologyEx(mask_morph, cv2.MORPH_OPEN, kernel, iterations=1)
        contours, _ = cv2.findContours(mask_morph, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if(len(contours) > 0):
            for cnt in contours:
                x,y,w,h = cv2.boundingRect(cnt)
                if w>30 and h>30:
                    rect = cv2.minAreaRect(cnt)
                    box = np.intp(cv2.boxPoints(rect))
                    middle_x, middle_y = box[0][0] + (box[2][0] - box[0][0])//2, box[0][1] + (box[2][1] - box[0][1])//2
                    cv2.drawContours(img, [box], 0, (0,255,0), 2)
                    cv2.circle(img, (middle_x, middle_y), 5, (0,255,0), -1)
                    coord_x, coord_y = self.pixel_to_real_coordinates(middle_x, middle_y)
                    try:
                        theta = -(atan((box[1][0] - box[0][0])/(box[1][1] - box[0][1]))%(pi/4))*180/pi
                    except Exception as e:
                        theta = 0.0
                    coords.append([round(coord_x,2), round(coord_y,2), round(theta,2)])
        # cv2.imshow('Green Pieces', img)
        # cv2.imshow('mask', mask_morph)
        # cv2.waitKey(0)
        return coords
        
    def get_coords_green_pieces(self, frame):
        # frame = cv2.undistort(frame,self.intrinsic_matrix, self.dist_coeffs)
        img = np.copy(frame)
        # cv2.imshow('img',img)
        cv2.waitKey(0)
        img_HSV = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        img_blur = cv2.blur(img_HSV, (7,7))
        mask = cv2.inRange(img_blur, np.array([0,0,190]), np.array([255,30,255]))
        kernel = np.ones((5, 5), np.uint8)
        mask_morph = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=3)
        mask_morph = cv2.dilate(mask_morph, kernel, iterations=2)

        contours, _ = cv2.findContours(mask_morph, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if(len(contours) > 0):
            for cnt in contours:
                x,y,w,h = cv2.boundingRect(cnt)
                if w>150 and h>150:
                    rect = cv2.minAreaRect(cnt)
                    box = np.intp(cv2.boxPoints(rect))
                    cv2.drawContours(mask_morph, [box], 0, 255, -1)

        mask_morph_not = cv2.bitwise_not(mask_morph)
        new_frame = cv2.bitwise_and(frame, frame, mask=mask_morph_not)
        # Apply Perspective Transform Algorithm
        pts1 = np.float32([[58, 90],     [429, 90],
                            [34, 512], [452, 512]])
        pts2 = np.float32([[0, 0],     [800, 0],
                            [0, 900], [800, 900]])
        matrix = cv2.getPerspectiveTransform(pts1, pts2)
        wood_plate = cv2.warpPerspective(new_frame, matrix, (800,900))
        # Find the green pieces
        # cv2.imshow('plateau', wood_plate)
        green_pieces_coords = self.find_green_pieces(wood_plate)
        return green_pieces_coords


if __name__=="__main__":

    vision = Vision_Tictactoe()

    cap = cv2.VideoCapture('https://192.168.1.16:8080/video')
    ret, frame = cap.read()
    if ret == True:

        green_pieces_coords = vision.get_coords_green_pieces(frame)
        print("\nGreen pieces coords : ",green_pieces_coords)
        cv2.waitKey(0)

        # board, squares_coords, full_board_img = vision.get_board(frame)
        # cv2.imshow('Board', full_board_img)
        # print("\nSquares coords : ", squares_coords)
        # print_board(board)
        # cv2.waitKey(0)


