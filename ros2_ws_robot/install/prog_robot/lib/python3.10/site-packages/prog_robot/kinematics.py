import numpy as np
from math import sin, cos, pi, atan, acos, asin, sqrt, atan2
from scipy.spatial.transform import Rotation as R
from prog_robot.fonctions import centre_angles_rad_autour_de_zero, remove_duplicates
import time
import matplotlib.pyplot as plt

def Transformation_Matrix(q, d, a, alpha):
    """Calcule la matrice de transformation via la méthode de Denavit-Hartenberg"""
    c = cos(q)
    s = sin(q)
    ca = cos(alpha)
    sa = sin(alpha)

    T = np.array([[c,-s*ca,s*sa,a*c],
                  [s,c*ca,-c*sa,a*s],
                  [0,sa,ca,d],
                  [0,0,0,1]])

    return T

def forward_kinematics(theta):
    """
    [q1,q2,q3,q4,q5,q6] (deg) --> pos [x,y,z] (mm), ori [z,y,x] (deg)
    -----------------------------------------------------------------
    Calcule la cinématique directe grâce au modèle géométrique.
    Renvoie la position et l'orientation du TCP en fonction des coordonnées articulaires de chaques axes
    """

    # Paramètres en mm
    a1, a2, a3, a4 = 226.5, 221.124, 223.5, 182.02

    # Conversion des angles en radians
    q = np.deg2rad(theta)

    # Matrices de transformations (q, d, a, alpha)
    T01 = Transformation_Matrix(q[0],  a1, 0,  -pi/2)
    T12 = Transformation_Matrix(q[1]-pi/2,  0,  a2, 0)
    T23 = Transformation_Matrix(q[2]+pi/2,  0,  0, pi/2)
    T34 = Transformation_Matrix(q[3],  a3, 0,  -pi/2)
    T45 = Transformation_Matrix(q[4],  0,  0,  pi/2)
    T56 = Transformation_Matrix(q[5],  a4, 0,  0)

    T02 = np.matmul(T01,T12)
    T03 = np.matmul(T02,T23)
    T04 = np.matmul(T03,T34)
    T05 = np.matmul(T04,T45)
    T06 = np.matmul(T05,T56)

    # On arrondi les éléments de la matrice au millième pour raison de lisibilité   
    T = np.around(T06, 3)

    # Affichage des différentes matrices de transformation 
    # print(f"T01 = {T01}\n")
    # print(f"T02 = {T02}\n")
    # print(f"T03 = {T03}\n")
    # print(f"T04 = {T04}\n")
    # print(f"T05 = {T05}\n")
    # print(f"T06 = {T}")

    # On récupère le vecteur position [x,y,z] et le vecteur orientation [A,B,C] ([z,y,x])
    pos = T[:3,3]
    ori = (R.from_matrix(T[0:3,0:3])).as_euler('zyx', degrees=True)

    return pos, ori

def forward_kinematics_3dof(q):
    ''' Cinématique direct en fonction de [theta1, theta2, theta3]'''   

    # Paramètres en mm
    a1, a2, a3 = 226.5, 221.124, 223.5 

    # Matrices de transformations (q, d, a, alpha)
    T01 = Transformation_Matrix(q[0],  a1, 0,  -pi/2)
    T12 = Transformation_Matrix(q[1]-pi/2,  0,  a2, 0)
    T23 = Transformation_Matrix(q[2]+pi/2,  0,  0, pi/2)
    T34 = Transformation_Matrix(0,  a3, 0,  -pi/2)

    T02 = np.matmul(T01,T12)
    T03 = np.matmul(T02,T23)
    T04 = np.matmul(T03,T34)

    # Affichage des différentes matrices de transformation 
    # print(f"T01 = {T01}\n")
    # print(f"T02 = {T02}\n")
    # print(f"T03 = {T03}\n")
    # print(f"T04 = {T04}\n")

    # On récupère le vecteur position [x,y,z] et le vecteur orientation [A,B,C] ([z,y,x])
    pos = T04[:3,3]
    # ori = (R.from_matrix(T[0:3,0:3])).as_euler('zyx', degrees=True)

    return pos

def R03(q):   

    # Paramètres en mm
    a1, a2, a3 = 226.5, 221.124, 223.5 

    # Matrices de transformations (q, d, a, alpha)
    T01 = Transformation_Matrix(q[0],  a1, 0,  -pi/2)
    T12 = Transformation_Matrix(q[1]-pi/2,  0,  a2, 0)
    T23 = Transformation_Matrix(q[2]+pi/2,  0,  0, pi/2)
    T34 = Transformation_Matrix(0,  a3, 0,  -pi/2)

    T02 = np.matmul(T01,T12)
    T03 = np.matmul(T02,T23)

    # Affichage des différentes matrices de transformation 
    # print(f"T01 = {T01}\n")
    # print(f"T02 = {T02}\n")
    # print(f"T03 = {T03}\n")
    # print(f"T04 = {T04}\n")

    R03 = T03[0:3,0:3]

    return R03

def workspace():
    '''Calcul et affiche le workspace'''
    # Paramètres en mm / rad
    a1, a2, a3 = 226.5, 221.124, 223.5
    lim_theta1, lim_theta2, lim_theta3 = [-1.95,1.95], [-1.5,1.92],[-2.10,2.10]

    theta2_s = np.linspace(lim_theta2[0], lim_theta2[1], 50)
    theta3_s = np.linspace(lim_theta3[0], lim_theta3[1], 50)
    X, Z = [], []
    for theta2 in theta2_s:
        for theta3 in theta3_s:
            X.append(a2*sin(theta2) + a3*sin(theta2 + theta3))
            Z.append(a2*cos(theta2) + a3*cos(theta2 + theta3) + a1)
    
    plt.scatter(X,Z, marker='o')
    plt.axis()
    plt.show()

def trois_premier_angles(pos_poignet, ori, elbow_config):
    '''
    Renvoie la liste de toutes les positions angulaires possibles en fonction de la position [x,y,z] du poignet et de l'orientation du gripper
    '''
    # Paramètres en mm / rad
    a1, a2, a3 = 226.5, 221.124, 223.5
    lim_theta1, lim_theta2, lim_theta3 = [-1.95,1.95], [-1.5,1.92],[-2.10,2.10] 
    
    # print("pos poignet : ",pos_poignet)

    x, y, z = pos_poignet[0], pos_poignet[1], pos_poignet[2]-a1
    
    r = sqrt(x**2 + y**2 + z**2)
    calc = round((a2**2 + a3**2 - r**2) / (2*a2*a3), 3)
    z_r = z/r
    
    if (r > a2 + a3) or (calc < -1 or calc > 1) or (z_r < -1 or z_r > 1):
        # print(f"Hors du workspace : \nr =  {r} : {r > a2 + a3} \ncalc = {calc} : {calc > -1 or calc < 1} \nz/r = {z_r} : {z_r > -1 or z_r < 1}")
        print("Hors du workspace")
        return []
    # print("r : ", r)
    else :
        # Liste des theta1 possibles
        if(x==0 and y==0):
            theta1_s = [ori[0]]
        elif(x==0 and y!=0):
            theta1_s = [-pi/2,pi/2]
        else:
            theta1_s = [atan(y/x), atan(y/x)+pi]
            
        # Liste des theta3 possibles
        try:
            theta3_s = [pi-acos(calc), pi+acos(calc)]
        except:
            theta3_s = []
        
        # Liste des theta2 possibles
        alpha = [pi - asin(z_r), asin(z_r)]
        calc_beta = [a3*sin(theta3_s[0])/(a2+a3*cos(theta3_s[0])),a3*sin(theta3_s[1])/(a2+a3*cos(theta3_s[1]))]
        beta = [atan(calc_beta[0]), atan(calc_beta[1]), pi + atan(calc_beta[0]), pi + atan(calc_beta[1])]
        theta2_s = []
        for a in alpha:
            for b in beta:
                theta2_s.append(pi/2 - (a + b))

        # print("theta1_s = ",theta1_s)
        # print("theta2_s = ",theta2_s)
        # print("theta3_s = ",theta3_s)
        # print("\n")

        # On centre tout entre -pi et pi
        theta1_s = centre_angles_rad_autour_de_zero(theta1_s)
        theta2_s = centre_angles_rad_autour_de_zero(theta2_s)
        theta3_s = centre_angles_rad_autour_de_zero(theta3_s)

        # On check si ça dépasse pas les limites
        i=0
        while(i != len(theta1_s)):
            if theta1_s[i] >= lim_theta1[0] and theta1_s[i] <= lim_theta1[1]:
                i += 1
            else:
                del(theta1_s[i])
        i=0
        while(i != len(theta2_s)):
            if theta2_s[i] >= lim_theta2[0] and theta2_s[i] <= lim_theta2[1]:
                i += 1
            else:
                del(theta2_s[i])
        i=0
        while(i != len(theta3_s)):
            if theta3_s[i] >= lim_theta3[0] and theta3_s[i] <= lim_theta3[1]:
                i += 1
            else:
                del(theta3_s[i])


        # print("theta1_s = ",theta1_s)
        # print("theta2_s = ",theta2_s)
        # print("theta3_s = ",theta3_s)
        # print("\n")

        # On garde les solutions possibles
        sol_s = []
        for theta1 in theta1_s:
            for theta2 in theta2_s:
                for theta3 in theta3_s:
                    [x_calc, y_calc, z_calc] = forward_kinematics_3dof([theta1, theta2, theta3])
                    # print(f"x_calc : {x_calc}, y_calc : {y_calc}, z_calc : {z_calc}")
                    x_condition = (x_calc>x-0.5 and x_calc<x+0.5)
                    y_condition = (y_calc>y-0.5 and y_calc<y+0.5)
                    z_condition = (z_calc>(z+a1)-0.5 and z_calc<(z+a1)+0.5)
                    # print(f"condition : {x_condition}, {y_condition}, {z_condition}")
                    if(x_condition and y_condition and z_condition):
                        sol_s.append([theta1,theta2,theta3])

        sol_s = np.round(sol_s,3).tolist()
        # On enlève les doublons
        sol_s = remove_duplicates(sol_s)

        if len(sol_s)<=1:
            return sol_s
        else:
            res=[]
            for sol in sol_s:
                if elbow_config=="above" and sol[2]>0:
                    res.append(sol)
                elif elbow_config=="below" and sol[2]<0:
                    res.append(sol)
            return res

def inverse_kinematics(pos, ori, elbow_config="above", wrist_turned=False):
    '''
    Position : [x,y,z], Orientation [A(z),B(y),C(x)]
    elbow_config : "above" or "below", default = "above"
    wrist_turned : True or False, default = False
    '''

    if pos[2] <= 0:
        return []
    else:
        # Parametres en mm
        a4 = 182.02
        # Limites thetas 4, 5, 6
        lim_theta4, lim_theta5, lim_theta6 = [-pi,pi], [-1.7,1.7],[-pi,pi] 

        R06 = np.zeros((3,3))
        Rx = np.array([[1,         0         ,          0          ],
                    [0, cos(ori[2]*pi/180), -sin(ori[2]*pi/180) ],
                    [0, sin(ori[2]*pi/180),  cos(ori[2]*pi/180) ]])
        
        Ry = np.array([[ cos(ori[1]*pi/180), 0, sin(ori[1]*pi/180) ],
                    [         0         , 1,          0         ],
                    [-sin(ori[1]*pi/180), 0, cos(ori[1]*pi/180) ]])
        
        Rz = np.array([[cos(ori[0]*pi/180), -sin(ori[0]*pi/180), 0 ],
                    [sin(ori[0]*pi/180),  cos(ori[0]*pi/180), 0 ],
                    [        0         ,          0         , 1 ]])
        
        R06[0:3,0:3] = np.matmul(np.matmul(Rz,Ry),Rx)
        
        wrist_center = (np.transpose(pos) - a4*np.matmul(R06,np.transpose(np.array([0,0,1])))).tolist()
        print("wrist center : ", wrist_center)

        liste_trois_premier_angles = trois_premier_angles(wrist_center, ori, elbow_config)
        print(liste_trois_premier_angles)
        if liste_trois_premier_angles == []:
            return []
        
        res = []
        for angle_1_2_3 in liste_trois_premier_angles:
            R36 = np.matmul(np.transpose(R03(angle_1_2_3)),R06)
            theta5 = acos(R36[3-1,3-1])

            if theta5 != 0 and theta5 <= lim_theta5[1]:
                theta5 = atan2(sqrt(1-R36[3-1,3-1]**2), R36[3-1,3-1]) 
                theta4 = atan2(R36[2-1,3-1], R36[1-1,3-1])
                theta6 = atan2(R36[3-1,2-1], -R36[3-1,1-1])

                # On check les limites avant d'ajouter la solution
                if (theta4 > lim_theta4[0] and theta4 < lim_theta4[1]) and (theta6 > lim_theta6[0] and theta6 < lim_theta6[1]):
                    res.append([angle_1_2_3[0], angle_1_2_3[1], angle_1_2_3[2], theta4, theta5, theta6])

                theta5 = atan2(-sqrt(1-R36[3-1,3-1]**2), R36[3-1,3-1])
                theta4 = atan2(-R36[2-1,3-1], -R36[1-1,3-1])
                theta6 = atan2(-R36[3-1,2-1], R36[3-1,1-1])
                
                # On check les limites avant d'ajouter la solution
                if (theta4 > lim_theta4[0] and theta4 < lim_theta4[1]) and (theta6 > lim_theta6[0] and theta6 < lim_theta6[1]):
                    res.append([angle_1_2_3[0], angle_1_2_3[1], angle_1_2_3[2], theta4, theta5, theta6])
            
            elif theta5  == 0:
                # On sait que R36[1,1] = cos(theta4 + theta6)
                # On fixe theta4 = 0 
                theta4 = 0
                theta6 = atan2(sqrt(1-R36[1-1,1-1]**2), R36[1-1,1-1])
                # On check les limites avant d'ajouter la solution
                if (theta4 > lim_theta4[0] and theta4 < lim_theta4[1]) and (theta6 > lim_theta6[0] and theta6 < lim_theta6[1]):
                    res.append([angle_1_2_3[0], angle_1_2_3[1], angle_1_2_3[2], theta4, theta5, theta6])
                
                theta6 = atan2(-sqrt(1-R36[1-1,1-1]**2), R36[1-1,1-1])
                # On check les limites avant d'ajouter la solution
                if (theta4 > lim_theta4[0] and theta4 < lim_theta4[1]) and (theta6 > lim_theta6[0] and theta6 < lim_theta6[1]):
                    res.append([angle_1_2_3[0], angle_1_2_3[1], angle_1_2_3[2], theta4, theta5, theta6])


        # Conversion des angles en degree
        sol_s = np.round(np.rad2deg(res),3).tolist()
        # print(sol_s)

        # On enlève les doublons
        sol_s = remove_duplicates(sol_s)

        if sol_s==[]:
            return []
        elif len(sol_s)==1:
            return sol_s[0]
        else:
            res = []
            for sol in sol_s:
                if not wrist_turned and (abs(sol[3])<90 or abs(sol[5])<90):
                    res.append(sol)
                elif wrist_turned and (abs(sol[3]>90) or abs(sol[5])>90):
                    res.append(sol)
            if len(res)==1:
                return res[0]
            else:
                print("Problème, toujours trop de solutions !")
                print(res)
                return []

if __name__=="__main__":
    # ang = [0,45,45,0,45,0]
    # ang = [18,100,-64,70,76,57]
    # ang = [37,65,75,0,40,0]
    # pos, ori = forward_kinematics(ang)
    # print("\nResultat : ")
    # print(f"pos : {pos} | ori : {ori}\n")

    # ang = [0,0,pi/2]
    # pos = forward_kinematics_3dof(ang)
    # print("\nResultat : ")
    # print(f"pos : {pos}")

    # t = time.time()
    # pos_poignet = [221.124, 0, 7]
    # thetas = trois_premier_angles(pos_poignet)
    # t = time.time() - t
    # print(t)
    # print(thetas)

    t = time.time()
    # print(inverse_kinematics(pos, ori))
    print(inverse_kinematics([225,0,350], [0,120,0], "above", False))
    t = time.time() - t
    print("Temps d'exécution (s) : ",t)

    # workspace()