import math
import numpy as np

def deg2Rad(L):
    '''Convertit une liste de degrés en radians'''
    res = []
    for x in L:
        res.append(x*180/math.pi)
    return res

def ang2Step(PoseJ, unitee):
    ''' Renvoie la liste du nombre de pas à faire pour les 7 moteurs | unitee = rad/deg'''
    MicroStep = 8
    DegPerStep = 1.8/MicroStep
    GearRatio = [10.25,5.45,22,1,4.4,1]
    if unitee == "rad":
        Ang = deg2Rad(PoseJ)
    else:
        Ang = PoseJ

    step = [int(-1*Ang[0]*GearRatio[0]/DegPerStep), int(1*Ang[1]*GearRatio[1]/DegPerStep), int(-1*Ang[1]*GearRatio[1]/DegPerStep), int(1*Ang[2]*GearRatio[2]/1.8), int(-1*Ang[3]*GearRatio[3]/DegPerStep), int(-1*Ang[4]*GearRatio[4]/DegPerStep), int(-1*Ang[5]*GearRatio[5]/DegPerStep)]

    return step

def centre_angles_rad_autour_de_zero(angles):
    ''' Centre les angles entre -pi et pi'''
    for i in range(len(angles)):
        while(angles[i] > math.pi):
            angles[i] -= 2*math.pi
        while(angles[i] < -math.pi):
             angles[i] += 2*math.pi
    return angles

def remove_duplicates(L):
    '''Supprime les listes jumelles dans une liste'''
    new_L = []
    for elt in L:
         if not any(l == list(elt) for l in new_L):
              new_L.append(list(elt))
    return new_L

def cercle(centre, rayon):
    '''renvoie deux tableau X, Y des points du cercle, commence en [Ox + r, Oy]'''
    Ox, Oy = centre[0], centre[1]
    # X = [-k for k in range(1,rayon)] + [k for k in range(rayon)]
    theta = np.linspace(0, 2*np.pi, 100)

    x = (rayon*np.cos(theta)).tolist()
    y = (rayon*np.sin(theta)).tolist()

    X, Y = [], []
    for elt in x:
        X.append(elt + Ox)
    for elt in y:
        Y.append(elt + Oy)

    return X,Y

if __name__=="__main__":

# L = [[-9.000e+01, -2.000e-03, -9.000e+01],[-9.000e+01, -2.000e-03, -9.000e+01],[9.000e+01, -2.000e-03, 9.000e+01],[9.000e+01, -2.000e-03, 9.000e+01],[9.000e+01, 9.112e+01, -9.000e+01]]
# print(remove_duplicates(L))
    
    print(cercle([300,0], 100))