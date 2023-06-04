import math

def deg2Rad(L):
        res = []
        for x in L:
            res.append(x*180/math.pi)
        return res

def ang2Step(PoseJ, unitee):
    ''' Unitée : rad/deg'''

    MicroStep = 8
    DegPerStep = 1.8/MicroStep
    GearRatio = [10.25,5.45,22.75,1,4.5,1]
    if unitee == "rad":
        Ang = deg2Rad(PoseJ)
    else:
        Ang = PoseJ

    step = [int(-1*Ang[0]*GearRatio[0]/DegPerStep), int(1*Ang[1]*GearRatio[1]/DegPerStep), int(-1*Ang[1]*GearRatio[1]/DegPerStep), int(1*Ang[2]*GearRatio[2]/DegPerStep), int(-1*Ang[3]*GearRatio[3]/DegPerStep), int(-1*Ang[4]*GearRatio[4]/DegPerStep), int(-1*Ang[5]*GearRatio[5]/DegPerStep)]

    return step

def centre_angles_rad_autour_de_zero(angles):
    for i in range(len(angles)):
        while(angles[i] > math.pi):
            angles[i] -= 2*math.pi
        while(angles[i] < -math.pi):
             angles[i] += 2*math.pi
    return angles

def remove_duplicates(L):
    new_L = []
    for elt in L:
         if not any(l == list(elt) for l in new_L):
              new_L.append(list(elt))
    return new_L


# L = [[-9.000e+01, -2.000e-03, -9.000e+01],[-9.000e+01, -2.000e-03, -9.000e+01],[9.000e+01, -2.000e-03, 9.000e+01],[9.000e+01, -2.000e-03, 9.000e+01],[9.000e+01, 9.112e+01, -9.000e+01]]
# print(remove_duplicates(L))