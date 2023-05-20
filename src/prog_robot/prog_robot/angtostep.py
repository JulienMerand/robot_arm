def ang2step(PoseJ, LastPoseJ):

    MicroStep = 8
    DegPerStep = 1.8/MicroStep
    GearRatio = [10.25,5.5,21.75,1,4.75,1]

    Ang = [i - j for i, j in zip(PoseJ,LastPoseJ)]

    step = [int(1*Ang[0]*GearRatio[0]/DegPerStep), int(-1*Ang[1]*GearRatio[1]/DegPerStep), int(1*Ang[1]*GearRatio[1]/DegPerStep), int(-1*Ang[2]*GearRatio[2]/DegPerStep), int(1*Ang[3]*GearRatio[3]/DegPerStep), int(1*Ang[4]*GearRatio[4]/DegPerStep), int(-1*Ang[5]*GearRatio[5]/DegPerStep)]


    print(step)

LastPoseJ = [0,0,0,0,0,0]
PoseJ = [30,59,80,-90,-90,180]

ang2step(PoseJ, LastPoseJ)