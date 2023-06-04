import serial
import time

def ang2step(PoseJ, LastPoseJ):

    MicroStep = 8
    DegPerStep = 1.8/MicroStep
    GearRatio = [10.25,5.5,21.75,1,4.75,1]

    Ang = [i - j for i, j in zip(PoseJ,LastPoseJ)]

    step = [int(-1*Ang[0]*GearRatio[0]/DegPerStep), int(1*Ang[1]*GearRatio[1]/DegPerStep), int(-1*Ang[1]*GearRatio[1]/DegPerStep), int(1*Ang[2]*GearRatio[2]/DegPerStep), int(-1*Ang[3]*GearRatio[3]/DegPerStep), int(-1*Ang[4]*GearRatio[4]/DegPerStep), int(-1*Ang[5]*GearRatio[5]/DegPerStep)]

    return step

def send(data):
    # Configurer le port série
    port = '/dev/ttyACM0'  # Spécifiez votre port série
    baudrate = 115200  # Spécifiez le débit en bauds

    # Créer une instance de l'objet Serial
    ser = serial.Serial(port, baudrate)

    # Vérifier si le port série est ouvert
    if ser.is_open:
        print(f'\nPort série {port} ouvert à {baudrate}...')
        try:
            ser.write(data.encode())
            print('Data envoyées : "' + data + '"')

            # Fermer le port série après l'envoie des data
            ser.close()
        except KeyboardInterrupt:
            # Fermer le port série lorsque l'utilisateur appuie sur Ctrl+C
            ser.close()
            print('Port série fermé.')
    else:
        print(f'Échec de l\'ouverture du port série {port}.')

def receive():
    # Configurer le port série
    port = '/dev/ttyACM0'  # Spécifiez votre port série
    baudrate = 115200  # Spécifiez le débit en bauds

    # Créer une instance de l'objet Serial
    ser = serial.Serial(port, baudrate)

    # Vérifier si le port série est ouvert
    if ser.is_open:
        try:
            print("En attente d'une réponse...")
            while True:
                # Lire une ligne de données depuis le port série
                line = ser.readline().decode().strip()
                # Vérifier si des données ont été reçues
                if line:
                    print(f'Données reçues: {line}')
                    break

            # Fermer le port série après l'envoie des data
            ser.close()
            print('Données reçues, port série fermé.')
        except KeyboardInterrupt:
            # Fermer le port série lorsque l'utilisateur appuie sur Ctrl+C
            ser.close()
            print('Port série fermé.')
    else:
        print(f'Échec de l\'ouverture du port série {port}.')

def moveJ(PoseJ):
    steps = ang2step(PoseJ, LastPoseJ)
    data = ','.join(str(i) for i in steps)
    send(data)
    time.sleep(1)
    receive()

def open_gripper():
    send("open")
    time.sleep(1)
    receive()

def close_gripper():
    send("close")
    # receive()

if __name__=="__main__":

    LastPoseJ = [0,0,0,0,0,0]
    print(ang2step([30,59,80,30,90,30], LastPoseJ))

    # open_gripper()
    # moveJ([30,59,80,0,90,0])
    # time.sleep(0.5)
    # close_gripper()
    # time.sleep(2)
    # moveJ([0,0,0,0,0,0])
    # open_gripper()
    # time.sleep(1)

