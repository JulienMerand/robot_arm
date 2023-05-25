import os, grp
[grp.getgrgid(g).gr_name for g in os.getgroups()]

import serial

# Configurer le port série
port = '/dev/ttyACM0'  # Spécifiez votre port série
baudrate = 115200  # Spécifiez le débit en bauds

# Créer une instance de l'objet Serial
ser = serial.Serial(port, baudrate)

# Vérifier si le port série est ouvert
if ser.is_open:
    print(f'Port série {port} ouvert.')

    try:
        while True:
            # # Lire une ligne de données depuis le port série
            # line = ser.readline().decode().strip()

            # # Vérifier si des données ont été reçues
            # if line:
            #     print(f'Données reçues: {line}')

            data = input("Commande gripper ('open'|'close'): ")
            ser.write(data.encode())

    except KeyboardInterrupt:
        # Fermer le port série lorsque l'utilisateur appuie sur Ctrl+C
        ser.close()
        print('Port série fermé.')

else:
    print(f'Échec de l\'ouverture du port série {port}.')