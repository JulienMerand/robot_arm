# Projet Robot 6 axes - Package ROS2 Humble
Dans le cadre d'un projet personnel, je réalise un bras à 6 degrées de libertés, entièrement imprimé en 3D.
Le modèle est le robot Moveo de BCN3D (https://github.com/BCN3D/BCN3D-Moveo)

<div style="display: flex;">
    <img src="/Images/Station.jpg" alt="Image 1" height="350">
    <img src="/Images/Robot.jpg" alt="Image 2" height="350">
</div>


Commandes utiles :

MICRO AGENT PORT SERIE :
MicroXRCEAgent serial --dev /dev/ttyACM0 -b 115200

ROSBRIDGE_SERVER :
ros2 launch rosbridge_server rosbridge_websocket_launch.xml

FLUTTER SUR TELEPHONE:
adb tcpip 5555   (connecter en usb si erreur, puis enlever)
adb connect 192.168.1.44


Julien MÉRAND
