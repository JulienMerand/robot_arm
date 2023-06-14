#include <AccelStepper.h>
#include<MultiStepper.h>
#include <Servo.h>

// Definition des pins
#define ENA_6 22
#define DIR_6 23
#define PUL_6 24

#define ENA_5 27
#define DIR_5 28
#define PUL_5 29

#define ENA_4 30
#define DIR_4 31
#define PUL_4 32

#define ENA_3 34
#define DIR_3 35
#define PUL_3 36

#define ENA_1 38
#define DIR_1 39
#define PUL_1 40

#define ENA_2B 44
#define DIR_2B 45
#define PUL_2B 46

#define ENA_2A 49
#define DIR_2A 50
#define PUL_2A 51

#define PIN_GRIPPER 52

// Déclaration du gripper
Servo gripper;

// Déclaration des moteurs AccelStepper
AccelStepper stepper1(AccelStepper::DRIVER, PUL_1, DIR_1);
AccelStepper stepper2A(AccelStepper::DRIVER, PUL_2A, DIR_2A);
AccelStepper stepper2B(AccelStepper::DRIVER, PUL_2B, DIR_2B);
AccelStepper stepper3(AccelStepper::DRIVER, PUL_3, DIR_3);
AccelStepper stepper4(AccelStepper::DRIVER, PUL_4, DIR_4);
AccelStepper stepper5(AccelStepper::DRIVER, PUL_5, DIR_5);
AccelStepper stepper6(AccelStepper::DRIVER, PUL_6, DIR_6);

AccelStepper stepper[] = {stepper1, stepper2A, stepper2B, stepper3, stepper4, stepper5, stepper6};
MultiStepper steppers;


// Paramètres
#define speed 500
#define accel 100

void setup() {

  Serial.begin(115200);

  // Broche de contrôle d'activation du moteur
  stepper[0].setEnablePin(ENA_1);
  stepper[1].setEnablePin(ENA_2A);
  stepper[2].setEnablePin(ENA_2B);
  stepper[3].setEnablePin(ENA_3);
  stepper[4].setEnablePin(ENA_4);
  stepper[5].setEnablePin(ENA_5);
  stepper[6].setEnablePin(ENA_6);

  // Activation des moteurs
  for(int i=0; i<7; i++){
    stepper[i].disableOutputs();
  }

  for(int i=0; i<7; i++){
    steppers.addStepper(stepper[i]);
  }

  for(int i=0; i<7; i++){
    int v = 700;
    stepper[i].setMaxSpeed(v);
    stepper[i].setAcceleration(accel);
  }

  //Gripper
  gripper.attach(PIN_GRIPPER);
}

void loop() {
  
  // if(Serial.available()){
  //   String data = Serial.readStringUntil('\n');
  //   data.trim();

  //   if(data == "open"){
  //     open_gripper();
  //     delay(2000);
  //     Serial.println("Gripper ouvert");
  //   }
  //   else if(data == "close"){
  //     close_gripper();
  //     delay(2000);
  //     Serial.println("Gripper ferme");
  //   }
  //   else{

  //     long step[7];
  //     int count = 0;

  //     char* token = strtok((char*)data.c_str(), ",");
  //     while (token != NULL && count < 7) {
  //       step[count] = atol(token);
  //       token = strtok(NULL, ",");
  //       count++;
  //     }
      
  //     set_speed(step);

  //     steppers.moveTo(step);
  //     steppers.runSpeedToPosition();
  //     Serial.println("Deplacement - OK");
  //     delay(2000);

  //   } 

  // }



  // Commande de position en step
  long home[] = {0, 0, 0, 0, 0, 0, 0};
  long step1[] = {2050, -1100, 1100, -8700, 0, 950, 0};
  long step2[] = {1366, -1442, 1442, -7733, 0, 865, 0};
  long step3[] = {1366, -1442, 1442, -7733, -400, -1900, -800};
  long step4[] = {-1366, 1442, -1442, 7733, -133, -1900, -133};
  
  // set_speed(step4);
  open_gripper();
  delay(1000);

  steppers.moveTo(step4);
  steppers.runSpeedToPosition();
  Serial.println("Deplacement 1 - OK");
  delay(1000);

  close_gripper();
  delay(1000);
  // set_speed(step2);

  steppers.moveTo(home);
  steppers.runSpeedToPosition();
  Serial.println("Deplacement 2 - OK");
  delay(1000);

  

  // // data de la forme "open" ou "close"
  // if(Serial.available() != 0) {
  //   String data = Serial.readString();
  //   data.trim();
  //   if(data == "open"){
  //     open_gripper();
  //   }
  //   else if(data == "close"){
  //     close_gripper();
  //   }
  //   else{
  //     Serial.println(data);
  //   }
  // }
}

void open_gripper() {
  gripper.write(85);
  Serial.println("Gripper ouvert");
}

void close_gripper() {
  gripper.write(175);
  Serial.println("Gripper ferme");
}

void set_speed(long step[]){
  // Max de steps
  int max_step = abs(step[0]);
  for(int i=0; i<7; i++){
    if(abs(step[i]) > max_step){
      max_step = abs(step[i]);
    }
  }

  // Vitesse maximale en pas par seconde
  // Accélération en pas par seconde carrée
  for(int i=0; i<7; i++){
    float rate = float(abs(step[i])) / float(max_step);
    int v = int(speed * rate);
    if(v==0){
      v = speed;
    }
    stepper[i].setMaxSpeed(v);
    stepper[i].setAcceleration(accel);
  }
}