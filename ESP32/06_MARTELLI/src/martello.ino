#include "FastAccelStepper.h"

#define MOT_A_PULSE 27
#define MOT_A_DIR 25
#define MOT_A_EN 26

#define MOT_B_PULSE 32
#define MOT_B_DIR 16
#define MOT_B_EN 33

#define MOT_C_PULSE 19
#define MOT_C_DIR 17
#define MOT_C_EN 18

#define MOT_D_PULSE 22
#define MOT_D_DIR 23
#define MOT_D_EN 21

#define MOT_E_PULSE 15
#define MOT_E_DIR 14
#define MOT_E_EN 13

#define INPUT_SIGNAL 34

FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *mot_A = NULL;
FastAccelStepper *mot_B = NULL;
FastAccelStepper *mot_C = NULL;
FastAccelStepper *mot_D = NULL;
FastAccelStepper *mot_E = NULL;

#define MOT_SPEED 8000
#define MOT_ACC 16000

void setup() {
	delay(1000);
	
	Serial.begin(9600);
	delay(100);

  pinMode(MOT_A_EN,OUTPUT);
  pinMode(MOT_B_EN,OUTPUT);
  pinMode(MOT_C_EN,OUTPUT);
  pinMode(MOT_D_EN,OUTPUT);
  pinMode(MOT_E_EN,OUTPUT);
  pinMode(INPUT_SIGNAL, INPUT);

	engine.init();
  mot_A = engine.stepperConnectToPin(MOT_A_PULSE);
  mot_B = engine.stepperConnectToPin(MOT_B_PULSE);
  mot_C = engine.stepperConnectToPin(MOT_C_PULSE);
  mot_D = engine.stepperConnectToPin(MOT_D_PULSE);
  mot_E = engine.stepperConnectToPin(MOT_E_PULSE);

  if (mot_A) {
      mot_A->setDirectionPin(MOT_A_DIR);  
      mot_A->setSpeedInHz(MOT_SPEED);  
      mot_A->setAcceleration(MOT_ACC);  
      //mot_A->runBackward();  
  }

  if (mot_B) {
      mot_B->setDirectionPin(MOT_B_DIR);  
      mot_B->setSpeedInHz(MOT_SPEED);  
      mot_B->setAcceleration(MOT_ACC);  
      //mot_B->runBackward();  
  }

  if (mot_C) {
      mot_C->setDirectionPin(MOT_C_DIR);  
      mot_C->setSpeedInHz(MOT_SPEED);  
      mot_C->setAcceleration(MOT_ACC);  
      //mot_C->runBackward();  
  }

  if (mot_D) {
      mot_D->setDirectionPin(MOT_D_DIR);  
      mot_D->setSpeedInHz(MOT_SPEED);  
      mot_D->setAcceleration(MOT_ACC);  
      //mot_D->runBackward();  
  }

  if (mot_E) {
      mot_E->setDirectionPin(MOT_E_DIR);  
      mot_E->setSpeedInHz(MOT_SPEED);  
      mot_E->setAcceleration(MOT_ACC);  
      //mot_E->runBackward();  
  }


}

#define OFFSET_A -1000
#define OFFSET_B 0
#define OFFSET_C 0
#define OFFSET_D -900
#define OFFSET_E 0

#define TIME_OFFSET 1600

long CORSA = 600;

bool go_A = false, go_B = false, go_C = false, go_D = false, go_E = false;

int iS, old_iS;
unsigned long T_MOT_A = 0, T_MOT_D = 0;
unsigned long T_MOT_B = 0, T_MOT_C = 0;
unsigned long T_MOT_E = 0;

void loop() {

  check_input();
 
  logic();

}

void check_input() {

  iS = digitalRead(INPUT_SIGNAL);

  if (iS != old_iS) {

    if (iS) {

      Serial.println("INIZIO");

      go_A = true;
      go_B = true;
      go_C = true;
      go_D = true;
      go_E = true;

      pinMode(MOT_A_EN, INPUT);
      pinMode(MOT_B_EN, INPUT);
      pinMode(MOT_C_EN, INPUT);
      pinMode(MOT_D_EN, INPUT);
      pinMode(MOT_E_EN, INPUT);

      delay(100);
     
      mot_B->setCurrentPosition(0);
      mot_C->setCurrentPosition(0);
      mot_E->setCurrentPosition(0);

      delay(100);

      mot_A->move(OFFSET_A);
      mot_D->move(OFFSET_D);

      while (mot_D->isRunning() || mot_A->isRunning()){};

      delay(100);

      mot_A->setCurrentPosition(0);
      mot_D->setCurrentPosition(0);

      delay(100);

      mot_A->move(CORSA/2);
      mot_B->move(CORSA/2);
      mot_C->move(CORSA/2);
      mot_D->move(CORSA/2);
      mot_E->move(CORSA/2);


    } else {

      Serial.println("FINE");

      go_A = false;
      go_B = false;
      go_C = false;
      go_D = false;
      go_E = false;

      mot_A->moveTo(-OFFSET_A);
      mot_B->moveTo(0);
      mot_C->moveTo(0);
      mot_D->moveTo(-OFFSET_D);
      mot_E->moveTo(0);
      
      while (mot_D->isRunning() || mot_A->isRunning() || mot_E->isRunning() || mot_B->isRunning() || mot_C->isRunning()){};

      delay(100);

      pinMode(MOT_A_EN, OUTPUT);
      pinMode(MOT_B_EN, OUTPUT);
      pinMode(MOT_C_EN, OUTPUT);
      pinMode(MOT_D_EN, OUTPUT);
      pinMode(MOT_E_EN, OUTPUT);

    }

    old_iS = iS;

  } 

}

void logic() {

  if (go_A) {
    if (!mot_A->isRunning()) {
      if (mot_A->getCurrentPosition() < 0) {
        mot_A->move(CORSA);
      } else {
        mot_A->move(-CORSA);
      }
    }
  }

  if (go_B) {
    if (!mot_B->isRunning()) {
      if (mot_B->getCurrentPosition() < 0) {
        mot_B->move(CORSA);
      } else {
        mot_B->move(-CORSA);
      }
    }
  }

  if (go_C) {
    if (!mot_C->isRunning()) {
      if (mot_C->getCurrentPosition() < 0) {
        mot_C->move(CORSA);
      } else {
        mot_C->move(-CORSA);
      }
    }
  }

  if (go_D) {
    if (!mot_D->isRunning()) {
      if (mot_D->getCurrentPosition() < 0) {
        mot_D->move(CORSA);
      } else {
        mot_D->move(-CORSA);
      }
    }
  }

  if (go_E) {
    if (!mot_E->isRunning()) {
      if (mot_E->getCurrentPosition() < 0) {
        mot_E->move(CORSA);
      } else {
        mot_E->move(-CORSA);
      }
    }
  } 

 
}


