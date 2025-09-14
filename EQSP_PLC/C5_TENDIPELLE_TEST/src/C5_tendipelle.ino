#define ADIO1   1
#define ADIO2   2
#define ADIO3   4
#define ADIO4   5
#define ADIO5   6
#define ADIO6   7
#define ADIO7   8
#define ADIO8   9

#define DIO9    11
#define DIO10   12
#define DIO11   13
#define DIO12   14
#define DIO13   15
#define DIO14   16
#define DIO15   19  
#define DIO16   20

#define BUZZ    3

#define RS485_DI    18
#define RS485_RO    17 
#define RS485_EN    35

#define LED_RED     38
#define LED_BLUE    37 //INPUT: OFF / OUTPUT_LOW: ON_BT / OUTPUT_HIGH: ON_WIFI

#define SDA_PIN 33
#define SCL_PIN 34

#include "FastAccelStepper.h"

#define M1_SPEED  8000
#define M1_ACC    8000

#define M2_SPEED  8000
#define M2_ACC    8000

#define CORSA     1000

FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *M1 = NULL;
FastAccelStepper *M2 = NULL;

void setup() {

  delay(1000);

  // DEBUG
  Serial.begin(115200);
  delay(100);

  // LED OFF
  pinMode(37, INPUT);

  // PANEL INPUT
  pinMode(ADIO1, INPUT);  // AUTO

  // DRIVER OUTPUT
  pinMode(DIO13, OUTPUT);

  engine.init();

  M1 = engine.stepperConnectToPin(DIO9);

  if (M1) {
      M1->setDirectionPin(DIO10);  
      M1->setSpeedInHz(M1_SPEED);  
      M1->setAcceleration(M1_ACC);   
  }

  M2 = engine.stepperConnectToPin(DIO11);

  if (M2) {
      M2->setDirectionPin(DIO12);  
      M2->setSpeedInHz(M2_SPEED);  
      M2->setAcceleration(M2_ACC);   
  }

  // BUZZER 
  ledcSetup(0, 1000, 8);
  ledcAttachPin(BUZZ, 0);

  // DOUBLE BEEP FOR SETUP END
  ledcWriteTone(0, 1000);
  delay(50);
  ledcWriteTone(0, 0);
  delay(175);
  ledcWriteTone(0, 1000);
  delay(50);
  ledcWriteTone(0, 0);

  delay(1000);

}

bool prima_volta = true;
bool prima_uscita = true;


int S = -1;
bool P = true, E = false;
unsigned long T = 0;

void loop() {

  if (digitalRead(ADIO1)) {
    if (prima_volta) {
      prima_volta = false;
      prima_uscita = true;

      digitalWrite(DIO11, LOW);
      S = 0;
      M1->moveTo(CORSA);
    }

    if (!M1->isRunning()) {

      if (M1->getCurrentPosition() == -CORSA) {
        M1->moveTo(CORSA);
      }

      else if (M1->getCurrentPosition() == CORSA) {
        M1->moveTo(-CORSA);
      }

      else if (M1->getCurrentPosition() == 0) {
        M1->moveTo(-CORSA);
      } 

      else {
        M1->moveTo(0);
      }

    }


    if (!M2->isRunning()) {

      if (M2->getCurrentPosition() == -CORSA) {
        M2->moveTo(CORSA);
      }

      else if (M2->getCurrentPosition() == CORSA) {
        M2->moveTo(-CORSA);
      }

      else if (M2->getCurrentPosition() == 0) {
        M2->moveTo(-CORSA);
      }

      else {
        M2->moveTo(0);
      }

    }

  } else {
    if (prima_uscita) {
      prima_uscita = false;
      prima_volta = true;

      M1->stopMove();
      while(M1->isRunning()){};
      M1->moveTo(0);
      while(M1->isRunning()){};

      delay(1000);
      S = -1;
      digitalWrite(DIO11, HIGH);
    }
  }



}
