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

#define M1_SPEED  1000
#define M1_ACC    3000

FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *M1 = NULL;

uint8_t STATE = 0; // 0 REMOTE - 1 AUTO - 2 MAN

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
  pinMode(DIO11, OUTPUT);

  engine.init();

  M1 = engine.stepperConnectToPin(DIO9);

  if (M1) {
      M1->setDirectionPin(DIO10);  
      M1->setSpeedInHz(M1_SPEED);  
      M1->setAcceleration(M1_ACC);   
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
      M1->runForward();
    }

  } else {

    if (prima_uscita) {
      prima_uscita = false;
      prima_volta = true;

      M1->stopMove();

      delay(1000);
      S = -1;
      digitalWrite(DIO11, HIGH);
    }

  }


  if (S == 0) {

    if (P) {
      P = false;
      E = false;
      M1->setSpeedInHz(500);
      M1->applySpeedAcceleration();
      T = millis();
    }

    if ((millis() - T) > 5000) {
      E = true;
    }


    if (E) {
      E = false;
      S = 1;
      P = true;
    }

  } else if (S == 1) {

    if (P) {
      P = false;
      E = false;
      M1->setSpeedInHz(1000);
      M1->applySpeedAcceleration();
      T = millis();
    }

    if ((millis() - T) > 5000) {
      E = true;
    }


    if (E) {
      E = false;
      S = 2;
      P = true;
    }
  } else if (S == 2) {

    if (P) {
      P = false;
      E = false;
      M1->setSpeedInHz(1500);
      M1->applySpeedAcceleration();
      T = millis();
    }

    if ((millis() - T) > 3000) {
      E = true;
    }


    if (E) {
      E = false;
      S = 3;
      P = true;
    }
  } else if (S == 3) {

    if (P) {
      P = false;
      E = false;
      M1->setSpeedInHz(250);
      M1->applySpeedAcceleration();
      T = millis();
    }

    if ((millis() - T) > 1000) {
      E = true;
    }


    if (E) {
      E = false;
      S = 0;
      P = true;
    }
  }



}
