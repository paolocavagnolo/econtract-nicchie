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

#define M_CUCIRE_SPEED  500
#define M_CUCIRE_ACC    500

#define M_MANICI_SPEED  25000
#define M_MANICI_ACC    25000

#define CORSA_MANICI    125000
#define ZERO_OFFSET_MANICI 250


FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *M_CUCIRE = NULL;
FastAccelStepper *M_MANICI = NULL;

uint8_t STATE = 0; // 0 REMOTE - 1 AUTO - 2 MAN

void setup() {

  delay(1000);

  // DEBUG
  Serial.begin(115200);
  delay(100);

  // LED OFF
  pinMode(37, INPUT);

  // PANEL INPUT
  pinMode(ADIO1, INPUT);  // cucire marius
  pinMode(ADIO2, INPUT);  // manici marius
  pinMode(ADIO3, INPUT);  // manici finecorsa

  // DRIVER OUTPUT
  pinMode(DIO11, OUTPUT); // cucire enable
  pinMode(DIO14, OUTPUT); // manici enable
  digitalWrite(DIO11, LOW);
  digitalWrite(DIO14, HIGH);

  engine.init();

  M_CUCIRE = engine.stepperConnectToPin(DIO9);

  if (M_CUCIRE) {
      M_CUCIRE->setDirectionPin(DIO10);  
      M_CUCIRE->setSpeedInHz(M_CUCIRE_SPEED);  
      M_CUCIRE->setAcceleration(M_CUCIRE_ACC);   
  }

  M_MANICI = engine.stepperConnectToPin(DIO12);

  if (M_MANICI) {
      M_MANICI->setDirectionPin(DIO13);  
      M_MANICI->setSpeedInHz(M_MANICI_SPEED);  
      M_MANICI->setAcceleration(M_MANICI_ACC);   
  }

  // AUDIO SIGNAL
  pinMode(DIO15, OUTPUT); // segnale audio cucire
  pinMode(DIO16, OUTPUT); // segnale audio manici
  digitalWrite(DIO15, LOW);
  digitalWrite(DIO16, LOW);


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

bool prima_volta_cucire = true;
bool prima_uscita_cucire = true;

bool prima_volta_manici = true;
bool prima_uscita_manici = true;

int S = -1;
bool P = true, E = false;
unsigned long T = 0;


void loop() {

  logica_cucire();

  logica_manici();

}

void logica_cucire() {

  if (digitalRead(ADIO1)) {

    if (prima_volta_cucire) {
      prima_volta_cucire = false;
      prima_uscita_cucire = true;

      digitalWrite(DIO11, HIGH);
      digitalWrite(DIO15, HIGH);
      delay(100);
      S = 0;
      M_CUCIRE->runForward();
    }

  } else {

    if (prima_uscita_cucire) {
      prima_uscita_cucire = false;
      prima_volta_cucire = true;

      M_CUCIRE->stopMove();

      delay(1000);
      S = -1;
      digitalWrite(DIO11, LOW);
      digitalWrite(DIO15, LOW);

    }
    delay(100);

  }


  if (S == 0) {

    if (P) {
      P = false;
      E = false;
      M_CUCIRE->setSpeedInHz(500);
      M_CUCIRE->applySpeedAcceleration();
      T = millis();
      Serial.println("parte!");
    }

    if ((millis() - T) > 10000) {
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
      M_CUCIRE->setSpeedInHz(1000);
      M_CUCIRE->applySpeedAcceleration();
      T = millis();
      Serial.println(S);
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
      M_CUCIRE->setSpeedInHz(1500);
      M_CUCIRE->applySpeedAcceleration();
      T = millis();
      Serial.println(S);
    }

    if ((millis() - T) > 5000) {
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
      M_CUCIRE->setSpeedInHz(250);
      M_CUCIRE->applySpeedAcceleration();
      T = millis();
      Serial.println(S);
    }

    if ((millis() - T) > 10000) {
      E = true;
    }


    if (E) {
      E = false;
      S = 0;
      P = true;
    }
  }
}

void logica_manici_test() {

  digitalWrite(DIO14, LOW);
  
  goToHome_manici();
  digitalWrite(DIO14, HIGH);
  delay(1000);

}

void logica_manici() {

  if (digitalRead(ADIO2)) {
  //if (true) {
    if (prima_volta_manici) {
      prima_volta_manici = false;
      prima_uscita_manici = true;

      digitalWrite(DIO14, LOW);
      digitalWrite(DIO16, HIGH);
      delay(100);
      goToHome_manici();

      M_MANICI->moveTo(CORSA_MANICI);
    }

    if (!M_MANICI->isRunning()) {
      if (M_MANICI->getCurrentPosition() == 0) {
        M_MANICI->moveTo(CORSA_MANICI);
      } else if (M_MANICI->getCurrentPosition() == CORSA_MANICI) {
        M_MANICI->moveTo(0);
      } else {
        M_MANICI->moveTo(0);
      }
    } 

  } else {
    if (prima_uscita_manici) {
      prima_uscita_manici = false;
      prima_volta_manici = true;

      M_MANICI->stopMove();
      while(M_MANICI->isRunning()){};
      goToHome_manici();

      delay(1000);
      digitalWrite(DIO14, HIGH);
      digitalWrite(DIO16, LOW);
    }
    delay(100);

  }
}

void goToHome_manici() {

  bool exit_flag = false;
  
  M_MANICI->runBackward();

  while (!exit_flag) {

    if (digitalRead(ADIO3)) {
      M_MANICI->forceStop();
      delay(100);
      M_MANICI->move(ZERO_OFFSET_MANICI);
      while (M_MANICI->isRunning()) {}
      exit_flag = true;
    }

  }

  M_MANICI->setCurrentPosition(0);

}