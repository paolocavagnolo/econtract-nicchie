#include "FastAccelStepper.h"

#define MOT_PULSE 25
#define MOT_DIR 26

#define SENS_PIN 5
#define RELAY 21

FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *mot = NULL;
FastAccelStepper *top = NULL;

void setup() {
  delay(1000);
  
  Serial.begin(9600);
  delay(100);

  pinMode(SENS_PIN, INPUT_PULLUP);
  bolleOff();
  pinMode(RELAY, OUTPUT);
  delay(100);

  engine.init();
  mot = engine.stepperConnectToPin(MOT_PULSE);

  if (mot) {
      mot->setDirectionPin(MOT_DIR);  
      mot->setSpeedInHz(8000);  
      mot->setAcceleration(6000);  
      //mot->runForward();  
      //mot->runBackward();
  }

  //while (true) {};

  delay(100);

}

unsigned long tClosing = 0;
bool presence = false;
bool pClose = false;
bool pOpen = true;

unsigned long t_BTN1 = 0, tS_BTN1 = 0;
bool s_BTN1 = false, p_BTN1 = true;

bool loop_pressa = false;

bool fine_corsa = false;
long target_pos = 70000;
long bolle_pos = 55000;
int attesa = 5000;

unsigned long tDebug = 0;
unsigned long tAttesa = 0;
unsigned long tCheck = 0;

int loop_counter = 0;

int S = 0;
bool P = true;
bool E = false;
unsigned long T = 0;

void loop() {

  check_BTN1();

  if (loop_pressa) {

    if (S == 0) { // MI TROVO SU
      if (P) {
        P = false;
        E = false;
        T = millis();
      }

      if ((millis() - T) > 3000) { // ASPETTO 3 SECONDI E PARTO
        E = true;
        mot->moveTo(target_pos);
      }

      if (E) {
        E = false;
        P = true;
        S = 1;
      }
    }

    else if (S == 1) { // DISCESA E ACCENSIONE BOLLE
      if (P) {
        P = false;
        E = false;
        T = millis();
      }

      if (mot->getCurrentPosition() > bolle_pos) {
        bolleOn();
      }

      if (!mot->isRunning()) {
        E = true;
      }
      
      if (E) {
        E = false;
        P = true;
        S = 2;
      }
    }

    else if (S == 2) { // ATTESA CON BOLLE 
      if (P) {
        P = false;
        E = false;
        T = millis();
      }

      if ((millis() - T) > 5000) {
        mot->moveTo(0);
        E = true;
      }
      
      if (E) {
        E = false;
        P = true;
        S = 3;
      }
    }

    else if (S == 3) { // RISALITA 
      if (P) {
        P = false;
        E = false;
        T = millis();
      }

      if (mot->getCurrentPosition() < bolle_pos) {
        bolleOff();
      }

      if (!mot->isRunning()) {
        E = true;
      }
      
      if (E) {
        E = false;
        P = true;
        S = 0;
      }
    }

  }


}

void bolleOn() {
  digitalWrite(RELAY,HIGH);
}

void bolleOff() {
  digitalWrite(RELAY,LOW);
}


void check_BTN1() {
  if (!s_BTN1) {
    if ((millis() - t_BTN1) > 200) {
      if (!digitalRead(SENS_PIN)) {
        if (p_BTN1) {
          p_BTN1 = false;
          tS_BTN1 = millis();
        }
        if ((millis() - tS_BTN1) > 200) {
          t_BTN1 = millis();
          s_BTN1 = true;
          
          // PREMUTO DOPO 200 ms 
          Serial.println("SENS IN");

          if (!loop_pressa) {
            Serial.println("INIZIO LOOP");
            loop_pressa = true;
            mot->moveTo(target_pos);
            bolleOff();
            loop_counter = 0;
          } else {
            loop_pressa = false;
            Serial.println("FINE LOOP");
            mot->stopMove();
            while (mot->isRunning()) {}
            mot->moveTo(0);
            bolleOff();
          }
    
        }
      } else {
        tS_BTN1 = millis();
      }
    }
  } else {
    if ((millis() - t_BTN1) > 200) {
      if (digitalRead(SENS_PIN)) {
        t_BTN1 = millis();
        s_BTN1 = false;
        p_BTN1 = true;

        // RILASCIATO DOPO 200 ms
        Serial.println("SENS OUT");

        
      }
    }
  }
}