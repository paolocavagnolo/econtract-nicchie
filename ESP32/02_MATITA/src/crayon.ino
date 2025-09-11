#include "FastAccelStepper.h"

#define BASE_PULSE 21
#define BASE_DIR 22

#define TOP_PULSE 18
#define TOP_DIR 19

#define SENS_PIN 5
#define AUDIO_PIN 21

FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *base = NULL;
FastAccelStepper *top = NULL;

#define CORSA_ROT 10*3200
#define CORSA_VER 20*3200

void setup() {
	delay(1000);
	
	Serial.begin(9600);
	delay(100);

	pinMode(SENS_PIN, INPUT_PULLUP);
  pinMode(AUDIO_PIN, OUTPUT);
  digitalWrite(AUDIO_PIN, HIGH);
	delay(100);

	engine.init();
  base = engine.stepperConnectToPin(BASE_PULSE);

  if (base) {
      base->setDirectionPin(BASE_DIR);  
      base->setSpeedInHz(3200);  
      base->setAcceleration(1600);  
      //base->moveTo(-CORSA_ROT);  
  }

  top = engine.stepperConnectToPin(TOP_PULSE);

  if (top) {
      top->setDirectionPin(TOP_DIR);  
      top->setSpeedInHz(5000);  
      top->setAcceleration(2000);  
      //top->runForward();  
  }

  delay(100);

}

unsigned long tClosing = 0;
bool presence = false;
bool pClose = false;
bool pOpen = true;

unsigned long t_BTN1 = 0, tS_BTN1 = 0;
bool s_BTN1 = false, p_BTN1 = true;

void loop() {

  
  if (!base->isRunning()) {

    if (base->getCurrentPosition() == -CORSA_ROT) {
      base->moveTo(CORSA_ROT);
    }

    if (base->getCurrentPosition() == CORSA_ROT) {
      base->moveTo(-CORSA_ROT);
    }

    if (base->getCurrentPosition() == 0) {
      base->moveTo(-CORSA_ROT);
    }

  }

  if (!top->isRunning()) {

    if (top->getCurrentPosition() == 0) {
      top->moveTo(CORSA_VER);
    }

    if (top->getCurrentPosition() == CORSA_VER) {
      top->moveTo(0);
    }

  }



}

void carillon_on() {
  base->runForward();
  top->runForward();
  digitalWrite(AUDIO_PIN,LOW);
}

void carillon_off() {
  base->stopMove();
  top->stopMove();
  digitalWrite(AUDIO_PIN,HIGH);
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
          presence = true;
          if (pOpen) {
          	pOpen = false;
          	Serial.println("APRO");
          	carillon_on();
          }
          pClose = true;
          tClosing = millis();

          
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
        presence = false;
        tClosing = millis();
        
      }
    }
  }
}