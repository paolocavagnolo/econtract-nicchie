#include "FastAccelStepper.h"

#define MOT_PULSE 25
#define MOT_DIR 27

FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *mot = NULL;

#define CORSA 100

void setup() {
	delay(1000);
	
	Serial.begin(9600);
	delay(100);

	engine.init();
  mot = engine.stepperConnectToPin(MOT_PULSE);

  if (mot) {
      mot->setDirectionPin(MOT_DIR);  
      mot->setSpeedInHz(1000);  
      mot->setAcceleration(3000);  
      mot->runForward();  
  }

  delay(100);

}

int S = 0;
bool P = true, E = false;
unsigned long T = 0;

void loop() {


  if (S == 0) {

    if (P) {
      P = false;
      E = false;
      mot->setSpeedInHz(1000);
      mot->applySpeedAcceleration();
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
      mot->setSpeedInHz(2500);
      mot->applySpeedAcceleration();
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
      mot->setSpeedInHz(5000);
      mot->applySpeedAcceleration();
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
      mot->setSpeedInHz(500);
      mot->applySpeedAcceleration();
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
