#include "FastAccelStepper.h"

#define BASE_PULSE 22
#define BASE_DIR 23

#define TOP_PULSE 25
#define TOP_DIR 26

FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *base = NULL;
FastAccelStepper *top = NULL;

#define CORSA 100

void setup() {
	delay(1000);
	
	Serial.begin(9600);
	delay(100);

	engine.init();
  base = engine.stepperConnectToPin(BASE_PULSE);

  if (base) {
      base->setDirectionPin(BASE_DIR);  
      base->setSpeedInHz(8000);  
      base->setAcceleration(8000);  
      //base->runForward();  
  }

  top = engine.stepperConnectToPin(TOP_PULSE);

  if (top) {
      top->setDirectionPin(TOP_DIR);  
      top->setSpeedInHz(8000);  
      top->setAcceleration(8000);  
      //top->runForward();  
  }

  delay(100);

}


void loop() {

  
  if (!base->isRunning()) {

    if (base->getCurrentPosition() == -CORSA) {
      base->moveTo(CORSA);
    }

    if (base->getCurrentPosition() == CORSA) {
      base->moveTo(-CORSA);
    }

    if (base->getCurrentPosition() == 0) {
      base->moveTo(-CORSA);
    }

  }


  if (!top->isRunning()) {

    if (top->getCurrentPosition() == -CORSA) {
      top->moveTo(CORSA);
    }

    if (top->getCurrentPosition() == CORSA) {
      top->moveTo(-CORSA);
    }

    if (top->getCurrentPosition() == 0) {
      top->moveTo(-CORSA);
    }

  }




}
