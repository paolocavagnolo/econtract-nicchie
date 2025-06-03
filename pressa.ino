#include "FastAccelStepper.h"

#define MOT_PULSE 25
#define MOT_DIR 26

#define SENS_PIN 5

FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *mot = NULL;
FastAccelStepper *top = NULL;

void setup() {
	delay(1000);
	
	Serial.begin(9600);
	delay(100);

	pinMode(SENS_PIN, INPUT_PULLUP);
	delay(100);

	engine.init();
  mot = engine.stepperConnectToPin(MOT_PULSE);

  if (mot) {
      mot->setDirectionPin(MOT_DIR);  
      mot->setSpeedInHz(10000);  
      mot->setAcceleration(10000);  
      //mot->runBackward();  
  }

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

unsigned long tDebug = 0;

void loop() {

	check_BTN1();

  if (loop_pressa) {

    if (!mot->isRunning()) {
      if (mot->getCurrentPosition() == target_pos) {
        mot->moveTo(100);
      }

      if (mot->getCurrentPosition() == 100) {
        mot->moveTo(target_pos);
      }

    }

  }	

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
          } else {
            loop_pressa = false;
            Serial.println("FINE LOOP");
            mot->stopMove();
            while (mot->isRunning()) {}
            mot->moveTo(0);
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