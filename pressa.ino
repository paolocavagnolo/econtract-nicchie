#include "FastAccelStepper.h"

#define MOT_PULSE 25
#define MOT_DIR 26

#define SENS_PIN 5

#define SPEED_FAST 12000
#define SPEED_SLOW 6000

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
      mot->setSpeedInHz(SPEED_SLOW);  
      mot->setAcceleration(5000);  
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
long target_A = 55000;
long target_B = 70000;

unsigned long tDebug = 0;

void loop() {

	check_BTN1();

  if (loop_pressa) {

    if (!mot->isRunning()) {

      // MOT IN A - GOES TO B SLOW
      if (mot->getCurrentPosition() == target_A) {
        mot->setSpeedInHz(SPEED_SLOW);  
        mot->moveTo(target_B);
      }

      // MOT IN B - RETURN UP FAST
      if (mot->getCurrentPosition() == target_B) {
        mot->setSpeedInHz(SPEED_FAST);  
        mot->moveTo(100);
      }

      // MOT UP - GOES FAST TO A
      if (mot->getCurrentPosition() == 100) {
        mot->setSpeedInHz(SPEED_FAST); 
        mot->moveTo(target_A);
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
            // GO FAST TO A
            mot->setSpeedInHz(SPEED_FAST); 
            mot->moveTo(target_A);
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
