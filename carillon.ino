#include "FastAccelStepper.h"

#define BASE_PULSE 25
#define BASE_DIR 26

#define TOP_PULSE 22
#define TOP_DIR 23

#define SENS_PIN 5
#define AUDIO_PIN 21

FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *base = NULL;
FastAccelStepper *top = NULL;

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
      base->setSpeedInHz(1000);  
      base->setAcceleration(200);  
      //base->runBackward();  
  }

  top = engine.stepperConnectToPin(TOP_PULSE);

  if (top) {
      top->setDirectionPin(TOP_DIR);  
      top->setSpeedInHz(500);  
      top->setAcceleration(100);  
      //top->runBackward();  
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

	check_BTN1();
	
	if (!presence) {
		if ((millis() - tClosing) > 30000) {
			if (pClose) {
				pClose = false;
				Serial.println("CHIUDO");
				pOpen = true;
				carillon_off();
			}
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