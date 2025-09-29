#include "FastAccelStepper.h"

#define BASE_SPEED  40*4
#define BASE_ACC    160*4

#define TOP_SPEED  500*4
#define TOP_ACC    5000*4

#define ZERO_OFFSET_BASE 5*4
#define ZERO_OFFSET_TOP 5*4

#define CORSA_BASE -120*4
#define CORSA_TOP 1400*4

FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *BASE = NULL;
FastAccelStepper *TOP = NULL;

uint8_t STATE = 0; // 0 REMOTE - 1 AUTO - 2 MAN
unsigned long tSequenza = 0;

void setup() {

  delay(1000);

  // DEBUG
  Serial.begin(115200);
  delay(100);

  // DRIVER OUTPUT

  pinMode(21, OUTPUT);
  pinMode(27, OUTPUT);

  disableMotor();

  engine.init();

  BASE = engine.stepperConnectToPin(25);

  if (BASE) {
      BASE->setDirectionPin(26);  
      BASE->setSpeedInHz(BASE_SPEED);  
      BASE->setAcceleration(BASE_ACC);

  }

  TOP = engine.stepperConnectToPin(18);

  if (TOP) {
      TOP->setDirectionPin(19, false);  
      TOP->setSpeedInHz(TOP_SPEED);  
      TOP->setAcceleration(TOP_ACC);   
  } 

  delay(1000);

  pinMode(4, INPUT_PULLUP);
  pinMode(5, INPUT_PULLUP);

  delay(1000);

  tSequenza = millis();

}


bool pAvantiA = true;
bool pAvantiZ = true;

bool pDietroA = true;
bool pDietroZ = true;

unsigned long t_AUTO_G = 0, tS_AUTO_G = 0, tSO_AUTO_G = 0;
bool s_AUTO_G = false, p_AUTO_G = true, f_AUTO_G = false, pO_AUTO_G = true;
bool f_REM_G = false;

unsigned long t_AUTO_H = 0, tS_AUTO_H = 0, tSO_AUTO_H = 0;
bool s_AUTO_H = false, p_AUTO_H = true, f_AUTO_H = false, pO_AUTO_H = true;
bool f_REM_H = false;

bool go_go = false;

void loop() {
  /*
  check_AUTO_G();

  if (f_AUTO_G) {
    f_AUTO_G = false;
    BASE->runForward();
    TOP->runForward();
  }

  if (f_REM_G) {
    f_REM_G = false;
    BASE->stopMove();
    TOP->stopMove();
  }*/

  check_AUTO_H();

  if (f_AUTO_H) {
    f_AUTO_H = false;
    go_go = true;
    tSequenza = millis();
    enableMotor();
    delay(200);
  }

  if (f_REM_H) {
    f_REM_H = false;
    go_go = false;
    BASE->stopMove();
    while(BASE->isRunning()){};
    BASE->moveTo(0);
    while(BASE->isRunning()){};
    TOP->stopMove();
    while(TOP->isRunning()){};
    TOP->moveTo(0);
    while(TOP->isRunning()){};
    disableMotor();
  }

  
  if (go_go) {
    if (!BASE->isRunning()) {

        if (BASE->getCurrentPosition() == CORSA_BASE) {
          act_BASE(-CORSA_BASE,12);
        }
        else if (BASE->getCurrentPosition() == 0) {
          act_BASE(CORSA_BASE,12);
        } else {
          long pos_base = BASE->getCurrentPosition();
          act_BASE(0 - pos_base,12);
        }

      }

      
      if ((millis() - tSequenza) > 6000){
        if (!TOP->isRunning()) {

          if (TOP->getCurrentPosition() == CORSA_TOP) {
            act_TOP(-CORSA_TOP,12);
          }
          else if (TOP->getCurrentPosition() == 0) {
            act_TOP(CORSA_TOP,12);
          } else {
            long pos_top = TOP->getCurrentPosition();
            act_BASE(0 - pos_top,12);
          }

        }
      }
    }

}

void disableMotor() {
  delay(100);  // DA TESTARE
  digitalWrite(21, HIGH);
  digitalWrite(27, HIGH);
}

void enableMotor() {
  digitalWrite(21, LOW);
  digitalWrite(27, LOW);
  delay(100);  // DA TESTARE
}

void act_BASE(int32_t pos, float tempo) {

  long V_max, A_cal;

  float V_media = (float)abs(pos) / tempo;

  V_max = V_media * 2;   // step / s
  A_cal = V_max / (tempo / 2);   // step / s / s

  BASE->setSpeedInHz(V_max);  
  BASE->setAcceleration(A_cal);
  BASE->applySpeedAcceleration();

  BASE->move(pos);

}

void act_TOP(int32_t pos, float tempo) {

  long V_max, A_cal;

  float V_media = (float)abs(pos) / tempo;

  V_max = V_media * 2;   // step / s
  A_cal = V_max / (tempo / 2);   // step / s / s

  TOP->setSpeedInHz(V_max);  
  TOP->setAcceleration(A_cal);
  TOP->applySpeedAcceleration();

  TOP->move(pos);

}

void check_AUTO_G() {
  if (!s_AUTO_G) {
    if ((millis() - t_AUTO_G) > 200) {
      if (!digitalRead(5)) {
        if (p_AUTO_G) {
          p_AUTO_G = false;
          tS_AUTO_G = millis();
        }
        if ((millis() - tS_AUTO_G) > 200) {
          t_AUTO_G = millis();
          s_AUTO_G = true;
          pO_AUTO_G = true;
          
          // PREMUTO DOPO 200 ms 
          Serial.println("PUSHED BTN_AUTO_G");

          f_AUTO_G = true;
        }
      } else {
        tS_AUTO_G = millis();
      }
    }
  } else {
    if ((millis() - t_AUTO_G) > 200) {
      if (digitalRead(5)) {
        if (pO_AUTO_G) {
          pO_AUTO_G = false;
          tSO_AUTO_G = millis();
        }

        if ((millis() - tSO_AUTO_G) > 200) {
          t_AUTO_G = millis();
          s_AUTO_G = false;
          p_AUTO_G = true;

          // RILASCIATO DOPO 200 ms
          Serial.println("RELEASED BTN_AUTO_G");
          f_REM_G = true;

        }
      } else {
        tSO_AUTO_G = millis();
      }
    }
  }
}

void check_AUTO_H() {
  if (!s_AUTO_H) {
    if ((millis() - t_AUTO_H) > 200) {
      if (!digitalRead(4)) {
        if (p_AUTO_H) {
          p_AUTO_H = false;
          tS_AUTO_H = millis();
        }
        if ((millis() - tS_AUTO_H) > 200) {
          t_AUTO_H = millis();
          s_AUTO_H = true;
          pO_AUTO_H = true;
          
          // PREMUTO DOPO 200 ms 
          Serial.println("PUSHED BTN_AUTO_H");

          f_AUTO_H = true;
        }
      } else {
        tS_AUTO_H = millis();
      }
    }
  } else {
    if ((millis() - t_AUTO_H) > 200) {
      if (digitalRead(4)) {
        if (pO_AUTO_H) {
          pO_AUTO_H = false;
          tSO_AUTO_H = millis();
        }

        if ((millis() - tSO_AUTO_H) > 200) {
          t_AUTO_H = millis();
          s_AUTO_H = false;
          p_AUTO_H = true;

          // RILASCIATO DOPO 200 ms
          Serial.println("RELEASED BTN_AUTO_H");
          f_REM_H = true;

        }
      } else {
        tSO_AUTO_H = millis();
      }
    }
  }
}