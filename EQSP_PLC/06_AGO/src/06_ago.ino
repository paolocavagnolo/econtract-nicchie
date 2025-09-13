// 18/08/2025

// DRIVER DM320T
// 01  02 03 04 05  06 
// ON ON ON ON ON ON 

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

#define BASE_SPEED  40
#define BASE_ACC    160

#define TOP_SPEED  500
#define TOP_ACC    5000

#define MAN_SPEED 500
#define MAN_ACC 1000

#define ZERO_OFFSET_BASE 5
#define ZERO_OFFSET_TOP 5

#define CORSA_BASE -150
#define CORSA_TOP 1000

FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *BASE = NULL;
FastAccelStepper *TOP = NULL;

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
  pinMode(ADIO2, INPUT);  // MAN

  pinMode(ADIO4, INPUT);  // FWD
  pinMode(ADIO5, INPUT);  // BWD

  pinMode(ADIO6, INPUT);  // SEL BASE
  pinMode(ADIO7, INPUT);  // SEL TOP

  pinMode(ADIO8, INPUT);  // FINE CORSA BASE
  pinMode(DIO9, INPUT);   // FINE CORSA TOP

  // DRIVER OUTPUT

  pinMode(DIO12, OUTPUT);
  pinMode(DIO15, OUTPUT);
  disableMotor();

  engine.init();

  BASE = engine.stepperConnectToPin(DIO10);

  if (BASE) {
      BASE->setDirectionPin(DIO11, false);  
      BASE->setSpeedInHz(BASE_SPEED);  
      BASE->setAcceleration(BASE_ACC);   
  }

  TOP = engine.stepperConnectToPin(DIO13);

  if (TOP) {
      TOP->setDirectionPin(DIO14, false);  
      TOP->setSpeedInHz(TOP_SPEED);  
      TOP->setAcceleration(TOP_ACC);   
  } 

  // BUZZER 
  ledcSetup(0, 1000, 8);
  ledcAttachPin(BUZZ, 0);

  delay(1000);

  // INITIAL STATE
  if (digitalRead(ADIO2)) {
    STATE = 2;
  } else {
    if (digitalRead(ADIO1)) {
      STATE = 1;
    } else {
      STATE = 0;
    }
  }

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

unsigned long t_MAN = 0, tS_MAN = 0;
bool s_MAN = false, p_MAN = true, f_MAN = false;
unsigned long t_AUTO = 0, tS_AUTO = 0;
bool s_AUTO = false, p_AUTO = true, f_AUTO = false;
bool f_REM = false;
unsigned long t_FWD = 0, tS_FWD = 0;
bool s_FWD = false, p_FWD = true, f_FWD_GO = false, f_FWD_STOP = false;
unsigned long t_BWD = 0, tS_BWD = 0;
bool s_BWD = false, p_BWD = true, f_BWD_GO = false, f_BWD_STOP = false;
unsigned long t_SEL_1 = 0, tS_SEL_1 = 0;
bool s_SEL_1 = false, p_SEL_1 = true;
unsigned long t_SEL_2 = 0, tS_SEL_2 = 0;
bool s_SEL_2 = false, p_SEL_2 = true;

bool HOMED = false;
unsigned long tSequenza = 0;

void loop() {

  check_MAN();
  check_AUTO();
  check_FWD();
  check_BWD();

  check_SEL_1();
  check_SEL_2();

  check_STATE();

  logica();

}

void logica() {
  if (STATE == 0) {
    // REMOTO - IDLE
    delay(50);

  } else if (STATE == 1) {
    // AUTO

    if (!BASE->isRunning()) {

      if (BASE->getCurrentPosition() == CORSA_BASE) {
        act_BASE(-CORSA_BASE,6);
      }
      else if (BASE->getCurrentPosition() == 0) {
        act_BASE(CORSA_BASE,6);
      } else {
        long pos_base = BASE->getCurrentPosition();
        act_BASE(0 - pos_base,6);
      }

    }

    if ((millis() - tSequenza) > 3000){
      if (!TOP->isRunning()) {

        if (TOP->getCurrentPosition() == CORSA_TOP) {
          act_TOP(-CORSA_TOP,6);
        }
        else if (TOP->getCurrentPosition() == 0) {
          act_TOP(CORSA_TOP,6);
        } else {
          long pos_top = TOP->getCurrentPosition();
          act_BASE(0 - pos_top,6);
        }

      }
    }
    

  } else if (STATE == 2) {
    // MAN
    if (s_SEL_1) {
      if (f_FWD_GO) {
        f_FWD_GO = false;

        BASE->runForward();
      }

      if (f_FWD_STOP) {
        f_FWD_STOP = false;

        BASE->stopMove();
      }

      if (f_BWD_GO) {
        f_BWD_GO = false;

        BASE->runBackward();
      }

      if (f_BWD_STOP) {
        f_BWD_STOP = false;

        BASE->stopMove();
      }  
    }

    else if (s_SEL_2) {
      if (f_FWD_GO) {
        f_FWD_GO = false;

        TOP->runForward();
      }

      if (f_FWD_STOP) {
        f_FWD_STOP = false;

        TOP->stopMove();
      }

      if (f_BWD_GO) {
        f_BWD_GO = false;

        TOP->runBackward();
      }

      if (f_BWD_STOP) {
        f_BWD_STOP = false;

        TOP->stopMove();
      }  
    }
    
  }

}

void check_STATE() {

  if (f_MAN) {

    f_MAN = false;
    STATE = 2;
    enableMotor();

    BASE->stopMove();
    while (BASE->isRunning());

    TOP->stopMove();
    while (TOP->isRunning());

    BASE->setSpeedInHz(BASE_SPEED);  
    BASE->setAcceleration(BASE_ACC);
    BASE->applySpeedAcceleration();

    TOP->setSpeedInHz(MAN_SPEED);  
    TOP->setAcceleration(MAN_ACC);
    TOP->applySpeedAcceleration();

  } else if ((f_AUTO) and (STATE != 2)) {

    f_AUTO = false;
    STATE = 1;

    enableMotor();

    if (!HOMED) {
      goToHome();
    }

    if ((BASE->isRunning()) || (TOP->isRunning())) {
      BASE->stopMove();
      TOP->stopMove();

      while ((BASE->isRunning()) || (TOP->isRunning())) {}
    }

    Serial.println("INIZIO LOOP");
    tSequenza = millis();

  } else if (f_REM) {

    f_REM = false;
    if (digitalRead(ADIO1)) {

      STATE = 1;
      f_AUTO = true;

    } else {

      STATE = 0;

      goToHome();

      disableMotor();

    }

  }

}

void check_MAN() {
  if (!s_MAN) {
    if ((millis() - t_MAN) > 200) {
      if (digitalRead(ADIO2)) {
        if (p_MAN) {
          p_MAN = false;
          tS_MAN = millis();
        }
        if ((millis() - tS_MAN) > 200) {
          t_MAN = millis();
          s_MAN = true;
          
          // PREMUTO DOPO 200 ms 
          Serial.println("PUSHED BTN_MAN");

          ledcWriteTone(0, 1000);
          delay(50);
          ledcWriteTone(0, 0);
          f_MAN = true;

        }
      } else {
        tS_MAN = millis();
      }
    }
  } else {
    if ((millis() - t_MAN) > 200) {
      if (!digitalRead(ADIO2)) {
        t_MAN = millis();
        s_MAN = false;
        p_MAN = true;

        // RILASCIATO DOPO 200 ms
        Serial.println("RELEASED BTN_MAN");
        f_REM = true;
        ledcWriteTone(0, 1000);
        delay(50);
        ledcWriteTone(0, 0);
        
      }
    }
  }
}

void check_AUTO() {
  if (!s_AUTO) {
    if ((millis() - t_AUTO) > 200) {
      if (digitalRead(ADIO1)) {
        if (p_AUTO) {
          p_AUTO = false;
          tS_AUTO = millis();
        }
        if ((millis() - tS_AUTO) > 200) {
          t_AUTO = millis();
          s_AUTO = true;
          
          // PREMUTO DOPO 200 ms 
          Serial.println("PUSHED BTN_AUTO");

          ledcWriteTone(0, 1000);
          delay(50);
          ledcWriteTone(0, 0);
          f_AUTO = true;
        }
      } else {
        tS_AUTO = millis();
      }
    }
  } else {
    if ((millis() - t_AUTO) > 200) {
      if (!digitalRead(ADIO1)) {
        t_AUTO = millis();
        s_AUTO = false;
        p_AUTO = true;

        // RILASCIATO DOPO 200 ms
        Serial.println("RELEASED BTN_AUTO");
        f_REM = true;
        ledcWriteTone(0, 1000);
        delay(50);
        ledcWriteTone(0, 0);
      }
    }
  }
}

void check_FWD() {
  if (!s_FWD) {
    if ((millis() - t_FWD) > 200) {
      if (digitalRead(ADIO4)) {
        if (p_FWD) {
          p_FWD = false;
          tS_FWD = millis();
        }
        if ((millis() - tS_FWD) > 200) {
          t_FWD = millis();
          s_FWD = true;
          
          // PREMUTO DOPO 200 ms 
          Serial.println("PUSHED BTN_FWD");

          ledcWriteTone(0, 1000);
          delay(50);
          ledcWriteTone(0, 0);

          f_FWD_GO = true;
        }
      } else {
        tS_FWD = millis();
      }
    }
  } else {
    if ((millis() - t_FWD) > 200) {
      if (!digitalRead(ADIO4)) {
        t_FWD = millis();
        s_FWD = false;
        p_FWD = true;

        // RILASCIATO DOPO 200 ms
        Serial.println("RELEASED BTN_FWD");
        f_FWD_STOP = true;
        
      }
    }
  }
}

void check_BWD() {
  if (!s_BWD) {
    if ((millis() - t_BWD) > 200) {
      if (digitalRead(ADIO5)) {
        if (p_BWD) {
          p_BWD = false;
          tS_BWD = millis();
        }
        if ((millis() - tS_BWD) > 200) {
          t_BWD = millis();
          s_BWD = true;
          
          // PREMUTO DOPO 200 ms 
          Serial.println("PUSHED BTN_BWD");
          ledcWriteTone(0, 1000);
          delay(50);
          ledcWriteTone(0, 0);

          f_BWD_GO = true;

        }
      } else {
        tS_BWD = millis();
      }
    }
  } else {
    if ((millis() - t_BWD) > 200) {
      if (!digitalRead(ADIO5)) {
        t_BWD = millis();
        s_BWD = false;
        p_BWD = true;

        // RILASCIATO DOPO 200 ms
        Serial.println("RELEASED BTN_BWD");

        f_BWD_STOP = true;
        
      }
    }
  }
}

void check_SEL_1() {
  if (!s_SEL_1) {
    if ((millis() - t_SEL_1) > 200) {
      if (digitalRead(ADIO6)) {
        if (p_SEL_1) {
          p_SEL_1 = false;
          tS_SEL_1 = millis();
        }
        if ((millis() - tS_SEL_1) > 200) {
          t_SEL_1 = millis();
          s_SEL_1 = true;
          
          // PREMUTO DOPO 200 ms 
          Serial.println("PUSHED BTN_SEL_1");

          ledcWriteTone(0, 1000);
          delay(50);
          ledcWriteTone(0, 0);

        }
      } else {
        tS_SEL_1 = millis();
      }
    }
  } else {
    if ((millis() - t_SEL_1) > 200) {
      if (!digitalRead(ADIO6)) {
        t_SEL_1 = millis();
        s_SEL_1 = false;
        p_SEL_1 = true;

        // RILASCIATO DOPO 200 ms
        Serial.println("RELEASED BTN_SEL_1");
      }
    }
  }
}

void check_SEL_2() {
  if (!s_SEL_2) {
    if ((millis() - t_SEL_2) > 200) {
      if (digitalRead(ADIO7)) {
        if (p_SEL_2) {
          p_SEL_2 = false;
          tS_SEL_2 = millis();
        }
        if ((millis() - tS_SEL_2) > 200) {
          t_SEL_2 = millis();
          s_SEL_2 = true;
          
          // PREMUTO DOPO 200 ms 
          Serial.println("PUSHED BTN_SEL_2");

          ledcWriteTone(0, 1000);
          delay(50);
          ledcWriteTone(0, 0);

        }
      } else {
        tS_SEL_2 = millis();
      }
    }
  } else {
    if ((millis() - t_SEL_2) > 200) {
      if (!digitalRead(ADIO7)) {
        t_SEL_2 = millis();
        s_SEL_2 = false;
        p_SEL_2 = true;

        // RILASCIATO DOPO 200 ms
        Serial.println("RELEASED BTN_SEL_2");
      }
    }
  }
}


void goToHome() {

  if ((BASE->isRunning()) || (TOP->isRunning())) {
    BASE->stopMove();
    TOP->stopMove();
    while ((BASE->isRunning()) || (TOP->isRunning())) {}
  }

  bool exit_flag_base = false;
  bool exit_flag_top = false;

  BASE->setSpeedInHz(BASE_SPEED);  
  BASE->setAcceleration(BASE_ACC);
  BASE->applySpeedAcceleration();

  TOP->setSpeedInHz(MAN_SPEED);  
  TOP->setAcceleration(MAN_ACC);
  TOP->applySpeedAcceleration();

  BASE->runForward();

  while (!exit_flag_base) {

    if (digitalRead(ADIO8)) {
      BASE->forceStop();
      exit_flag_base = true;
    }

  }

  TOP->runBackward();

  while (!exit_flag_top) {

    if (digitalRead(DIO9)) {
      TOP->forceStop();
      exit_flag_top = true;
    }

  }

  delay(100);

  BASE->move(ZERO_OFFSET_BASE);
  TOP->move(ZERO_OFFSET_TOP);

  while ((BASE->isRunning()) || (TOP->isRunning())) {}

  BASE->setCurrentPosition(0);
  TOP->setCurrentPosition(0);

  HOMED = true;

}

void disableMotor() {
  delay(100);  // DA TESTARE
  digitalWrite(DIO12, LOW);
  digitalWrite(DIO15, LOW);
}

void enableMotor() {
  digitalWrite(DIO12, HIGH);
  digitalWrite(DIO15, HIGH);
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
