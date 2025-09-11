// 18/08/2025
// LOGICA OK
// DA CAMBIARE CORSA QUANDO SI FISSA IL FINECORSA

// DRIVER DM542T
// 01 02 03 04  05  06  07 08 
// ON ON ON OFF OFF OFF ON ON 

// 10/09/2025
// @POLENE
// OK


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

#define FAST_SPEED  36000
#define FAST_ACC    50000
#define SLOW_SPEED  6000
#define SLOW_ACC    3000

#define MAN_SPEED 1500
#define MAN_ACC 30000

#define ZERO_OFFSET 1000

#define TARGET_A 20000  // DOVE AVVIENE LO SPRINT
#define TARGET_B 33000 

FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *M1 = NULL;

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

  pinMode(ADIO6, INPUT);  // FINE CORSA

  // DRIVER OUTPUT

  pinMode(DIO11, OUTPUT);
  disableMotor();

  engine.init();

  M1 = engine.stepperConnectToPin(DIO9);

  if (M1) {
      M1->setDirectionPin(DIO10);  
      M1->setSpeedInHz(SLOW_SPEED);  
      M1->setAcceleration(SLOW_ACC);   
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

bool HOMED = false;

unsigned long t_MAN = 0, tS_MAN = 0;
bool s_MAN = false, p_MAN = true, f_MAN = false;
unsigned long t_AUTO = 0, tS_AUTO = 0;
bool s_AUTO = false, p_AUTO = true, f_AUTO = false;
bool f_REM = false;
unsigned long t_FWD = 0, tS_FWD = 0;
bool s_FWD = false, p_FWD = true, f_FWD_GO = false, f_FWD_STOP = false;
unsigned long t_BWD = 0, tS_BWD = 0;
bool s_BWD = false, p_BWD = true, f_BWD_GO = false, f_BWD_STOP = false;

bool fast_sprint = true;

void loop() {

  check_MAN();
  check_AUTO();
  
  check_STATE();

  logica();

}

void logica() {
  if (STATE == 0) {
    // REMOTO - IDLE
    delay(50);

  } else if (STATE == 1) {
    // AUTO

    if (!M1->isRunning()) {

      // MOT IN B - RETURN UP SLOW
      if (M1->getCurrentPosition() == TARGET_B) {
        M1->setSpeedInHz(SLOW_SPEED); 
        M1->setAcceleration(SLOW_ACC);
        M1->applySpeedAcceleration(); 
        M1->moveTo(0);
      }

      else if (M1->getCurrentPosition() == 0) {
        fast_sprint = true;
        M1->setSpeedInHz(SLOW_SPEED); 
        M1->setAcceleration(SLOW_ACC);
        M1->applySpeedAcceleration(); 
        M1->moveTo(TARGET_B);
      }

      else {

        goToHome();

      }

    }

    else {

      // MOT IN A - GOES TO B SLOW
      if (fast_sprint) {
        if (M1->getCurrentPosition() > TARGET_A) {
          fast_sprint = false;
          M1->setSpeedInHz(FAST_SPEED);  
          M1->setAcceleration(FAST_ACC);
          M1->applySpeedAcceleration();
        }
      }

    }

  } else if (STATE == 2) {
    // MAN

    check_FWD();
    check_BWD();

    if (f_FWD_GO) {
      f_FWD_GO = false;
      
      M1->runForward();
    }

    if (f_FWD_STOP) {
      f_FWD_STOP = false;

      M1->stopMove();
    }

    if (f_BWD_GO) {
      f_BWD_GO = false;

      M1->runBackward();
    }

    if (f_BWD_STOP) {
      f_BWD_STOP = false;

      M1->stopMove();
    }  

  }
}

void check_STATE() {

  if (f_MAN) {

    f_MAN = false;
    STATE = 2;

    enableMotor();

    M1->stopMove();
    while (M1->isRunning());

    M1->setSpeedInHz(MAN_SPEED);
    M1->setAcceleration(MAN_ACC);
    M1->applySpeedAcceleration();

  } else if ((f_AUTO) and (STATE != 2)) {

    f_AUTO = false;
    STATE = 1;

    enableMotor();

    if (!HOMED) {
      goToHome();
    }

    if (M1->isRunning()) {
      M1->stopMove();
      while (M1->isRunning()) {}
    }

    Serial.println("INIZIO LOOP");

    M1->setSpeedInHz(SLOW_SPEED); 
    M1->setAcceleration(SLOW_ACC);
    M1->applySpeedAcceleration();

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


void goToHome() {

  if (M1->isRunning()) {
    M1->stopMove();
    while (M1->isRunning()) {}
  } else {
    enableMotor();
  }

  bool exit_flag = false;

  M1->setSpeedInHz(MAN_SPEED);  
  M1->setAcceleration(MAN_ACC);
  M1->applySpeedAcceleration();
  
  M1->runBackward();

  while (!exit_flag) {

    if (!digitalRead(ADIO6)) {
      M1->forceStop();
      delay(100);
      M1->move(ZERO_OFFSET);
      while (M1->isRunning()) {}
      exit_flag = true;
    }

  }

  M1->setCurrentPosition(0);

  HOMED = true;

}

void disableMotor() {
  delay(100);  // DA TESTARE
  digitalWrite(DIO11, HIGH);
}

void enableMotor() {
  digitalWrite(DIO11, LOW);
  delay(100);  // DA TESTARE
}

