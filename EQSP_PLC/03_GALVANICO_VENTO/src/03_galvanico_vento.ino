// 18/08/2025
// PROBLEMA DRIVER

// DRIVER DM320T
// 01 02 03 04 05  06 
// ON ON ON ON OFF ON

// 10/09/2025 @POLENE
// MANCA TEST SU VENTO CABLATI

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

#define SLOW_SPEED  8000
#define SLOW_ACC    6000

#define MAN_SPEED 3000
#define MAN_ACC 10000

#define TARGET_POS 103000
#define BOLLE_POS 69000

#define ZERO_OFFSET 1000

FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *M1 = NULL;

uint8_t STATE_G = 0; // 0 REMOTE - 1 AUTO - 2 MAN
uint8_t STATE_V = 0; // 0 REMOTE - 1 AUTO - 2 MAN

void setup() {

  delay(1000);

  // DEBUG
  Serial.begin(115200);
  delay(100);

  // LED OFF
  pinMode(37, INPUT);

  // PANEL INPUT
  pinMode(ADIO1, INPUT);  // AUTO GALV
  pinMode(ADIO2, INPUT);  // MAN GALV
  
  pinMode(ADIO4, INPUT);  // FWD
  pinMode(ADIO5, INPUT);  // BWD

  pinMode(ADIO7, INPUT);  // AUTO VENTO
  pinMode(ADIO8, INPUT);  // MAN VENTO
  pinMode(DIO9, INPUT);  // FINECORSA MOT GALV

  // RELAY OUTPUT
  pinMode(DIO13, OUTPUT);

  // VENTO OUTPUT
  pinMode(DIO14, OUTPUT);
  pinMode(DIO15, OUTPUT);
  pinMode(DIO16, OUTPUT);

  // DRIVER OUTPUT
  pinMode(DIO12, OUTPUT);
  disableMotor();

  engine.init();

  M1 = engine.stepperConnectToPin(DIO10);

  if (M1) {
      M1->setDirectionPin(DIO11, false);  
      M1->setSpeedInHz(SLOW_SPEED);  
      M1->setAcceleration(SLOW_ACC);   
  }  

  // BUZZER 
  ledcSetup(0, 1000, 8);
  ledcAttachPin(BUZZ, 0);

  delay(1000);

  // INITIAL STATE
  if (digitalRead(ADIO2)) {
    STATE_G = 2;
  } else {
    if (digitalRead(ADIO1)) {
      STATE_G = 1;
    } else {
      STATE_G = 0;
    }
  }

  if (digitalRead(ADIO8)) {
    STATE_V = 2;
  } else {
    if (digitalRead(ADIO7)) {
      STATE_V = 1;
    } else {
      STATE_V = 0;
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

unsigned long t_MAN_G = 0, tS_MAN_G = 0, tSO_AUTO_G = 0;
bool s_MAN_G = false, p_MAN_G = true, f_MAN_G = false, pO_AUTO_G = true;

unsigned long t_MAN_V = 0, tS_MAN_V = 0;
bool s_MAN_V = false, p_MAN_V = true, f_MAN_V = false;

unsigned long t_AUTO_G = 0, tS_AUTO_G = 0;
bool s_AUTO_G = false, p_AUTO_G = true, f_AUTO_G = false;

unsigned long t_AUTO_V = 0, tS_AUTO_V = 0;
bool s_AUTO_V = false, p_AUTO_V = true, f_AUTO_V = false;

bool f_REM_G = false;
bool f_REM_V = false;

unsigned long t_FWD = 0, tS_FWD = 0;
bool s_FWD = false, p_FWD = true, f_FWD_GO = false, f_FWD_STOP = false;
unsigned long t_BWD = 0, tS_BWD = 0;
bool s_BWD = false, p_BWD = true, f_BWD_GO = false, f_BWD_STOP = false;

unsigned long tVento = 0;
bool sVento = true;
uint8_t SUB_STATE_VENTO = 0;
bool pVento = true;
unsigned long wVento = 0;

int S = 0;
bool P = true;
bool E = false;
unsigned long T = 0;

void loop() {

  check_MAN_G();
  check_MAN_V();
  check_AUTO_G();
  check_AUTO_V();
  check_FWD();
  check_BWD();

  check_STATE_G();
  check_STATE_V();

  logica_G();
  logica_V();

}

void bolleOn() {
  digitalWrite(DIO13,HIGH);
}

void bolleOff() {
  digitalWrite(DIO13,LOW);
}

void logica_G() {

  if (STATE_G == 0) {
    // REMOTO - IDLE
    delay(50);

  } else if (STATE_G == 1) {
    // AUTO
    if (S == 0) { // MI TROVO SU
      if (P) {
        P = false;
        E = false;
        T = millis();
      }

      if ((millis() - T) > 3000) { // ASPETTO 3 SECONDI E PARTO
        E = true;
        M1->moveTo(TARGET_POS);
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

      if (M1->getCurrentPosition() > BOLLE_POS) {
        bolleOn();
      }

      if (!M1->isRunning()) {
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
        M1->moveTo(0);
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

      if (M1->getCurrentPosition() < BOLLE_POS) {
        bolleOff();
      }

      if (!M1->isRunning()) {
        E = true;
      }
      
      if (E) {
        E = false;
        P = true;
        S = 0;
      }
    }


  } else if (STATE_G == 2) {
    // MAN

    if (f_FWD_GO) {
      f_FWD_GO = false;

      M1->runForward();
      bolleOn();
    }

    if (f_FWD_STOP) {
      f_FWD_STOP = false;

      M1->stopMove();
      bolleOff();
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

void logica_V() {

  if (STATE_V == 0) {
    // REMOTO - IDLE
    delay(50);


  } else if (STATE_V == 1) {
    // AUTO

    /*
    if (SUB_STATE_VENTO == 0) {
      if (pVento) {
        pVento = false;
        digitalWrite(DIO14, HIGH);
        tVento = millis();
      }

      if ((millis() - tVento) > 3000) {
        SUB_STATE_VENTO = 1;
        pVento = true;
        digitalWrite(DIO14, LOW);
      }

    } else if (SUB_STATE_VENTO == 1) {
      if (pVento) {
        pVento = false;
        digitalWrite(DIO15, HIGH);
        tVento = millis();
      }

      if ((millis() - tVento) > 200) {
        SUB_STATE_VENTO = 2;
        pVento = true;
        digitalWrite(DIO15, LOW);
      }

    } else if (SUB_STATE_VENTO == 2) {
      if (pVento) {
        pVento = false;
        digitalWrite(DIO16, HIGH);
        tVento = millis();
      }

      if ((millis() - tVento) > 200) {
        SUB_STATE_VENTO = 3;
        pVento = true;
        digitalWrite(DIO16, LOW);
      }

    } else if (SUB_STATE_VENTO == 3) {
      if (pVento) {
        pVento = false;
        digitalWrite(DIO14, LOW);
        digitalWrite(DIO15, LOW);
        digitalWrite(DIO16, LOW);
        tVento = millis();
      }

      if ((millis() - tVento) > 1500) {
        SUB_STATE_VENTO = 0;
        pVento = true;
      }
    }
    */

    if ((millis() - tVento) > wVento) {
      tVento = millis();

      digitalWrite(DIO14, sVento);
      digitalWrite(DIO15, sVento);

      sVento = !sVento;

      if (sVento) wVento = random(1600,3200); // tempo spente
      else wVento = 600; // tempo accese
    }

  } else if (STATE_V == 2) {
    // MAN

    // if ((millis() - tVento) > wVento) {
    //   tVento = millis();

    //   digitalWrite(DIO14, sVento);
    //   digitalWrite(DIO15, sVento);
    //   digitalWrite(DIO16, sVento);

    //   sVento = !sVento;

    //   if (sVento) wVento = 2000;
    //   else wVento = 5000;
    // }

    if (f_FWD_GO) {
      f_FWD_GO = false;

      digitalWrite(DIO14, HIGH);

    }

    if (f_FWD_STOP) {
      f_FWD_STOP = false;

      digitalWrite(DIO14, LOW);
    }

    if (f_BWD_GO) {
      f_BWD_GO = false;

      digitalWrite(DIO15, HIGH);
    }

    if (f_BWD_STOP) {
      f_BWD_STOP = false;

      digitalWrite(DIO15, LOW);
    }
    
  }
}

void check_STATE_G() {

  if (f_MAN_G) {

    Serial.println("MANUALE");

    f_MAN_G = false;
    STATE_G = 2;

    enableMotor();
    bolleOff();

    M1->stopMove();
    while (M1->isRunning());

    M1->setSpeedInHz(MAN_SPEED);
    M1->setAcceleration(MAN_ACC);
    M1->applySpeedAcceleration();

  } else if ((f_AUTO_G) and (STATE_G != 2)) {

    f_AUTO_G = false;
    STATE_G = 1;

    Serial.println("AUTO");

    enableMotor();

    if (!HOMED) {
      goToHome();
    }

    if (M1->isRunning()) {
      M1->stopMove();
      while (M1->isRunning()) {}
    }

    M1->setSpeedInHz(SLOW_SPEED); 
    M1->setAcceleration(SLOW_ACC);
    M1->applySpeedAcceleration();

    Serial.println("INIZIO LOOP GALV");
    S = 1;
    P = true;
    M1->moveTo(TARGET_POS);

  } else if (f_REM_G) {
    f_REM_G = false;

    Serial.println("REMOTO");

    if (digitalRead(ADIO1)) {

      STATE_G = 1;
      f_AUTO_G = true;

    } else {

      STATE_G = 0;

      goToHome();
      bolleOff();
      disableMotor();

    }
    
  }

}

void check_STATE_V() {

  if (f_MAN_V) {

    f_MAN_V = false;
    STATE_V = 2;
    sVento = true;
    wVento = 0;

  } else if ((f_AUTO_V) and (STATE_V != 2)) {

    f_AUTO_V = false;
    STATE_V = 1;
    Serial.println("INIZIO LOOP VENTO");
    SUB_STATE_VENTO = 0;
    wVento = 0;
    pVento = true;

  } else if (f_REM_V) {
    f_REM_V = false;

    if (digitalRead(ADIO7)) {

      STATE_V = 1;
      f_AUTO_V = true;

    } else {

      STATE_V = 0;
      digitalWrite(DIO14, LOW);
      digitalWrite(DIO15, LOW);
      digitalWrite(DIO16, LOW);

    }
    

  }

}

void check_MAN_G() {
  if (!s_MAN_G) {
    if ((millis() - t_MAN_G) > 200) {
      if (digitalRead(ADIO2)) {
        if (p_MAN_G) {
          p_MAN_G = false;
          tS_MAN_G = millis();
        }
        if ((millis() - tS_MAN_G) > 200) {
          t_MAN_G = millis();
          s_MAN_G = true;
          
          // PREMUTO DOPO 200 ms 
          Serial.println("PUSHED BTN_MAN_G");

          ledcWriteTone(0, 1000);
          delay(50);
          ledcWriteTone(0, 0);
          f_MAN_G = true;

        }
      } else {
        tS_MAN_G = millis();
      }
    }
  } else {
    if ((millis() - t_MAN_G) > 200) {
      if (!digitalRead(ADIO2)) {
        t_MAN_G = millis();
        s_MAN_G = false;
        p_MAN_G = true;

        // RILASCIATO DOPO 200 ms
        Serial.println("RELEASED BTN_MAN_G");
        f_REM_G = true;
        ledcWriteTone(0, 1000);
        delay(50);
        ledcWriteTone(0, 0);
        
      }
    }
  }
}

void check_MAN_V() {
  if (!s_MAN_V) {
    if ((millis() - t_MAN_V) > 200) {
      if (digitalRead(ADIO8)) {
        if (p_MAN_V) {
          p_MAN_V = false;
          tS_MAN_V = millis();
        }
        if ((millis() - tS_MAN_V) > 200) {
          t_MAN_V = millis();
          s_MAN_V = true;
          
          // PREMUTO DOPO 200 ms 
          Serial.println("PUSHED BTN_MAN_V");

          ledcWriteTone(0, 1000);
          delay(50);
          ledcWriteTone(0, 0);
          f_MAN_V = true;

        }
      } else {
        tS_MAN_V = millis();
      }
    }
  } else {
    if ((millis() - t_MAN_V) > 200) {
      if (!digitalRead(ADIO8)) {
        t_MAN_V = millis();
        s_MAN_V = false;
        p_MAN_V = true;

        // RILASCIATO DOPO 200 ms
        Serial.println("RELEASED BTN_MAN_V");
        f_REM_V = true;
        ledcWriteTone(0, 1000);
        delay(50);
        ledcWriteTone(0, 0);
        
      }
    }
  }
}

void check_AUTO_G() {
  if (!s_AUTO_G) {
    if ((millis() - t_AUTO_G) > 200) {
      if (digitalRead(ADIO1)) {
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

          ledcWriteTone(0, 1000);
          delay(50);
          ledcWriteTone(0, 0);
          f_AUTO_G = true;
        }
      } else {
        tS_AUTO_G = millis();
      }
    }
  } else {
    if ((millis() - t_AUTO_G) > 200) {
      if (!digitalRead(ADIO1)) {
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
          ledcWriteTone(0, 1000);
          delay(50);
          ledcWriteTone(0, 0);
        }
      } else {
        tSO_AUTO_G = millis();
      }
    }
  }
}

void check_AUTO_V() {
  if (!s_AUTO_V) {
    if ((millis() - t_AUTO_V) > 200) {
      if (digitalRead(ADIO7)) {
        if (p_AUTO_V) {
          p_AUTO_V = false;
          tS_AUTO_V = millis();
        }
        if ((millis() - tS_AUTO_V) > 200) {
          t_AUTO_V = millis();
          s_AUTO_V = true;
          
          // PREMUTO DOPO 200 ms 
          Serial.println("PUSHED BTN_AUTO_V");

          ledcWriteTone(0, 1000);
          delay(50);
          ledcWriteTone(0, 0);
          f_AUTO_V = true;
        }
      } else {
        tS_AUTO_V = millis();
      }
    }
  } else {
    if ((millis() - t_AUTO_V) > 200) {
      if (!digitalRead(ADIO7)) {
        t_AUTO_V = millis();
        s_AUTO_V = false;
        p_AUTO_V = true;

        // RILASCIATO DOPO 200 ms
        Serial.println("RELEASED BTN_AUTO_V");
        f_REM_V = true;
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
    while (M1->isRunning()) {};
  }

  bool exit_flag = false;

  M1->setSpeedInHz(MAN_SPEED);  
  M1->setAcceleration(MAN_ACC);
  M1->applySpeedAcceleration();
  
  M1->runBackward();

  while (!exit_flag) {

    if (digitalRead(DIO9)) {
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
  digitalWrite(DIO12, LOW);
}

void enableMotor() {
  digitalWrite(DIO12, HIGH);
  delay(100);  // DA TESTARE
}

