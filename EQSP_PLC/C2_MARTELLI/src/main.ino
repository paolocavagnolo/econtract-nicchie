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

#include "modbus_crc.h"

#define RS485_DI    18
#define RS485_RO    17 
#define RS485_EN    35

#define LED_RED     38
#define LED_BLUE    37 //INPUT: OFF / OUTPUT_LOW: ON_BT / OUTPUT_HIGH: ON_WIFI

#define SDA_PIN 33
#define SCL_PIN 34

#include "FastAccelStepper.h"

#define M1_SPEED  8000
#define M1_ACC    8000

#define M2_SPEED  8000
#define M2_ACC    8000

#define M3_SPEED  8000
#define M3_ACC    8000

#define MAN_SPEED 2000
#define MAN_ACC 2000

#define ZERO_OFFSET 10

#define CORSA_M1 200
#define CORSA_M2 200
#define CORSA_M3 200

FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *M1 = NULL;
FastAccelStepper *M2 = NULL;
FastAccelStepper *M3 = NULL;

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

  pinMode(ADIO3, INPUT);  // FWD
  pinMode(ADIO4, INPUT);  // BWD

  pinMode(ADIO5, INPUT);  // FINE CORSA M1
  pinMode(ADIO6, INPUT);  // FINE CORSA M2
  pinMode(ADIO7, INPUT);  // FINE CORSA M3

  // RS485
  pinMode(RS485_EN,OUTPUT);
  digitalWrite(RS485_EN,HIGH);

  // DRIVER OUTPUT
  pinMode(DIO11, OUTPUT);
  pinMode(DIO14, OUTPUT);
  pinMode(ADIO8, OUTPUT);
  disableMotor();

  engine.init();

  M1 = engine.stepperConnectToPin(DIO9);

  if (M1) {
      M1->setDirectionPin(DIO10);  
      M1->setSpeedInHz(M1_SPEED);  
      M1->setAcceleration(M1_ACC);   
  }

  M2 = engine.stepperConnectToPin(DIO12);

  if (M2) {
      M2->setDirectionPin(DIO13);  
      M2->setSpeedInHz(M2_SPEED);  
      M2->setAcceleration(M2_ACC);   
  } 

  M3 = engine.stepperConnectToPin(DIO15);

  if (M3) {
      M3->setDirectionPin(DIO16);  
      M3->setSpeedInHz(M3_SPEED);  
      M3->setAcceleration(M3_ACC);   
  } 

  // BUZZER 
  ledcSetup(0, 1000, 8);
  ledcAttachPin(BUZZ, 0);

  // CANNOT START IN AUTO MODE
  while (digitalRead(ADIO1)) {
    ledcWriteTone(0, 1000);
    delay(50);
    ledcWriteTone(0, 0);
    delay(175);
  }

  delay(1000);

  // INITIAL STATE
  if (digitalRead(ADIO1)) {
    STATE = 1;
  } else {
    if (digitalRead(ADIO2)) {
      STATE = 2;
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

bool HOMED = false;

unsigned long T = 0;
bool P = true;
bool manState = true;
unsigned long manWait = 0;

void loop() {

  check_MAN();
  check_AUTO();
  check_FWD();
  check_BWD();

  check_STATE();

  logica();

}

#define NUM_BOBINE 24
bool s_B[NUM_BOBINE]{false};
unsigned long t_B[NUM_BOBINE]{0};
unsigned long w_B[NUM_BOBINE]{0};
bool f_B[NUM_BOBINE]{false};

void logica() {
  if (STATE == 0) {
    // REMOTO - IDLEb
    delay(50);

  } else if (STATE == 1) {
    // AUTO

    // BOBINE
    for (uint8_t i=0; i<NUM_BOBINE; i++) {
      if (!s_B[i]) {
        if ((millis() - t_B[i]) > w_B[i]) {
          t_B[i] = millis();
          s_B[i] = true;
          w_B[i] = random(2000,10000); // TEMPO ACCESO
          relay_write(i,true);
        }
      } else {
        if ((millis() - t_B[i]) > w_B[i]) {
          t_B[i] = millis();
          s_B[i] = false;
          w_B[i] = random(5000,30000); // TEMPO SPENTO
          relay_write(i,false);
        }
      }
    }

    // TIRA_PELLE
    if (!M1->isRunning()) {

      if (M1->getCurrentPosition() == CORSA_M1) {
        M1->moveTo(0);
      }

      if (M1->getCurrentPosition() == 0) {
        M1->moveTo(CORSA_M1);
      }

    }

    if (!M2->isRunning()) {

      if (M2->getCurrentPosition() == CORSA_M2) {
        M2->moveTo(0);
      }

      if (M2->getCurrentPosition() == 0) {
        M2->moveTo(CORSA_M2);
      }

    }

  } else if (STATE == 2) {
    // MAN

    // BOBINE
    if ((millis() - T) > manWait) {
      T = millis();

      relay_write(0xFF, manState);
      manState = !manState;

      if (manState) {
        manWait = 5000;
      } else {
        manWait = 2000;
      }

    }

    // TIRAPELLE
    if (f_FWD_GO) {
      f_FWD_GO = false;
      
      M1->runForward();
      M2->runForward();
    }

    if (f_FWD_STOP) {
      f_FWD_STOP = false;

      M1->stopMove();
      M2->stopMove();
    }

    if (f_BWD_GO) {
      f_BWD_GO = false;

      M1->runBackward();
      M2->runBackward();
    }

    if (f_BWD_STOP) {
      f_BWD_STOP = false;

      M1->stopMove();
      M2->stopMove();
    }  

  }
}

void check_STATE() {

  if (f_MAN) {

    f_MAN = false;
    STATE = 2;
    relay_write(0xFF,true);
    manState = true;
    manWait = 5000;

    enableMotor();

    M1->setSpeedInHz(MAN_SPEED);  
    M1->setAcceleration(MAN_ACC);
    M1->applySpeedAcceleration();

    M2->setSpeedInHz(MAN_SPEED);  
    M2->setAcceleration(MAN_ACC);
    M2->applySpeedAcceleration();

  } else if (f_AUTO) {

    f_AUTO = false;
    STATE = 1;
    relay_write(0xFF,false);

    for (uint8_t i=0; i<NUM_BOBINE; i++) {
      s_B[i] = false;
      w_B[i] = random(15000); // TEMPO SPENTO
      t_B[i] = millis();
    }

    if (!HOMED) {
      goToHome();
    }

    if ((M1->isRunning()) || (M2->isRunning())) {
      M1->stopMove();
      M2->stopMove();

      while ((M1->isRunning()) || (M2->isRunning())) {}
    }

    enableMotor();

    Serial.println("INIZIO LOOP");

    M1->setSpeedInHz(M1_SPEED); 
    M1->setAcceleration(M1_ACC);
    M1->applySpeedAcceleration();

    M2->setSpeedInHz(M2_SPEED); 
    M2->setAcceleration(M2_ACC);
    M2->applySpeedAcceleration();


  } else if (f_REM) {

    f_REM = false;
    STATE = 0;
    relay_write(0xFF,false);

    goToHome();

  }

}

void relay_write(uint8_t id_relay, bool stato_relay) {

  unsigned char cmd[8]; 

  cmd[0] = 0x01;      // DEVICE ADDRESS: 0x00 BROADCAST
  cmd[1] = 0x05;      // COMMAND: 0x01: READ COIL 0x03: READ ADDRESS 0x05 WRITE COIL 0x06 SET BAUDRATE AND ADRESS 0x0F WRITE MULTIPLE
  cmd[2] = 0x00;      // RELAY ADDRESS MSB:
  cmd[3] = id_relay;  // RELAY ADDRESS LSB:

  if (!stato_relay) {
    cmd[4] = 0x00;  // STATO RELAY (0x0 oppure 0xFF)
    cmd[5] = 0x00;
  }
  else {
    cmd[4] = 0xFF;
    cmd[5] = 0x00;
  }

  uint16_t crc = ModbusCRC((unsigned char  *)cmd,6);
  cmd[6] = crc & 0xFF;
  cmd[7] = crc >> 8;
  
  // SEND IT
  for (uint8_t i=0; i<8; i++) {
    Serial2.write(cmd[i]);
  }

}

void check_MAN() {
  if (!s_MAN) {
    if ((millis() - t_MAN) > 200) {
      if (!digitalRead(ADIO2)) {
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
      if (digitalRead(ADIO2)) {
        t_MAN = millis();
        s_MAN = false;
        p_MAN = true;

        // RILASCIATO DOPO 200 ms
        Serial.println("RELEASED BTN_MAN");
        f_REM = true;
        
      }
    }
  }
}

void check_AUTO() {
  if (!s_AUTO) {
    if ((millis() - t_AUTO) > 200) {
      if (!digitalRead(ADIO1)) {
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
      if (digitalRead(ADIO1)) {
        t_AUTO = millis();
        s_AUTO = false;
        p_AUTO = true;

        // RILASCIATO DOPO 200 ms
        Serial.println("RELEASED BTN_AUTO");
        f_REM = true;
      }
    }
  }
}

void check_FWD() {
  if (!s_FWD) {
    if ((millis() - t_FWD) > 200) {
      if (!digitalRead(ADIO4)) {
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
      if (digitalRead(ADIO4)) {
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
      if (!digitalRead(ADIO5)) {
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
      if (digitalRead(ADIO5)) {
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

  if ((M1->isRunning()) || (M2->isRunning()) || (M3->isRunning())) {
    M1->stopMove();
    M2->stopMove();
    M3->stopMove();
    while ((M1->isRunning()) || (M2->isRunning()) || (M3->isRunning())) {}
  } else {
    enableMotor();
  }

  bool exit_flag_m1 = false;
  bool exit_flag_m2 = false;
  bool exit_flag_m3 = false;

  M1->setSpeedInHz(MAN_SPEED);  
  M1->setAcceleration(MAN_ACC);
  M1->applySpeedAcceleration();

  M2->setSpeedInHz(MAN_SPEED);  
  M2->setAcceleration(MAN_ACC);
  M2->applySpeedAcceleration();

  M3->setSpeedInHz(MAN_SPEED);  
  M3->setAcceleration(MAN_ACC);
  M3->applySpeedAcceleration();

  M1->runBackward();
  M2->runBackward();
  M3->runBackward();

  while ((!exit_flag_m1) || (!exit_flag_m2) || (!exit_flag_m3)) {

    if (digitalRead(ADIO5)) {
      M1->forceStop();
      exit_flag_m1 = true;
    }

    if (digitalRead(ADIO6)) {
      M2->forceStop();
      exit_flag_m2 = true;
    }

    if (digitalRead(ADIO7)) {
      M3->forceStop();
      exit_flag_m3 = true;
    }

  }

  delay(100);

  M1->move(ZERO_OFFSET);
  M2->move(ZERO_OFFSET);
  M3->move(ZERO_OFFSET);

  while ((M1->isRunning()) || (M2->isRunning()) || (M3->isRunning())) {}

  M1->setCurrentPosition(0);
  M2->setCurrentPosition(0);
  M3->setCurrentPosition(0);

  disableMotor();

  HOMED = true;

}

void disableMotor() {
  delay(10);  // DA TESTARE
  digitalWrite(DIO11, HIGH);
  digitalWrite(DIO14, HIGH);
  digitalWrite(ADIO8, HIGH);
}

void enableMotor() {
  digitalWrite(DIO11, LOW);
  digitalWrite(DIO14, LOW);
  digitalWrite(ADIO8, LOW);
  delay(10);  // DA TESTARE
}