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

HardwareSerial RS485(2);

#define LED_RED     38
#define LED_BLUE    37 //INPUT: OFF / OUTPUT_LOW: ON_BT / OUTPUT_HIGH: ON_WIFI

#define SDA_PIN 33
#define SCL_PIN 34

#include "FastAccelStepper.h"

#define M1_SPEED  8000
#define M1_ACC    5000

#define M2_SPEED  6000
#define M2_ACC    10000

#define CORSA     900
#define ZERO_OFFSET_TENDIPELLE 0

FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *M1 = NULL;

void setup() {

  delay(1000);

  // DEBUG
  Serial.begin(115200);
  delay(100);

  // LED OFF
  pinMode(37, INPUT);

  // RS485
  RS485.begin(9600, SERIAL_8N1, RS485_DI, RS485_RO);
  delay(100);
  pinMode(RS485_EN,OUTPUT);
  digitalWrite(RS485_EN,HIGH);

  // PANEL INPUT
  pinMode(ADIO1, INPUT);  // bobine marius
  pinMode(ADIO2, INPUT);  // tendipelle marius
  pinMode(ADIO3, INPUT);  // finecorsa M12

  // DRIVER OUTPUT
  pinMode(DIO11, OUTPUT); // M1 enable
  digitalWrite(DIO11, LOW);

  delay(1000);

  engine.init();

  M1 = engine.stepperConnectToPin(DIO9);

  if (M1) {
      M1->setDirectionPin(DIO10, false);  
      M1->setSpeedInHz(M1_SPEED);  
      M1->setAcceleration(M1_ACC); 
 
  }


  // AUDIO OUTPUT
  pinMode(DIO13, OUTPUT);   // audio bobine
  pinMode(DIO16, OUTPUT);   // audio tirapelle
  digitalWrite(DIO13, LOW);
  digitalWrite(DIO16, LOW);

  // BUZZER 
  ledcSetup(0, 1000, 8);
  ledcAttachPin(BUZZ, 0);

  // DOUBLE BEEP FOR SETUP END
  ledcWriteTone(0, 1000);
  delay(50);
  ledcWriteTone(0, 0);
  delay(175);
  ledcWriteTone(0, 1000);
  delay(50);
  ledcWriteTone(0, 0);

  delay(3000);

}

bool prima_volta_bobine = true;
bool prima_uscita_bobine = true;

bool prima_volta_tendipelle = true;
bool prima_uscita_tendipelle = true;

uint8_t bobina_scelta[3];
unsigned long tWait = 0, tBobine = 0;
bool tOn = true;
unsigned long tUscitaBobine = 0;

void loop() {

 logica_bobine_tre();
 //logica_bobine_all();

  // for (uint8_t i=1; i<25; i++) {
  //   accendi_bobina(i);
  // }
  
  logica_tendipelle();



}

bool mezza = true;

void logica_tendipelle() {

  if (digitalRead(ADIO2)) {

    if (prima_volta_tendipelle) {
      prima_volta_tendipelle = false;
      prima_uscita_tendipelle = true;

      digitalWrite(DIO11, HIGH);
      digitalWrite(DIO14, HIGH);
      digitalWrite(DIO16, HIGH);

      delay(200);

      goToHome_tendipelle();

    }

    if (M1->getCurrentPosition() == 0) {
      M1->moveTo(CORSA);
    }
    if (M1->getCurrentPosition() == CORSA) {
      M1->moveTo(0);
    }

  } else {
    if (prima_uscita_tendipelle) {
      prima_uscita_tendipelle = false;
      prima_volta_tendipelle = true;

      M1->stopMove();
      while(M1->isRunning()){};
      M1->moveTo(0);
      while(M1->isRunning()){};

      delay(100);

      digitalWrite(DIO11, LOW);

      digitalWrite(DIO16, LOW);
      digitalWrite(DIO10, LOW);
    }
  }
}

void accendi_bobina(uint8_t n) {

  if (prima_volta_bobine) {
    prima_volta_bobine = false;

    relay_write(0x00, 0xFF, false);
    delay(100);
    relay_write(0x00, 0xFF, false);
    delay(100);
    relay_write(0x00, 0xFF, false);
    delay(100);
  }

  if (n <= 0) {
    n = 1;
  } else if (n >= 24) {
    n = 24;
  } 

  uint8_t nn = n - 1;

  if (nn < 16) {
    relay_write(1,nn,true);
    delay(5000);
    relay_write(1,nn,false);
    delay(2000);
  } else {
    relay_write(2,nn-16,true);
    delay(5000);
    relay_write(2,nn-16,false);
    delay(2000);
  }

}

void logica_bobine_all() {

  if (analogRead(ADIO1) > 4000) {

    if (prima_volta_bobine) {
      Serial.println("PARTO");
      prima_volta_bobine = false;
      prima_uscita_bobine = true;
      relay_write(0x00, 0xFF, false);
      delay(300);
      relay_write(0x00, 0xFF, false);
      delay(300);
      tOn = true;
      tWait = 0;
      digitalWrite(DIO13, HIGH);
    }

    if ((millis() - tBobine) > tWait) {
      tBobine = millis();


      if (tOn) {
        tOn = false;
        tWait = (random(1000,4000));

        if (mezza) {
          for (uint8_t i=0; i<12; i++) {
            relay_write(1,i,true);
            delay(150);
          }
        } else {
          for (uint8_t i=12; i<16; i++) {
            relay_write(1,i,true);
            delay(150);
          }
          for (uint8_t i=16; i<24; i++) {
            relay_write(2,i-16,true);
            delay(150);
          }
        }
        

      } else {
        tOn = true;
        tWait = 500;
        if (mezza) {
          for (uint8_t i=0; i<12; i++) {
            relay_write(1,i,false);
            delay(150);
          }
        } else {
          for (uint8_t i=12; i<16; i++) {
            relay_write(1,i,false);
            delay(150);
          }
          for (uint8_t i=16; i<24; i++) {
            relay_write(2,i-16,false);
            delay(150);
          }
        }
        mezza = !mezza;
      }
    }
    

  } else {

    if (prima_uscita_bobine) {
      prima_uscita_bobine = false;
      prima_volta_bobine = true;

      relay_write(0x00, 0xFF, false);
      delay(300);
      relay_write(0x00, 0xFF, false);
      delay(300);
      digitalWrite(DIO13, LOW);
    }
    
  }

}

void logica_bobine_tre() {

  if (digitalRead(ADIO1)) {

    if (prima_volta_bobine) {
      Serial.println("PARTO");
      prima_volta_bobine = false;
      prima_uscita_bobine = true;
      relay_write(0x00, 0xFF, false);
      delay(100);
      relay_write(0x00, 0xFF, false);
      delay(100);
      relay_write(0x00, 0xFF, false);
      delay(100);
      tOn = true;
      tWait = 0;
      digitalWrite(15, HIGH);
    }

    if ((millis() - tBobine) > tWait) {
      tBobine = millis();


      if (tOn) {
        bobina_scelta[0] = random(24);
        bobina_scelta[1] = random(24);

        while (bobina_scelta[1] == bobina_scelta[0]) {
          bobina_scelta[1] = random(24);
        }
        
        bobina_scelta[2] = random(24);
        while ((bobina_scelta[2] == bobina_scelta[1]) || (bobina_scelta[2] == bobina_scelta[0])) {
          bobina_scelta[2] = random(24);
        }

        Serial.print( bobina_scelta[0]);
        Serial.print( bobina_scelta[1]);
        Serial.println( bobina_scelta[2]);

        tOn = false;
        tWait = (random(1000,4000));

        for (uint8_t i = 0; i<3; i++) {
          if (bobina_scelta[i] < 16) {
            relay_write(1,bobina_scelta[i],true);
          } else {
            relay_write(2,bobina_scelta[i]-16,true);
          }
          delay(200);
        }

      } else {
        tOn = true;
        tWait = 500;
        for (uint8_t i = 0; i<3; i++) {
          if (bobina_scelta[i] < 16) {
            relay_write(1,bobina_scelta[i],false);
          } else {
            relay_write(2,bobina_scelta[i]-16,false);
          }
          delay(200);
        }
      }
    }
    

  } else {

    if (prima_uscita_bobine) {
      prima_uscita_bobine = false;
      prima_volta_bobine = true;

      relay_write(0x00, 0xFF, false);
      delay(100);
      relay_write(0x00, 0xFF, false);
      delay(100);
      relay_write(0x00, 0xFF, false);
      digitalWrite(15, LOW);
    }
    
  }

}

void relay_write(uint8_t id_device, uint8_t id_relay, bool stato_relay) {

  unsigned char cmd[8]; 

  cmd[0] = id_device;      // DEVICE ADDRESS: 0x00 BROADCAST
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
    RS485.write(cmd[i]);
  }

}

void goToHome_tendipelle() {

  M1->setSpeedInHz(M1_SPEED/4);  
  M1->setAcceleration(M1_ACC/4); 
  M1->applySpeedAcceleration();
  
  M1->runBackward();
  while(!digitalRead(ADIO3)){};
  M1->forceStop();
  delay(200);
  M1->move(ZERO_OFFSET_TENDIPELLE);
  while(M1->isRunning()){};
  M1->setCurrentPosition(0);

  M1->setSpeedInHz(M1_SPEED);  
  M1->setAcceleration(M1_ACC); 
  M1->applySpeedAcceleration();

}
