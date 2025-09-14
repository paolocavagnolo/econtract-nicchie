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
#define M1_ACC    8000

#define M2_SPEED  8000
#define M2_ACC    8000

#define CORSA     1000

FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *M1 = NULL;
FastAccelStepper *M2 = NULL;

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
  pinMode(ADIO3, INPUT);  // finecorsa M1
  pinMode(ADIO4, INPUT);  // finecorsa M2

  // DRIVER OUTPUT
  pinMode(DIO11, OUTPUT); // M1 enable
  pinMode(DIO14, OUTPUT); // M2 enable

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

  delay(1000);

}

bool prima_volta_bobine = true;

bool prima_volta_tendipelle = true;
bool prima_uscita_tendipelle = true;

void loop() {

  // a_caso();

  // accendi_bobina();

  // logica_tendipelle();

}

void logica_tendipelle() {
  if (digitalRead(ADIO1)) {
    if (prima_volta_tendipelle) {
      prima_volta_tendipelle = false;
      prima_uscita_tendipelle = true;

      digitalWrite(DIO11, LOW);
      digitalWrite(DIO14, LOW);
      S = 0;
      M1->moveTo(CORSA);
      M2->moveTo(CORSA);
    }

    if (!M1->isRunning()) {

      if (M1->getCurrentPosition() == -CORSA) {
        M1->moveTo(CORSA);
      }

      else if (M1->getCurrentPosition() == CORSA) {
        M1->moveTo(-CORSA);
      }

      else if (M1->getCurrentPosition() == 0) {
        M1->moveTo(-CORSA);
      } 

      else {
        M1->moveTo(0);
      }

    }


    if (!M2->isRunning()) {

      if (M2->getCurrentPosition() == -CORSA) {
        M2->moveTo(CORSA);
      }

      else if (M2->getCurrentPosition() == CORSA) {
        M2->moveTo(-CORSA);
      }

      else if (M2->getCurrentPosition() == 0) {
        M2->moveTo(-CORSA);
      }

      else {
        M2->moveTo(0);
      }

    }

  } else {
    if (prima_uscita_tendipelle) {
      prima_uscita_tendipelle = false;
      prima_volta_tendipelle = true;

      M1->stopMove();
      M2->stopMove();
      while((M1->isRunning()) || (M2->isRunning())){};
      goToHome_tendipelle();

      delay(1000);

      digitalWrite(DIO11, HIGH);
      digitalWrite(DIO14, LOW);
    }
  }
}

void accendi_bobina(uint8_t n) {

  if (prima_volta) {
    prima_volta = false;
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
    delay(random(1000,4000));
    relay_write(1,nn,false);
    delay(500);
  } else {
    relay_write(2,nn-16,true);
    delay(random(1000,4000));
    relay_write(2,nn-16,false);
    delay(500);
  }

}

void a_caso() {

  if (prima_volta) {
    prima_volta = false;
    relay_write(0x00, 0xFF, false);
    delay(100);
    relay_write(0x00, 0xFF, false);
    delay(100);
    relay_write(0x00, 0xFF, false);
    delay(100);
  }

  uint8_t bobina_scelta = random(24);

  if (bobina_scelta < 16) {
    relay_write(1,bobina_scelta,true);
    delay(random(1000,4000));
    relay_write(1,bobina_scelta,false);
    delay(500);
  } else {
    relay_write(2,bobina_scelta-16,true);
    delay(random(1000,4000));
    relay_write(2,bobina_scelta-16,false);
    delay(500);
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
