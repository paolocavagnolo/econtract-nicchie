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
  RS485.begin(9600, SERIAL_8N1, RS485_DI, RS485_RO);
  delay(100);
  pinMode(RS485_EN,OUTPUT);
  digitalWrite(RS485_EN,HIGH);

  // DRIVER OUTPUT
  pinMode(DIO11, OUTPUT);
  pinMode(DIO14, OUTPUT);
  pinMode(ADIO8, OUTPUT);

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

void loop() {


  relay_write(1,5,true);
  delay(1000);
  relay_write(1,5,false);
  delay(1000);

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

void set_address(uint8_t new_address) {

  unsigned char cmd[8]; 

  cmd[0] = 0x01;      // DEVICE ADDRESS: 0x00 BROADCAST
  cmd[1] = 0x06;      // COMMAND: 0x01: READ COIL 0x03: READ ADDRESS 0x05 WRITE COIL 0x06 SET BAUDRATE AND ADRESS 0x0F WRITE MULTIPLE
  
  cmd[2] = 0x40;      // DEVICE ADDRESS REGISTER
  cmd[3] = 0x00;      

  cmd[4] = 0x00;
  cmd[5] = new_address;

  uint16_t crc = ModbusCRC((unsigned char  *)cmd,6);
  cmd[6] = crc & 0xFF;
  cmd[7] = crc >> 8;
  
  // SEND IT
  for (uint8_t i=0; i<8; i++) {
    RS485.write(cmd[i]);
  }

}