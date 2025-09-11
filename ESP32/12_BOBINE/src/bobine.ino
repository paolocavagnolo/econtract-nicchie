#include "modbus_crc.h"

#define RS485_ENABLE_PIN 21

void setup()  
{
  delay(100);
  Serial.begin(9600);

  delay(100);
  Serial2.begin(9600);

  pinMode(RS485_ENABLE_PIN,OUTPUT);
  digitalWrite(RS485_ENABLE_PIN,HIGH);
}

int old_id, id, tempo, pausa;

void loop()
{   
  
  while (id == old_id) {
    id = random(4);
  }
  tempo = random(1000,5000);
  pausa = random(1000);
  
  relay_write(id,true);
  delay(tempo);
  relay_write(id,false);
  delay(pausa);

  old_id = id;
    
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
    Serial2.write(cmd[i]);
  }

}
