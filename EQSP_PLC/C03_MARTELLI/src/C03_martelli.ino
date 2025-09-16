// DRIVER DM542T
// 01 02 03 04  05  06  07 08 
// ON OFF ON OFF OFF OFF ON ON 

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

#define M1_SPEED  1800
#define M1_ACC    1200

#define M2_SPEED  1800
#define M2_ACC    1200

#define M3_SPEED  1800
#define M3_ACC    1200

#define ZERO_OFFSET 50

#define CORSA_M1 1700
#define CORSA_M2 1700
#define CORSA_M3 1700

FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *M1 = NULL;
FastAccelStepper *M2 = NULL;
FastAccelStepper *M3 = NULL;

uint8_t STATE = 0; // 0 REMOTE - 1 AUTO - 2 MAN

#define ATTESA_ATTUATORE 5000

void setup() {

  delay(1000);

  // DEBUG
  Serial.begin(115200);
  delay(100);

  // LED OFF
  pinMode(37, INPUT);

  // PANEL INPUT
  pinMode(ADIO1, INPUT);  // AVVIO REMOTO
  pinMode(ADIO2, INPUT);  // FINECORSA MOT 1
  pinMode(ADIO3, INPUT);  // FINECORSA MOT 2
  pinMode(ADIO4, INPUT);  // FINECORSA MOT 3

  // RS485
  RS485.begin(9600, SERIAL_8N1, RS485_DI, RS485_RO);
  delay(100);
  pinMode(RS485_EN,OUTPUT);
  digitalWrite(RS485_EN,HIGH);

  // DRIVER OUTPUT
  pinMode(DIO15, OUTPUT);
  digitalWrite(DIO15, HIGH);

  pinMode(DIO16, OUTPUT);
  digitalWrite(DIO16, LOW);

  engine.init();

  M1 = engine.stepperConnectToPin(DIO9);

  if (M1) {
      M1->setDirectionPin(DIO10);  
      M1->setSpeedInHz(M1_SPEED);  
      M1->setAcceleration(M1_ACC);  

  }

  M2 = engine.stepperConnectToPin(DIO11);

  if (M2) {
      M2->setDirectionPin(DIO12);  
      M2->setSpeedInHz(M2_SPEED);  
      M2->setAcceleration(M2_ACC);   
  } 

  M3 = engine.stepperConnectToPin(DIO13);

  if (M3) {
      M3->setDirectionPin(DIO14);  
      M3->setSpeedInHz(M3_SPEED);  
      M3->setAcceleration(M3_ACC);   
  } 

  //goToHome();

  // BUZZER 
  ledcSetup(0, 1000, 8);
  ledcAttachPin(BUZZ, 0);

  delay(1000);

  // DOUBLE BEEP FOR SETUP END
  ledcWriteTone(0, 1000);
  delay(50);
  ledcWriteTone(0, 0);
  delay(175);
  ledcWriteTone(0, 1000);
  delay(50);
  ledcWriteTone(0, 0);

  delay(1000);

  spegni_tutti_relay();

}

void spegni_tutti_relay() {
  relay_single_write(0x00, 0xFF, false);
  delay(100);
  relay_single_write(0x00, 0xFF, false);
  delay(100);
  relay_single_write(0x00, 0xFF, false);
  delay(100);
}

// WAVESHARE RELAY:
// M1: 0 - 15 : 0 - 15
// M2: 0 - 15 : 16 - 31
// M3: 0 - 15 : 32 - 47
// M4: 0 - 15 : 48 - 63
// M5: 0 - 15 : 64 - 79

// ASTA 1: 26 ch : 13 martelli : 0 - 12
// ASTA 2: 26 ch : 13 martelli : 13 - 25
// ASTA 3: 26 ch : 13 martelli : 26 - 38


#define NUM_MARTELLI 39

uint8_t martello_scelto;
uint8_t motore_scelto = 0, old_motore_scelto = 0;
uint8_t canale_scelto = 0, old_canale_scelto = 0;

uint8_t n_mov[] = {0, 1, 2};
uint8_t n_mar[] = {0, 0, 0};

bool prima_volta = true, prima_spento = true;

void loop() {

  //test_motore_1();
  // test_motore_2();
  // test_motore_3();

  // test_con_finecorsa_1();
  // test_con_finecorsa_2();
  // test_con_finecorsa_3();

  // test_motorino(1);

  //test_alzata(1);

  // sequenza_3_ottimizzata();

  // sequenza_con_marius();

  // relay_single_write(1,1,true);
  // delay(2000);
  // relay_single_write(1,1,false);
  // delay(2000);

  if (digitalRead(ADIO1)) {
    if (prima_volta) {
      prima_volta = false;
      prima_spento = true;
      digitalWrite(DIO15, LOW);
      digitalWrite(DIO16, HIGH);

    }

    test_alzata_in3();

  } else {
    if (prima_spento) {
      prima_spento = false;
      prima_volta = true;
      digitalWrite(DIO15, HIGH);
      digitalWrite(DIO16, LOW);
    }
  }

  

}

int S = -1;
unsigned long T = 0;
bool P = true, E = false;
int C = 0;
uint8_t MS[3]{0};
int AS = -1, old_AS = -1;

void test_alzata_in3() {

  // AZZERA
  if (S == -1) {

    // spegni tutti i relay e aspetta 5 secondi
    Serial.println("SPENGO TUTTO");
    relay_single_write(0x00, 0xFF, false);
    Serial.println("GO TO HOME AND WAIT 5 SEC");
    goToHome();
    delay(ATTESA_ATTUATORE);

    S = 0;
    P = true;
  }

  // INSERISCI
  else if (S == 0) {
    if (P) {
      P = false;
      Serial.println("INSERT AND WAIT");

      AS = random(3);
      while (AS == old_AS) {
        AS = random(3);
      }
      old_AS = AS;
      Serial.print("ASSE ");
      Serial.println(AS);

      if (AS == 2) {
        for (uint8_t j=0; j<3; j++) {
          MS[j] = random(13);
          while ((MS[j] == 0) || (MS[j] == 3) || (MS[j] == 6) || (MS[j] == 7)) {
            MS[j] = random(13);
          }
        }
      } else {
        MS[0] = random(13);
        MS[1] = random(13);
        MS[2] = random(13);
      }

      Serial.print("MARTELLI NUMERO: ");

      for (uint8_t i=0; i<3; i++) {
        uint8_t ms = MS[i] + AS*13;
        Serial.print(ms);
        Serial.print(" ");
        relay_double_write(modulo_ws(ms), numero_relay(ms+1), true);
        delay(100);
      }
      Serial.println();

      T = millis();
      
    }

    if ((millis() - T) > ATTESA_ATTUATORE) {
      S = 1;
      P = true;
    }

  }

  // SALI SCENDI
  else if (S == 1) {
    if (P) {
      P = false;
      C = 0;
    }

    if (C < 3) {
      if (AS == 0) {
        M1->move(CORSA_M1);
        while(M1->isRunning()){};
        M1->move(-CORSA_M1);
        while(M1->isRunning()){};

      } else if (AS == 1) {
        M2->move(CORSA_M2);
        while(M2->isRunning()){};
        M2->move(-CORSA_M2);
        while(M2->isRunning()){};

      } else if (AS == 2) {
        M3->move(CORSA_M3);
        while(M3->isRunning()){};
        M3->move(-CORSA_M3);
        while(M3->isRunning()){};

      }
      
      C++;
    } else {
      S = 2;
      P = true;
    }

  }

  // TOGLI PERNI
  else if (S == 2) {
    if (P) {
      P = false;
      for (uint8_t i=0; i<3; i++) {
        uint8_t ms = MS[i] + AS*13;
        relay_double_write(modulo_ws(ms), numero_relay(ms+1), false);
        delay(100);
      }
      T = millis();
    }

    S = 0;
    P = true;

  }


}

void test_alzata(uint8_t n) {

  if (prima_volta) {
    prima_volta = false;

    // spegni tutti i relay e aspetta 5 secondi
    Serial.println("SPENGO TUTTO");
    relay_single_write(0x00, 0xFF, false);
    Serial.println("aspetto 5 sec.");
    delay(ATTESA_ATTUATORE);
  }

  uint8_t nn;

  if (n <=0) {
    n = 1;
  } else if (n >= 40) {
    n = 39;
  } 

  nn = n - 1;

  martello_scelto = nn;
  Serial.print("SCELGO MARTELLO: ");
  Serial.println(martello_scelto);

  Serial.print("MODULO: ");
  Serial.print(modulo_ws(martello_scelto));
  Serial.print(" - NUMERO: ");
  Serial.println(numero_relay(martello_scelto));

  // spingi attuatore lineare
  Serial.println("SPINGO ATTUATORE LINEARE");
  relay_double_write(modulo_ws(martello_scelto), numero_relay(martello_scelto+1), true);

  // aspetto 5 secondi
  Serial.println("aspetto 5 sec");
  delay(ATTESA_ATTUATORE);

  if (martello_scelto < 13) {
  // se 0 - 12 allora M1

    Serial.println("ALZO BARRA 1");
    M1->move(CORSA_M1);
    while(M1->isRunning()){};

    Serial.println("ABBASSO BARRA 1");
    M1->move(-CORSA_M1);
    while(M1->isRunning()){};
    
  } else if (martello_scelto < 26) {
  // se 13 - 25 allora M2

    Serial.println("ALZO BARRA 2");
    M2->move(CORSA_M2);
    while(M2->isRunning()){};

    Serial.println("ABBASSO BARRA 2");
    M2->move(-CORSA_M2);
    while(M2->isRunning()){};

  } else if (martello_scelto < 39) {
  // se 26 - 38 allora M3
    Serial.println("ALZO BARRA 3");
    M3->move(CORSA_M3);
    while(M3->isRunning()){};
    
    Serial.println("ABBASSO BARRA 3");
    M3->move(-CORSA_M3);
    while(M3->isRunning()){};
  }

  // ritorno attuatore lineare
  Serial.println("RITORNO ATTUATORE LINEARE");
  relay_double_write(modulo_ws(martello_scelto), numero_relay(martello_scelto+1), false);

  // aspetto 5 secondi
  Serial.println("aspetto 5 sec");
  delay(ATTESA_ATTUATORE);

  while (1);

}

void test_motorino(uint8_t n) {

  if (prima_volta) {
    prima_volta = false;

    // spegni tutti i relay e aspetta 5 secondi
    Serial.println("SPENGO TUTTO");
    relay_single_write(0x00, 0xFF, false);
    Serial.println("aspetto 5 sec.");
    delay(ATTESA_ATTUATORE);
  }

  uint8_t nn;

  if (n <=0) {
    n = 1;
  } else if (n >= 40) {
    n = 39;
  } 

  nn = n + 1;

  relay_double_write(modulo_ws(nn), numero_relay(nn), true);
  Serial.print("acceso ");
  Serial.println(n);
  delay(5000);
  relay_double_write(modulo_ws(nn), numero_relay(nn), false);
  Serial.print("spento ");
  Serial.println(n);
  delay(6000);
}

void test_motore_1() {
  M1->move(CORSA_M1);
  while(M1->isRunning()){};
  M1->move(-CORSA_M1);
  while(M1->isRunning()){};

}

void test_motore_2() {
  if (!M2->isRunning()) {
    if (M2->getCurrentPosition() == 0) {
      M2->moveTo(CORSA_M2);
    } else if (M2->getCurrentPosition() == CORSA_M2) {
      M2->moveTo(0);
    } else {
      M2->moveTo(0);
    }
  }
}

void test_motore_3() {
  if (!M3->isRunning()) {
    if (M3->getCurrentPosition() == 0) {
      M3->moveTo(CORSA_M3);
    } else if (M3->getCurrentPosition() == CORSA_M3) {
      M3->moveTo(0);
    } else {
      M3->moveTo(0);
    }
  }
}

void test_con_finecorsa_1() {
  if (!M1->isRunning()) {
    if (M1->getCurrentPosition() == 0) {
      M1->move(CORSA_M1);
    } else if (M1->getCurrentPosition() == CORSA_M1) {
      M1->runBackward();
      while (!digitalRead(ADIO2)){};
      M1->forceStop();
      delay(100);
      M1->setCurrentPosition(0);
      M1->moveTo(CORSA_M1);
    } else {
      M1->runBackward();
      while (!digitalRead(ADIO2)){};
      M1->forceStop();
      delay(100);
      M1->setCurrentPosition(0);
      M1->moveTo(CORSA_M1);
    }
  }
}

void test_con_finecorsa_2() {
  if (!M2->isRunning()) {
    if (M2->getCurrentPosition() == 0) {
      M2->move(CORSA_M2);
    } else if (M2->getCurrentPosition() == CORSA_M2) {
      M2->runBackward();
      while (!digitalRead(ADIO3)){};
      M2->forceStop();
      delay(100);
      M2->setCurrentPosition(0);
      M2->moveTo(CORSA_M2);
    } else {
      M2->runBackward();
      while (!digitalRead(ADIO3)){};
      M2->forceStop();
      delay(100);
      M2->setCurrentPosition(0);
      M2->moveTo(CORSA_M2);
    }
  }
}

void test_con_finecorsa_3() {
  if (!M3->isRunning()) {
    if (M3->getCurrentPosition() == 0) {
      M3->move(CORSA_M3);
    } else if (M3->getCurrentPosition() == CORSA_M3) {
      M3->runBackward();
      while (!digitalRead(ADIO4)){};
      M3->forceStop();
      delay(100);
      M3->setCurrentPosition(0);
      M3->moveTo(CORSA_M3);
    } else {
      M3->runBackward();
      while (!digitalRead(ADIO4)){};
      M3->forceStop();
      delay(100);
      M3->setCurrentPosition(0);
      M3->moveTo(CORSA_M3);
    }
  }
}

void sequenza_con_marius() {

  bool go = digitalRead(ADIO1);

  if (go) {

    digitalWrite(DIO15, LOW);

    if (prima_volta) {
      prima_volta = false;
      prima_spento = true;

      // spegni tutti i relay e aspetta 5 secondi
      Serial.println("SPENGO TUTTO");
      relay_single_write(0x00, 0xFF, false);
      Serial.println("aspetto 5 sec.");
      delay(ATTESA_ATTUATORE);

      // spara 1 batacchio su ogni motore
      Serial.println("SPARO 1 SU OGNI MOTORE");
      n_mar[0] = random(13);
      n_mar[1] = random(13);
      n_mar[2] = random(13);

      for (uint8_t i=0; i<3; i++) {
        martello_scelto = n_mar[i] + i*13;
        relay_double_write(modulo_ws(martello_scelto), numero_relay(martello_scelto), true);
        delay(100);
      }

      Serial.println("aspetto 5 sec.");
      delay(ATTESA_ATTUATORE);
    }

    // MUOVO IL PRIMO ELEMENTO DELLA LISTA n_mov 
    // E 
    // CAMBIO IL BATACCHIO DELL'ULTIMO !
    Serial.print("MUOVO ASTA NUM: ");
    Serial.println(n_mov[0]);

    Serial.print("e cambio batacchio dell'ultimo: da : ");
    martello_scelto = n_mar[n_mov[2]] + n_mov[2]*13;
    Serial.print(martello_scelto);
    relay_double_write(modulo_ws(martello_scelto), numero_relay(martello_scelto), false);
    delay(100);

    n_mar[n_mov[2]] = random(13);

    Serial.print(" a : ");
    martello_scelto = n_mar[n_mov[2]] + n_mov[2]*13;
    Serial.println(martello_scelto);
    relay_double_write(modulo_ws(martello_scelto), numero_relay(martello_scelto), true);

    if (n_mov[0] == 0) {
    // se 0 - 12 allora M1

      Serial.println("ALZO BARRA 1");
      M1->moveTo(CORSA_M1);
      while(M1->isRunning()){};

      Serial.println("aspetto 1 secondo");
      delay(1000);

      Serial.println("ABBASSO BARRA 1");
      M1->moveTo(0);
      while(M1->isRunning()){};
      
    } else if (n_mov[0] == 1) {
    // se 13 - 25 allora M2

      Serial.println("ALZO BARRA 2");
      M2->moveTo(CORSA_M2);
      while(M2->isRunning()){};

      Serial.println("aspetto 1 secondo");
      delay(1000);

      Serial.println("ABBASSO BARRA 2");
      M2->moveTo(0);
      while(M2->isRunning()){};

    } else if (n_mov[0] == 2) {
    // se 26 - 38 allora M3
      Serial.println("ALZO BARRA 3");
      M3->moveTo(CORSA_M3);
      while(M3->isRunning()){};

      Serial.println("aspetto 1 secondo");
      delay(1000);
      
      Serial.println("ABBASSO BARRA 3");
      M3->moveTo(0);
      while(M3->isRunning()){};
    }

    n_mov[0] = n_mov[1];
    n_mov[1] = n_mov[2];
    
    while (n_mov[2] == n_mov[1]) {
      n_mov[2] = random(3);
    }

  } else {

    if (prima_spento) {
      prima_spento = false;
      
      goToHome();

      Serial.println("SPENGO TUTTO");
      relay_single_write(0x00, 0xFF, false);
      Serial.println("aspetto 5 sec.");
      delay(ATTESA_ATTUATORE);

      digitalWrite(DIO15, HIGH);
      delay(100);
    }
    

  }


}



void sequenza_3_ottimizzata() {

  if (prima_volta) {
    prima_volta = false;

    // spegni tutti i relay e aspetta 5 secondi
    Serial.println("SPENGO TUTTO");
    relay_single_write(0x00, 0xFF, false);
    Serial.println("aspetto 5 sec.");
    delay(ATTESA_ATTUATORE);

    // spara 1 batacchio su ogni motore
    Serial.println("SPARO 1 SU OGNI MOTORE");
    n_mar[0] = random(13);
    n_mar[1] = random(13);
    n_mar[2] = random(13);

    for (uint8_t i=0; i<3; i++) {
      martello_scelto = n_mar[i] + i*13;
      relay_double_write(modulo_ws(martello_scelto), numero_relay(martello_scelto), true);
      delay(100);
    }

    Serial.println("aspetto 5 sec.");
    delay(ATTESA_ATTUATORE);
  }

  // MUOVO IL PRIMO ELEMENTO DELLA LISTA n_mov 
  // E 
  // CAMBIO IL BATACCHIO DELL'ULTIMO !
  Serial.print("MUOVO ASTA NUM: ");
  Serial.println(n_mov[0]);

  Serial.print("e cambio batacchio dell'ultimo: da : ");
  martello_scelto = n_mar[n_mov[2]] + n_mov[2]*13;
  Serial.print(martello_scelto);
  relay_double_write(modulo_ws(martello_scelto), numero_relay(martello_scelto), false);
  delay(100);

  n_mar[n_mov[2]] = random(13);

  Serial.print(" a : ");
  martello_scelto = n_mar[n_mov[2]] + n_mov[2]*13;
  Serial.println(martello_scelto);
  relay_double_write(modulo_ws(martello_scelto), numero_relay(martello_scelto), true);

  if (n_mov[0] == 0) {
  // se 0 - 12 allora M1

    Serial.println("ALZO BARRA 1");
    M1->moveTo(CORSA_M1);
    while(M1->isRunning()){};

    Serial.println("aspetto 1 secondo");
    delay(1000);

    Serial.println("ABBASSO BARRA 1");
    M1->moveTo(0);
    while(M1->isRunning()){};
    
  } else if (n_mov[0] == 1) {
  // se 13 - 25 allora M2

    Serial.println("ALZO BARRA 2");
    M2->moveTo(CORSA_M2);
    while(M2->isRunning()){};

    Serial.println("aspetto 1 secondo");
    delay(1000);

    Serial.println("ABBASSO BARRA 2");
    M2->moveTo(0);
    while(M2->isRunning()){};

  } else if (n_mov[0] == 2) {
  // se 26 - 38 allora M3
    Serial.println("ALZO BARRA 3");
    M3->moveTo(CORSA_M3);
    while(M3->isRunning()){};

    Serial.println("aspetto 1 secondo");
    delay(1000);
    
    Serial.println("ABBASSO BARRA 3");
    M3->moveTo(0);
    while(M3->isRunning()){};
  }

  n_mov[0] = n_mov[1];
  n_mov[1] = n_mov[2];
  
  while (n_mov[2] == n_mov[1]) {
    n_mov[2] = random(3);
  }

}

void sequenza_3_prima() {
  // genera numero da 0 a 38 : 39 martelli - 13 per asta
  martello_scelto = random(NUM_MARTELLI); 
  Serial.print("SCELGO MARTELLO: ");
  Serial.println(martello_scelto);

  Serial.print("MODULO: ");
  Serial.print(modulo_ws(martello_scelto));
  Serial.print(" - NUMERO: ");
  Serial.println(numero_relay(martello_scelto));

  // spingi attuatore lineare
  Serial.println("SPINGO ATTUATORE LINEARE");
  relay_double_write(modulo_ws(martello_scelto), numero_relay(martello_scelto), true);

  // aspetto 5 secondi
  Serial.println("aspetto 5 sec");
  delay(ATTESA_ATTUATORE);

  if (martello_scelto < 13) {
  // se 0 - 12 allora M1

    Serial.println("ALZO BARRA 1");
    M1->moveTo(CORSA_M1);
    while(M1->isRunning()){};

    Serial.println("aspetto 1 secondo");
    delay(1000);

    Serial.println("ABBASSO BARRA 1");
    M1->moveTo(0);
    while(M1->isRunning()){};
    
  } else if (martello_scelto < 26) {
  // se 13 - 25 allora M2

    Serial.println("ALZO BARRA 2");
    M2->moveTo(CORSA_M2);
    while(M2->isRunning()){};

    Serial.println("aspetto 1 secondo");
    delay(1000);

    Serial.println("ABBASSO BARRA 2");
    M2->moveTo(0);
    while(M2->isRunning()){};

  } else if (martello_scelto < 39) {
  // se 26 - 38 allora M3
    Serial.println("ALZO BARRA 3");
    M3->moveTo(CORSA_M3);
    while(M3->isRunning()){};

    Serial.println("aspetto 1 secondo");
    delay(1000);
    
    Serial.println("ABBASSO BARRA 3");
    M3->moveTo(0);
    while(M3->isRunning()){};
  }

  // ritorno attuatore lineare
  Serial.println("RITORNO ATTUATORE LINEARE");
  relay_double_write(modulo_ws(martello_scelto), numero_relay(martello_scelto), false);

  // aspetto 5 secondi
  Serial.println("aspetto 5 sec");
  delay(ATTESA_ATTUATORE);

  Serial.println();
}

void relay_single_write(uint8_t id_device, uint8_t id_relay, bool stato_relay) {

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

void relay_double_write(uint8_t id_device, uint8_t id_relay, bool stato_relay) {

  unsigned char cmd[10]; 

  cmd[0] = id_device;      // DEVICE ADDRESS: 0x00 BROADCAST
  cmd[1] = 0x0F;      // COMMAND: 0x01: READ COIL 0x03: READ ADDRESS 0x05 WRITE COIL 0x06 SET BAUDRATE AND ADRESS 0x0F WRITE MULTIPLE
  cmd[2] = 0x00;      // RELAY ADDRESS MSB:
  cmd[3] = id_relay - 2;  // RELAY ADDRESS LSB:

  cmd[4] = 0x00;  // NUMERO RELAY DA OPERARE
  cmd[5] = 0x02;

  cmd[6] = 0x01; // in quanti byte scrivo lo stato dei relay?

  if (stato_relay) {
    cmd[7] = 0x03; // tutte e due accesi
  } else {
    cmd[7] = 0x00; // tutte e due spenti
  }
  
  uint16_t crc = ModbusCRC((unsigned char  *)cmd,8);
  cmd[8] = crc & 0xFF;
  cmd[9] = crc >> 8;
  
  // SEND IT
  for (uint8_t i=0; i<10; i++) {
    RS485.write(cmd[i]);
  }

}

uint8_t modulo_ws(uint8_t num_mot) {
  if (num_mot < 8) {
    return 1;
  } else if (num_mot < 16) {
    return 2;
  } else if (num_mot < 24) {
    return 3;
  } else if (num_mot < 32) {
    return 4;
  } else if (num_mot < 40) {
    return 5;
  } else {
    return 1;
  }
}

uint8_t numero_relay(uint8_t num_mot) {
  uint8_t mod = modulo_ws(num_mot) - 1;
  return (num_mot - mod * 8) * 2;
}

void goToHome() {

  M1->runBackward();
  while (!digitalRead(ADIO2)){};
  M1->forceStop();
  delay(100);
  M1->setCurrentPosition(0);

  M2->runBackward();
  while (!digitalRead(ADIO3)){};
  M2->forceStop();
  delay(100);
  M2->setCurrentPosition(0);

  M3->runBackward();
  while (!digitalRead(ADIO4)){};
  M3->forceStop();
  delay(100);
  M3->setCurrentPosition(0);

}