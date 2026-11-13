// =====================================================================
// C02 - BOBINE + TENDIPELLE  (scheda EQSP32)
// Bobine: relè su RS485/Modbus, accese a gruppi di tre a caso (ADIO1).
// Tendipelle: due motori M1/M2 avanti/indietro con finecorsa (ADIO2).
// Nessun selettore MAN/AUTO.
// SICUREZZA: se un finecorsa del tendipelle non arriva in tempo, il
// tendipelle va in BLOCCO (motori fermi e spenti). Le bobine continuano.
// Si sblocca riavviando la scheda.
// PAGINA WEB: WiFi "C02_BOBINE_TENDIPELLE", 192.168.4.1. Senza selettore la
// scheda e' sempre "su AUTO": la pagina comanda finche' non arriva un
// avvio esterno. Il manuale da pagina muove i due motori del tendipelle.
// =====================================================================

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

// ATTENZIONE: i nomi DI/RO potrebbero essere scambiati rispetto al
// datasheet (RS485.begin vuole RX, TX). Se i relè rispondono va bene
// cosi', ma conviene verificarlo sul cablaggio.
#define RS485_DI    18
#define RS485_RO    17 
#define RS485_EN    35

HardwareSerial RS485(2);

#define LED_RED     38
#define LED_BLUE    37 //INPUT: OFF / OUTPUT_LOW: ON_BT / OUTPUT_HIGH: ON_WIFI

#define SDA_PIN 33
#define SCL_PIN 34

#include "FastAccelStepper.h"
#include <PaginaWebNicchie.h>   // WiFi proprio + pagina comandi (firmware/lib_comune)
#include <JogWebFAS.h>

// ---- NOMI DEGLI INGRESSI/USCITE ----
#define PIN_AVVIO_BOBINE   ADIO1
#define PIN_AVVIO_TENDI    ADIO2
#define PIN_FC_M1          ADIO3   // finecorsa M1
#define PIN_FC_M2          ADIO4   // finecorsa M2
#define PIN_M1_STEP        DIO9
#define PIN_M1_DIR         DIO10
#define PIN_M1_EN          DIO11   // LOW = acceso
#define PIN_M2_STEP        DIO12
#define PIN_M2_DIR         DIO13
#define PIN_M2_EN          DIO14   // LOW = acceso
#define PIN_AUDIO_BOBINE   DIO15
#define PIN_AUDIO_TENDI    DIO16

// ---- SICUREZZA ----
#define TIMEOUT_FC_M1_MS        3000UL  // ms: ricerca finecorsa M1
#define TIMEOUT_FC_M2_AVANTI_MS 1000UL  // ms: prima ricerca M2 in avanti (normale che scada)
#define TIMEOUT_FC_M2_MS        4000UL  // ms: ricerca M2 all'indietro (se scade -> BLOCCO)

#define M1_SPEED  8000
#define M1_ACC    5000

#define M2_SPEED  8000
#define M2_ACC    5000

#define CORSA     900
#define ZERO_OFFSET_TENDIPELLE 0

// ---- PAGINA WEB ----
#define NOME_RETE   "C02_BOBINE_TENDIPELLE"  // nome della rete WiFi della scheda
#define JOG_LENTO   1000                     // passi/s: jog tendipelle "Lento"
#define JOG_MEDIO   (M1_SPEED / 2)           // passi/s: "Medio" (velocita' dell'azzeramento)
#define JOG_VELOCE  M1_SPEED                 // passi/s: massimo (velocita' del ciclo)

FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *M1 = NULL;
FastAccelStepper *M2 = NULL;

bool prima_volta_bobine = true;
bool prima_uscita_bobine = true;

bool prima_volta_tendipelle = true;
bool prima_uscita_tendipelle = true;

uint8_t bobina_scelta[3];
unsigned long tWait = 0, tBobine = 0;
bool tOn = true;
unsigned long tUscitaBobine = 0;
bool mezza = true;

// Blocco di sicurezza del tendipelle: solo in RAM, si cancella riavviando
bool BLOCCO = false;

// Manuale da pagina web (questa scheda non ha un manuale da quadro)
bool inManualeWeb = false;

void logica_tendipelle();
void logica_bobine_tre();
void logica_bobine_all();
void accendi_bobina(uint8_t n);
void relay_write(uint8_t id_device, uint8_t id_relay, bool stato_relay);
void goToHome_tendipelle();
void attivaBlocco();
bool leggiAvvioBobine();
bool leggiAvvioTendi();
void gestisciManualeWeb();
void configuraPaginaWeb();
void aggiornaPaginaWeb();

void setup() {

  delay(1000);

  // DEBUG
  Serial.begin(115200);
  delay(100);

  // LED OFF
  pinMode(LED_BLUE, INPUT);

  // RS485
  RS485.begin(9600, SERIAL_8N1, RS485_DI, RS485_RO);
  delay(100);
  pinMode(RS485_EN,OUTPUT);
  digitalWrite(RS485_EN,HIGH);

  // PANEL INPUT
  pinMode(PIN_AVVIO_BOBINE, INPUT);  // bobine marius
  pinMode(PIN_AVVIO_TENDI, INPUT);  // tendipelle marius
  pinMode(PIN_FC_M1, INPUT);  // finecorsa M1
  pinMode(PIN_FC_M2, INPUT);  // finecorsa M2

  // DRIVER OUTPUT
  pinMode(PIN_M1_EN, OUTPUT); // M1 enable
  digitalWrite(PIN_M1_EN, LOW);
  pinMode(PIN_M2_EN, OUTPUT); // M2 enable
  digitalWrite(PIN_M2_EN, LOW);

  delay(1000);

  engine.init();

  M1 = engine.stepperConnectToPin(PIN_M1_STEP);

  if (M1) {
      M1->setDirectionPin(PIN_M1_DIR);  
      M1->setSpeedInHz(M1_SPEED);  
      M1->setAcceleration(M1_ACC); 
      //M1->runForward();
 
  }

  M2 = engine.stepperConnectToPin(PIN_M2_STEP);

  if (M2) {
      M2->setDirectionPin(PIN_M2_DIR);  
      M2->setSpeedInHz(M2_SPEED);  
      M2->setAcceleration(M2_ACC); 
      //M2->runForward(); 
  }


  // AUDIO OUTPUT
  pinMode(PIN_AUDIO_BOBINE, OUTPUT);   // audio bobine
  pinMode(PIN_AUDIO_TENDI, OUTPUT);   // audio tirapelle
  digitalWrite(PIN_AUDIO_BOBINE, LOW);
  digitalWrite(PIN_AUDIO_TENDI, LOW);

  // BUZZER 
  ledcSetup(0, 1000, 8);
  ledcAttachPin(BUZZ, 0);

  configuraPaginaWeb();

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


void loop() {

 logica_bobine_tre();
 //logica_bobine_all();

 // for (uint8_t i=1; i<25; i++) {
 //    accendi_bobina(i);
 //  }
  
  logica_tendipelle();

  // Manuale da pagina: permesso anche in BLOCCO, per liberare la meccanica
  gestisciManualeWeb();
  aggiornaPaginaWeb();



}


void logica_tendipelle() {

  // In BLOCCO il tendipelle resta fermo fino al riavvio
  if (BLOCCO) return;

  if (leggiAvvioTendi()) {

    if (prima_volta_tendipelle) {
      prima_volta_tendipelle = false;
      prima_uscita_tendipelle = true;

      digitalWrite(PIN_AUDIO_TENDI, HIGH);

      digitalWrite(PIN_M1_EN, LOW);
      digitalWrite(PIN_M2_EN, LOW);
      delay(200);

      goToHome_tendipelle();
      if (BLOCCO) return;

    }

    if (!M1->isRunning() && !M2->isRunning()) {
      if ((M1->getCurrentPosition() == 0) && (M2->getCurrentPosition() == 0)) {
        M1->moveTo(CORSA);
        M2->moveTo(CORSA);
      }
      else if ((M1->getCurrentPosition() == CORSA) && (M2->getCurrentPosition() == CORSA)) {
        M1->moveTo(0);
        M2->moveTo(0);
      } else {
        M1->moveTo(0);
        M2->moveTo(0);
      }
    }
    



  } else {
    if (prima_uscita_tendipelle) {
      prima_uscita_tendipelle = false;
      prima_volta_tendipelle = true;

      M1->stopMove();
      while(M1->isRunning()){};
      M1->moveTo(0);
      while(M1->isRunning()){};

      M2->stopMove();
      while(M2->isRunning()){};
      M2->moveTo(0);
      while(M2->isRunning()){};

      delay(100);

      digitalWrite(PIN_M1_EN, HIGH);
      digitalWrite(PIN_M2_EN, HIGH);

      digitalWrite(PIN_AUDIO_TENDI, LOW);

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
    delay(1000);
    relay_write(1,nn,false);
    delay(500);
  } else {
    relay_write(2,nn-16,true);
    delay(1000);
    relay_write(2,nn-16,false);
    delay(500);
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

  if (leggiAvvioBobine()) {

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
      digitalWrite(PIN_AUDIO_BOBINE, HIGH);
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
      digitalWrite(PIN_AUDIO_BOBINE, LOW);
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

  M1->setSpeedInHz(M1_SPEED/2);  
  M1->setAcceleration(M1_ACC/2);  
  M1->applySpeedAcceleration();
  M2->setSpeedInHz(M2_SPEED/2);  
  M2->setAcceleration(M2_ACC/2);  
  M2->applySpeedAcceleration();

  paginaWeb.impostaStato("Tendipelle: azzeramento in corso");
  unsigned long tStart = millis();
  bool timeout = false;

  M1->runBackward();
  while (!digitalRead(PIN_FC_M1)){
    if ((millis() - tStart) > TIMEOUT_FC_M1_MS) {
      Serial.println("TIMEOUT 1");
      attivaBlocco();
      return;
    }
  };

  if (!timeout) {
    Serial.println("FINE CORSA 1 OK");
  } else {
    timeout = false;
  }

  M1->forceStop();
  delay(100);
  M1->setCurrentPosition(0);
  tStart = millis();

  M2->runForward();
  while (!digitalRead(PIN_FC_M2)){
    if ((millis() - tStart) > TIMEOUT_FC_M2_AVANTI_MS) {
      Serial.println("TIMEOUT FORWARD");
      timeout = true;
      break;
    }
  };

  tStart = millis();

  if (!timeout) {

    Serial.println("FINE CORSA 2 OK");

  } else {

    M2->runBackward();

    while (!digitalRead(PIN_FC_M2)){
      if ((millis() - tStart) > TIMEOUT_FC_M2_MS) {
        Serial.println("TIMEOUT BACKWARD");
        attivaBlocco();
        return;
      }
    };


  }

  M2->forceStop();
  delay(100);
  M2->setCurrentPosition(230);
  tStart = millis();

  M1->setSpeedInHz(M1_SPEED);  
  M1->setAcceleration(M1_ACC);  
  M1->applySpeedAcceleration();
  M2->setSpeedInHz(M2_SPEED);  
  M2->setAcceleration(M2_ACC);  
  M2->applySpeedAcceleration();

}

// Ferma il tendipelle e lo mette in BLOCCO (si toglie riavviando la scheda)
void attivaBlocco() {
  M1->forceStop();
  M2->forceStop();
  while (M1->isRunning() || M2->isRunning()) {}
  digitalWrite(PIN_M1_EN, HIGH);   // spento
  digitalWrite(PIN_M2_EN, HIGH);   // spento
  digitalWrite(PIN_AUDIO_TENDI, LOW);
  BLOCCO = true;
  paginaWeb.impostaBlocco(true);
  Serial.println("BLOCCO tendipelle: finecorsa non trovato. Riavviare la scheda.");
  for (int k = 0; k < 3; k++) {   // tre beep lunghi
    ledcWriteTone(0, 1000);
    delay(400);
    ledcWriteTone(0, 0);
    delay(200);
  }
}

// ---------------------------------------------------------------------
// PAGINA WEB (vedi firmware/lib_comune/PaginaWebNicchie)
// ---------------------------------------------------------------------

// Avvio "visto" dal programma: ingresso esterno OPPURE AVVIO dalla pagina
bool leggiAvvioBobine() { return digitalRead(PIN_AVVIO_BOBINE) || paginaWeb.avvioWeb(0); }
bool leggiAvvioTendi()  { return digitalRead(PIN_AVVIO_TENDI)  || paginaWeb.avvioWeb(1); }

// Entra/esce dal manuale da pagina e applica il jog. In manuale i finecorsa
// sono ignorati. Si entra solo a tendipelle fermo (la pagina toglie prima l'AVVIO).
void gestisciManualeWeb() {
  bool richiesto = paginaWeb.manualeWeb();
  bool fermo = BLOCCO || prima_volta_tendipelle;

  if (richiesto && !inManualeWeb && fermo) {
    inManualeWeb = true;
    digitalWrite(PIN_M1_EN, LOW);   // acceso
    digitalWrite(PIN_M2_EN, LOW);   // acceso
    Serial.println("MANUALE DA PAGINA");
  }

  jogWeb(0, M1, M1_ACC, M1_SPEED, inManualeWeb && richiesto);
  jogWeb(1, M2, M2_ACC, M2_SPEED, inManualeWeb && richiesto);

  if (!richiesto && inManualeWeb) {
    inManualeWeb = false;
    while (M1->isRunning() || M2->isRunning()) {}   // fine rampa di arresto
    digitalWrite(PIN_M1_EN, HIGH);  // spento (al prossimo avvio rifa' lo zero)
    digitalWrite(PIN_M2_EN, HIGH);  // spento
    Serial.println("FINE MANUALE DA PAGINA");
  }
}

void configuraPaginaWeb() {
  paginaWeb.aggiungiAvvio("BOBINE", PIN_AVVIO_BOBINE, PWN_NESSUN_PIN);
  paginaWeb.aggiungiAvvio("TENDIPELLE", PIN_AVVIO_TENDI, PWN_NESSUN_PIN);
  paginaWeb.aggiungiFinecorsa("Finecorsa M1", PIN_FC_M1);
  paginaWeb.aggiungiFinecorsa("Finecorsa M2", PIN_FC_M2);
  paginaWeb.aggiungiMotore("TENDIPELLE M1", JOG_LENTO, JOG_MEDIO, JOG_VELOCE);
  paginaWeb.aggiungiMotore("TENDIPELLE M2", JOG_LENTO, JOG_MEDIO, JOG_VELOCE);
  paginaWeb.aggiungiSpia("Audio bobine");
  paginaWeb.aggiungiSpia("Audio tendipelle");
  paginaWeb.begin(NOME_RETE, "Bobine e tendipelle");
}

void aggiornaPaginaWeb() {
  char testo[64];
  const char* t = inManualeWeb ? "manuale da pagina" : (BLOCCO ? "BLOCCO" : (prima_volta_tendipelle ? "fermo" : "ciclo"));
  snprintf(testo, sizeof(testo), "Bobine: %s - Tendipelle: %s", prima_volta_bobine ? "ferme" : "ciclo", t);
  paginaWeb.impostaStato(testo);
  paginaWeb.impostaBlocco(BLOCCO);
  paginaWeb.impostaSpia(0, pwnUscita(PIN_AUDIO_BOBINE));
  paginaWeb.impostaSpia(1, pwnUscita(PIN_AUDIO_TENDI));
}
