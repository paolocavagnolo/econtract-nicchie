// 06_AGO - scheda EQSP32 (base 18/08/2025 di Paolo, con motore BASE)
// AUTO: zero di BASE e TOP sui finecorsa, poi BASE e TOP oscillano tra 0 e la corsa (6 s per tratta).
// MAN: pulsanti AVANTI/INDIETRO sul motore scelto (BASE o TOP), finecorsa ignorati.
// REMOTE: torna a zero e spegne i driver.

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
#include <Preferences.h>
#include <PaginaWebNicchie.h>   // WiFi proprio + pagina comandi (firmware/lib_comune)
#include <JogWebFAS.h>

// ---- NOMI DEGLI INGRESSI/USCITE ----
#define PIN_SEL_AUTO    ADIO1   // selettore su AUTO
#define PIN_SEL_MAN     ADIO2   // selettore su MANUALE
#define PIN_FC_TOP      ADIO3   // finecorsa zero TOP (prima era DIO9)
#define PIN_BTN_FWD     ADIO4   // pulsante avanti (solo MAN)
#define PIN_BTN_BWD     ADIO5   // pulsante indietro (solo MAN)
#define PIN_SEL_BASE    ADIO6   // scelta motore BASE (solo MAN)
#define PIN_SEL_TOP     ADIO7   // scelta motore TOP (solo MAN)
#define PIN_FC_BASE     ADIO8   // finecorsa zero BASE
#define PIN_BASE_STEP   DIO10
#define PIN_BASE_DIR    DIO11
#define PIN_BASE_EN     DIO12
#define PIN_TOP_STEP    DIO13
#define PIN_TOP_DIR     DIO14
#define PIN_TOP_EN      DIO15

// ---- SICUREZZA ----
#define TIMEOUT_HOME_BASE_MS   20000UL  // ms: tempo massimo per trovare il finecorsa BASE (motore lento)
#define TIMEOUT_HOME_TOP_MS    10000UL  // ms: tempo massimo per trovare il finecorsa TOP
#define STABILITA_SELETTORE_MS 200UL    // ms

#define BASE_SPEED  40      // passi/s: BASE in azzeramento e manuale
#define BASE_ACC    160     // passi/s^2

#define TOP_SPEED  500      // passi/s
#define TOP_ACC    5000     // passi/s^2

#define MAN_SPEED 500       // passi/s: TOP in azzeramento e manuale
#define MAN_ACC 1000        // passi/s^2

#define ZERO_OFFSET_BASE 5  // passi dopo il finecorsa
#define ZERO_OFFSET_TOP 5   // passi dopo il finecorsa

#define CORSA_BASE -120     // passi: corsa BASE in AUTO
#define CORSA_TOP 1200      // passi: corsa TOP in AUTO
#define TEMPO_TRATTA_S 6    // s: durata di ogni tratta in AUTO
#define RITARDO_TOP_MS 3000UL // ms: il TOP parte dopo l'avvio del ciclo

// ---- PAGINA WEB ----
#define NOME_RETE        "06_AGO"          // nome della rete WiFi della scheda
#define JOG_LENTO_TOP    (MAN_SPEED / 2)   // passi/s: jog da pagina TOP "Lento"
#define JOG_MEDIO_TOP    MAN_SPEED         // passi/s: TOP come i pulsanti del quadro
#define JOG_VELOCE_TOP   (MAN_SPEED * 2)   // passi/s: massimo da pagina TOP
#define JOG_LENTO_BASE   (BASE_SPEED / 2)  // passi/s: jog da pagina BASE "Lento"
#define JOG_MEDIO_BASE   BASE_SPEED        // passi/s: BASE come i pulsanti del quadro
#define JOG_VELOCE_BASE  (BASE_SPEED * 2)  // passi/s: massimo da pagina BASE

FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *BASE = NULL;
FastAccelStepper *TOP = NULL;

uint8_t STATE = 0; // 0 REMOTE - 1 AUTO - 2 MAN

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
unsigned long tSequenza = 0;   // inizio del ciclo AUTO

// Blocco di sicurezza (salvato in memoria permanente)
Preferences sicurezza;
bool BLOCCO = false;

void goToHome();
void attivaBlocco();
void inizializzaSicurezza();
void enableMotor();
void disableMotor();
bool leggiMan();
bool leggiAvvio();
void configuraPaginaWeb();
void aggiornaPaginaWeb();

void setup() {

  delay(1000);

  // DEBUG
  Serial.begin(115200);
  delay(100);

  // LED OFF
  pinMode(LED_BLUE, INPUT);

  // PANEL INPUT
  pinMode(PIN_SEL_AUTO, INPUT);  // AUTO
  pinMode(PIN_SEL_MAN, INPUT);  // MAN

  pinMode(PIN_BTN_FWD, INPUT);  // FWD
  pinMode(PIN_BTN_BWD, INPUT);  // BWD

  pinMode(PIN_SEL_BASE, INPUT);  // SEL BASE
  pinMode(PIN_SEL_TOP, INPUT);  // SEL TOP

  pinMode(PIN_FC_BASE, INPUT);  // FINE CORSA BASE
  pinMode(PIN_FC_TOP, INPUT);   // FINE CORSA TOP //DIO9

  // DRIVER OUTPUT

  pinMode(PIN_BASE_EN, OUTPUT);
  pinMode(PIN_TOP_EN, OUTPUT);
  disableMotor();

  engine.init();

  BASE = engine.stepperConnectToPin(PIN_BASE_STEP);

  if (BASE) {
      BASE->setDirectionPin(PIN_BASE_DIR, false);  
      BASE->setSpeedInHz(BASE_SPEED);  
      BASE->setAcceleration(BASE_ACC);   
  }

  TOP = engine.stepperConnectToPin(PIN_TOP_STEP);

  if (TOP) {
      TOP->setDirectionPin(PIN_TOP_DIR, false);  
      TOP->setSpeedInHz(TOP_SPEED);  
      TOP->setAcceleration(TOP_ACC);   
  } 

  // BUZZER 
  ledcSetup(0, 1000, 8);
  ledcAttachPin(BUZZ, 0);

  delay(1000);

  inizializzaSicurezza();

  // INITIAL STATE
  if (digitalRead(PIN_SEL_MAN)) {
    STATE = 2;
  } else {
    if (digitalRead(PIN_SEL_AUTO)) {
      STATE = 1;
    } else {
      STATE = 0;
    }
  }

  configuraPaginaWeb();

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

  check_MAN();
  check_AUTO();
  check_FWD();
  check_BWD();

  check_SEL_1();
  check_SEL_2();

  check_STATE();

  logica();

  // pagina web: jog solo in MANUALE, poi aggiorno lo stato mostrato
  jogWeb(0, TOP,  MAN_ACC,  MAN_SPEED,  STATE == 2);
  jogWeb(1, BASE, BASE_ACC, BASE_SPEED, STATE == 2);
  aggiornaPaginaWeb();

}

void logica() {
  if (BLOCCO && STATE != 2) {
    delay(50);
    return;
  }
  if (STATE == 0) {
    // REMOTO - IDLE
    delay(50);

  } else if (STATE == 1) {
    // AUTO
    if (!BASE->isRunning()) {
      long pos_base = BASE->getCurrentPosition();
      if (pos_base == CORSA_BASE)  act_BASE(-CORSA_BASE, TEMPO_TRATTA_S);
      else if (pos_base == 0)      act_BASE(CORSA_BASE, TEMPO_TRATTA_S);
      else                         act_BASE(0 - pos_base, TEMPO_TRATTA_S);
    }

    if ((millis() - tSequenza) > RITARDO_TOP_MS) {
      if (!TOP->isRunning()) {
        long pos_top = TOP->getCurrentPosition();
        if (pos_top == CORSA_TOP)    act_TOP(-CORSA_TOP, TEMPO_TRATTA_S);
        else if (pos_top == 0)       act_TOP(CORSA_TOP, TEMPO_TRATTA_S);
        else                         act_TOP(0 - pos_top, TEMPO_TRATTA_S);   // (nel codice 18/08 qui c'era act_BASE: corretto)
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

  // In BLOCCO: AUTO e REMOTE disabilitati, MANUALE sempre permesso
  if (BLOCCO && !f_MAN && STATE != 2) {
    f_AUTO = false;
    f_REM = false;
    return;
  }

  if (f_MAN) {

    f_MAN = false;
    STATE = 2;
    HOMED = false;  // al ritorno in AUTO rifà lo zero
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
      if (!HOMED) return;  // timeout: macchina in BLOCCO
    }

    if ((BASE->isRunning()) || (TOP->isRunning())) {
      BASE->stopMove();
      TOP->stopMove();
      while ((BASE->isRunning()) || (TOP->isRunning()));
    }

    Serial.println("INIZIO LOOP");
    tSequenza = millis();

  } else if (f_REM) {

    f_REM = false;
    if (leggiAvvio()) {

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
      if (leggiMan()) {
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
      if (!leggiMan()) {
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
      if (leggiAvvio()) {
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
      if (!leggiAvvio()) {
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
      if (digitalRead(PIN_BTN_FWD)) {
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
      if (!digitalRead(PIN_BTN_FWD)) {
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
      if (digitalRead(PIN_BTN_BWD)) {
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
      if (!digitalRead(PIN_BTN_BWD)) {
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
      if (digitalRead(PIN_SEL_BASE)) {
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
      if (!digitalRead(PIN_SEL_BASE)) {
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
      if (digitalRead(PIN_SEL_TOP)) {
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
      if (!digitalRead(PIN_SEL_TOP)) {
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
  // Mai in BLOCCO o in MANUALE (in MAN i finecorsa sono ignorati)
  if (BLOCCO || STATE == 2) return;

  HOMED = false;
  paginaWeb.impostaStato("Azzeramento in corso");

  if ((BASE->isRunning()) || (TOP->isRunning())) {
    BASE->stopMove();
    TOP->stopMove();
    while ((BASE->isRunning()) || (TOP->isRunning()));
  }

  BASE->setSpeedInHz(BASE_SPEED);
  BASE->setAcceleration(BASE_ACC);
  BASE->applySpeedAcceleration();

  TOP->setSpeedInHz(MAN_SPEED);
  TOP->setAcceleration(MAN_ACC);
  TOP->applySpeedAcceleration();

  // 1) BASE avanti fino al suo finecorsa
  BASE->runForward();
  uint32_t tInizio = millis();
  while (!digitalRead(PIN_FC_BASE)) {
    if ((millis() - tInizio) > TIMEOUT_HOME_BASE_MS) {
      attivaBlocco();   // finecorsa BASE non trovato: NON si considera azzerato
      return;
    }
  }
  BASE->forceStop();

  // 2) TOP indietro fino al suo finecorsa
  TOP->runBackward();
  tInizio = millis();
  while (!digitalRead(PIN_FC_TOP)) {
    if ((millis() - tInizio) > TIMEOUT_HOME_TOP_MS) {
      attivaBlocco();   // finecorsa TOP non trovato: NON si considera azzerato
      return;
    }
  }
  TOP->forceStop();

  delay(100);

  BASE->move(ZERO_OFFSET_BASE);
  TOP->move(ZERO_OFFSET_TOP);
  while ((BASE->isRunning()) || (TOP->isRunning()));

  BASE->setCurrentPosition(0);
  TOP->setCurrentPosition(0);

  HOMED = true;
}

// Ferma tutto e mette la macchina in BLOCCO (resta anche da spenta)
void attivaBlocco() {
  BASE->forceStop();
  TOP->forceStop();
  while ((BASE->isRunning()) || (TOP->isRunning())) {}
  disableMotor();
  HOMED = false;
  BLOCCO = true;
  STATE = 0;
  sicurezza.putBool("blocco", true);
  paginaWeb.impostaBlocco(true);
  Serial.println("BLOCCO: finecorsa non trovato entro il tempo massimo. Riaccendere in MANUALE per sbloccare.");
  for (int k = 0; k < 3; k++) {   // tre beep lunghi
    ledcWriteTone(0, 1000);
    delay(400);
    ledcWriteTone(0, 0);
    delay(200);
  }
}

// Legge il blocco salvato; se si accende con il selettore su MANUALE lo cancella
void inizializzaSicurezza() {
  sicurezza.begin("sicurezza", false);
  BLOCCO = sicurezza.getBool("blocco", false);

  bool man = digitalRead(PIN_SEL_MAN);
  uint32_t stabileDa = millis();
  while ((millis() - stabileDa) < STABILITA_SELETTORE_MS) {
    bool lettura = digitalRead(PIN_SEL_MAN);
    if (lettura != man) {
      man = lettura;
      stabileDa = millis();
    }
    delay(1);
  }

  if (BLOCCO && man) {
    BLOCCO = false;
    sicurezza.putBool("blocco", false);
    Serial.println("Sicurezza: BLOCCO cancellato (accensione in MANUALE)");
  } else if (BLOCCO) {
    Serial.println("BLOCCO attivo: AUTO/REMOTE disabilitati. Riaccendere in MANUALE.");
  }
}

void disableMotor() {
  delay(100);  // DA TESTARE
  digitalWrite(PIN_BASE_EN, LOW);
  digitalWrite(PIN_TOP_EN, LOW);
}

void enableMotor() {
  digitalWrite(PIN_BASE_EN, HIGH);
  digitalWrite(PIN_TOP_EN, HIGH);
  delay(100);  // DA TESTARE
}

void act_BASE(int32_t pos, float tempo) {

  if (pos == 0) return;

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

  if (pos == 0) return;

  long V_max, A_cal;

  float V_media = (float)abs(pos) / tempo;

  V_max = V_media * 2;   // step / s
  A_cal = V_max / (tempo / 2);   // step / s / s

  TOP->setSpeedInHz(V_max);  
  TOP->setAcceleration(A_cal);
  TOP->applySpeedAcceleration();

  TOP->move(pos);

}

// ---------------------------------------------------------------------
// PAGINA WEB (vedi firmware/lib_comune/PaginaWebNicchie)
// ---------------------------------------------------------------------

// Selettore "visto" dal programma: quello fisico OPPURE il comando della pagina.
// La pagina comanda solo con il selettore fisico su AUTO (lo controlla la libreria).
bool leggiMan()   { return digitalRead(PIN_SEL_MAN) || paginaWeb.manualeWeb(); }
bool leggiAvvio() { return digitalRead(PIN_SEL_AUTO) || paginaWeb.avvioWeb(0); }

void configuraPaginaWeb() {
  paginaWeb.aggiungiAvvio("AGO", PIN_SEL_AUTO, PIN_SEL_MAN);
  paginaWeb.aggiungiFinecorsa("Finecorsa TOP", PIN_FC_TOP);
  paginaWeb.aggiungiFinecorsa("Finecorsa BASE", PIN_FC_BASE);
  paginaWeb.aggiungiMotore("TOP", JOG_LENTO_TOP, JOG_MEDIO_TOP, JOG_VELOCE_TOP);
  paginaWeb.aggiungiMotore("BASE", JOG_LENTO_BASE, JOG_MEDIO_BASE, JOG_VELOCE_BASE);
  paginaWeb.begin(NOME_RETE, "Ago");
}

void aggiornaPaginaWeb() {
  paginaWeb.impostaBlocco(BLOCCO);
  if (STATE == 2)      paginaWeb.impostaStato(paginaWeb.manualeWeb() ? "Manuale da pagina" : "Manuale");
  else if (BLOCCO)     paginaWeb.impostaStato("Ferma (BLOCCO)");
  else if (STATE == 1) paginaWeb.impostaStato(HOMED ? "Ciclo automatico" : "Azzeramento");
  else                 paginaWeb.impostaStato("Ferma, in attesa di avvio");
}