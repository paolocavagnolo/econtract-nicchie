// =====================================================================
// C01 - CUCIRE + MANICI  (scheda EQSP32)
// Due motori: macchina da cucire (giri a velocita' variabile) e manici
// (avanti/indietro con finecorsa di zero). Nessun selettore MAN/AUTO.
// Ingressi: ADIO1 avvio cucire, ADIO4 avvio manici, ADIO5 finecorsa manici
// Uscite: DIO9/10/11 cucire STEP/DIR/EN, DIO12/13/14 manici STEP/DIR/EN,
//         DIO15/16 segnali audio
// SICUREZZA: se il finecorsa dei manici non arriva entro TIMEOUT_HOME_MS
// la scheda va in BLOCCO (motori fermi). Si sblocca riavviando la scheda.
// PAGINA WEB: WiFi "C01_CUCIRE_MANICI", 192.168.4.1. Senza selettore la
// scheda e' sempre "su AUTO": la pagina comanda finche' non arriva un
// avvio esterno. Il manuale da pagina muove CUCIRE e MANICI.
// =====================================================================

#define ADIO1   1
#define ADIO2   2
#define ADIO3   4
#define ADIO4   5
#define ADIO5   8 //6 //7
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
#include <PaginaWebNicchie.h>   // WiFi proprio + pagina comandi (firmware/lib_comune)
#include <JogWebFAS.h>

// ---- NOMI DEGLI INGRESSI/USCITE ----
#define PIN_AVVIO_CUCIRE   ADIO1
#define PIN_AVVIO_MANICI   ADIO4
#define PIN_FC_MANICI      ADIO5   // finecorsa zero manici
#define PIN_CUCIRE_STEP    DIO9
#define PIN_CUCIRE_DIR     DIO10
#define PIN_CUCIRE_EN      DIO11   // HIGH = acceso
#define PIN_MANICI_STEP    DIO12
#define PIN_MANICI_DIR     DIO13
#define PIN_MANICI_EN      DIO14   // LOW = acceso
#define PIN_AUDIO_CUCIRE   DIO15
#define PIN_AUDIO_MANICI   DIO16

// ---- SICUREZZA ----
#define TIMEOUT_HOME_MS    10000UL  // ms: tempo massimo per trovare il finecorsa manici

#define M_CUCIRE_SPEED  1000
#define M_CUCIRE_ACC    1000

#define M_MANICI_SPEED  20000
#define M_MANICI_ACC    20000

#define CORSA_MANICI    100000
#define ZERO_OFFSET_MANICI 5000

// ---- PAGINA WEB ----
#define NOME_RETE          "C01_CUCIRE_MANICI"  // nome della rete WiFi della scheda
#define JOG_CUCIRE_LENTO   500                  // passi/s: jog cucire "Lento"
#define JOG_CUCIRE_MEDIO   1500                 // passi/s: jog cucire "Medio" (primo giro del ciclo)
#define JOG_CUCIRE_VELOCE  3200                 // passi/s: massimo (giro piu' veloce del ciclo)
#define JOG_MANICI_LENTO   2500                 // passi/s: jog manici "Lento"
#define JOG_MANICI_MEDIO   10000                // passi/s: jog manici "Medio"
#define JOG_MANICI_VELOCE  M_MANICI_SPEED       // passi/s: massimo (velocita' del ciclo)


FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *M_CUCIRE = NULL;
FastAccelStepper *M_MANICI = NULL;

uint8_t STATE = 0; // 0 REMOTE - 1 AUTO - 2 MAN

bool prima_volta_cucire = true;
bool prima_uscita_cucire = true;

bool prima_volta_manici = true;
bool prima_uscita_manici = true;

int S = -1;
bool P = true, E = false;
unsigned long T = 0;

unsigned long tt = 0;
int ponte = 0, old_ponte = 0;
int fc = 0, old_fc = 0;

unsigned long tDebug = 0;

// Blocco di sicurezza: solo in RAM, si cancella riavviando la scheda
bool BLOCCO = false;

// Manuale da pagina web (questa scheda non ha un manuale da quadro)
bool inManualeWeb = false;

void logica_cucire();
void logica_manici();
void goToHome_manici();
void attivaBlocco();
bool leggiAvvioCucire();
bool leggiAvvioManici();
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

  // PANEL INPUT
  pinMode(PIN_AVVIO_CUCIRE, INPUT);  // cucire marius
  pinMode(PIN_AVVIO_MANICI, INPUT);  // manici marius
  pinMode(PIN_FC_MANICI, INPUT);  // manici finecorsa

  // DRIVER OUTPUT
  pinMode(PIN_CUCIRE_EN, OUTPUT); // cucire enable
  pinMode(PIN_MANICI_EN, OUTPUT); // manici enable
  digitalWrite(PIN_CUCIRE_EN, LOW);
  digitalWrite(PIN_MANICI_EN, HIGH);

  engine.init();

  M_CUCIRE = engine.stepperConnectToPin(PIN_CUCIRE_STEP);

  if (M_CUCIRE) {
      M_CUCIRE->setDirectionPin(PIN_CUCIRE_DIR);  
      M_CUCIRE->setSpeedInHz(M_CUCIRE_SPEED);  
      M_CUCIRE->setAcceleration(M_CUCIRE_ACC);   
  }

  M_MANICI = engine.stepperConnectToPin(PIN_MANICI_STEP);

  if (M_MANICI) {
      M_MANICI->setDirectionPin(PIN_MANICI_DIR);  
      M_MANICI->setSpeedInHz(M_MANICI_SPEED);  
      M_MANICI->setAcceleration(M_MANICI_ACC);   
  }

  // AUDIO SIGNAL
  pinMode(PIN_AUDIO_CUCIRE, OUTPUT); // segnale audio cucire
  pinMode(PIN_AUDIO_MANICI, OUTPUT); // segnale audio manici
  digitalWrite(PIN_AUDIO_CUCIRE, LOW);
  digitalWrite(PIN_AUDIO_MANICI, LOW);


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

  delay(1000);

}


void loop() {

  // Manuale da pagina: permesso anche in BLOCCO, per liberare la meccanica
  gestisciManualeWeb();
  aggiornaPaginaWeb();

  // In BLOCCO non si muove piu' niente fino al riavvio
  if (BLOCCO) {
    delay(50);
    return;
  }

  logica_cucire();

  logica_manici();

}

void logica_cucire() {

  if (leggiAvvioCucire()) {

    if (prima_volta_cucire) {
      prima_volta_cucire = false;
      prima_uscita_cucire = true;

      digitalWrite(PIN_CUCIRE_EN, HIGH);
      digitalWrite(PIN_AUDIO_CUCIRE, HIGH);
      delay(200);
      S = 0;
      M_CUCIRE->runForward();
    }

  } else {

    if (prima_uscita_cucire) {
      prima_uscita_cucire = false;
      prima_volta_cucire = true;

      M_CUCIRE->stopMove();

      delay(1000);
      S = -1;
      digitalWrite(PIN_CUCIRE_EN, LOW);
      digitalWrite(PIN_AUDIO_CUCIRE, LOW);

    }

  }


  if (S == 0) {

    if (P) {
      P = false;
      E = false;
      M_CUCIRE->setSpeedInHz(500*3);
      M_CUCIRE->applySpeedAcceleration();
      T = millis();
      Serial.println("parte!");
    }

    if ((millis() - T) > 10000) {
      E = true;
    }


    if (E) {
      E = false;
      S = 1;
      P = true;
    }

  } else if (S == 1) {

    if (P) {
      P = false;
      E = false;
      M_CUCIRE->setSpeedInHz(1000*3);
      M_CUCIRE->applySpeedAcceleration();
      T = millis();
      Serial.println(S);
    }

    if ((millis() - T) > 5000) {
      E = true;
    }


    if (E) {
      E = false;
      S = 2;
      P = true;
    }
  } else if (S == 2) {

    if (P) {
      P = false;
      E = false;
      M_CUCIRE->setSpeedInHz(1600*2);
      M_CUCIRE->applySpeedAcceleration();
      T = millis();
      Serial.println(S);
    }

    if ((millis() - T) > 5000) {
      E = true;
    }


    if (E) {
      E = false;
      S = 3;
      P = true;
    }
  } else if (S == 3) {

    if (P) {
      P = false;
      E = false;
      M_CUCIRE->setSpeedInHz(250*3);
      M_CUCIRE->applySpeedAcceleration();
      T = millis();
      Serial.println(S);
    }

    if ((millis() - T) > 10000) {
      E = true;
    }


    if (E) {
      E = false;
      S = 0;
      P = true;
    }
  }
}


void logica_manici() {

  if (leggiAvvioManici()) {
    if (prima_volta_manici) {
      Serial.println("DENTRO");
      prima_volta_manici = false;
      prima_uscita_manici = true;

      digitalWrite(PIN_MANICI_EN, LOW);
      digitalWrite(PIN_AUDIO_MANICI, HIGH);
      delay(300);
      goToHome_manici();
      if (BLOCCO) return;

    }

    if (!M_MANICI->isRunning()) {
      if (M_MANICI->getCurrentPosition() == 0) {
        M_MANICI->move(CORSA_MANICI);
      } else if (M_MANICI->getCurrentPosition() == CORSA_MANICI) {
        M_MANICI->move(-CORSA_MANICI);
      } else {
        M_MANICI->moveTo(0);
      }
    } 

  } else {
    if (prima_uscita_manici) {
      Serial.println("FUORI");
      prima_uscita_manici = false;
      prima_volta_manici = true;

      M_MANICI->stopMove();
      while(M_MANICI->isRunning()){};
       M_MANICI->moveTo(0);
      while(M_MANICI->isRunning()){};
      //goToHome_manici();

      delay(1000);
      digitalWrite(PIN_MANICI_EN, HIGH);
      digitalWrite(PIN_AUDIO_MANICI, LOW);
    }

  }
}

void goToHome_manici() {

  Serial.println("ZERO INZIO");
  paginaWeb.impostaStato("Manici: azzeramento in corso");
  M_MANICI->runBackward();

  unsigned long time_out = millis();

  while (!digitalRead(PIN_FC_MANICI)) {
    if ((millis() - time_out) > TIMEOUT_HOME_MS) {
      attivaBlocco();   // finecorsa non trovato: NON si considera azzerato
      return;
    }
  }
  M_MANICI->forceStop();
  while(M_MANICI->isRunning());
  M_MANICI->move(ZERO_OFFSET_MANICI);
  while (M_MANICI->isRunning()){};
  M_MANICI->setCurrentPosition(0);
  Serial.println("ZERO FINE");

}

// Ferma tutti i motori e mette la scheda in BLOCCO (si toglie riavviando)
void attivaBlocco() {
  M_MANICI->forceStop();
  M_CUCIRE->forceStop();
  while (M_MANICI->isRunning() || M_CUCIRE->isRunning()) {}
  digitalWrite(PIN_MANICI_EN, HIGH);   // manici spento
  digitalWrite(PIN_CUCIRE_EN, LOW);    // cucire spento
  digitalWrite(PIN_AUDIO_CUCIRE, LOW);
  digitalWrite(PIN_AUDIO_MANICI, LOW);
  BLOCCO = true;
  paginaWeb.impostaBlocco(true);
  Serial.println("BLOCCO: finecorsa manici non trovato. Riavviare la scheda.");
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
bool leggiAvvioCucire() { return digitalRead(PIN_AVVIO_CUCIRE) || paginaWeb.avvioWeb(0); }
bool leggiAvvioManici() { return digitalRead(PIN_AVVIO_MANICI) || paginaWeb.avvioWeb(1); }

// Entra/esce dal manuale da pagina e applica il jog. In manuale i finecorsa
// sono ignorati. Si entra solo a cicli fermi (la pagina toglie prima l'AVVIO).
void gestisciManualeWeb() {
  bool richiesto = paginaWeb.manualeWeb();
  // cicli fermi = nessun ciclo in corso (in BLOCCO i motori sono gia' fermati)
  bool cicliFermi = BLOCCO || (prima_volta_cucire && prima_volta_manici);

  if (richiesto && !inManualeWeb && cicliFermi) {
    inManualeWeb = true;
    digitalWrite(PIN_CUCIRE_EN, HIGH);   // cucire acceso
    digitalWrite(PIN_MANICI_EN, LOW);    // manici acceso
    Serial.println("MANUALE DA PAGINA");
  }

  jogWeb(0, M_CUCIRE, M_CUCIRE_ACC, M_CUCIRE_SPEED, inManualeWeb && richiesto);
  jogWeb(1, M_MANICI, M_MANICI_ACC, M_MANICI_SPEED, inManualeWeb && richiesto);

  if (!richiesto && inManualeWeb) {
    inManualeWeb = false;
    while (M_CUCIRE->isRunning() || M_MANICI->isRunning()) {}   // fine rampa di arresto
    digitalWrite(PIN_CUCIRE_EN, LOW);    // cucire spento
    digitalWrite(PIN_MANICI_EN, HIGH);   // manici spento (al prossimo avvio rifa' lo zero)
    Serial.println("FINE MANUALE DA PAGINA");
  }
}

void configuraPaginaWeb() {
  paginaWeb.aggiungiAvvio("CUCIRE", PIN_AVVIO_CUCIRE, PWN_NESSUN_PIN);
  paginaWeb.aggiungiAvvio("MANICI", PIN_AVVIO_MANICI, PWN_NESSUN_PIN);
  paginaWeb.aggiungiFinecorsa("Finecorsa manici", PIN_FC_MANICI);
  paginaWeb.aggiungiMotore("CUCIRE", JOG_CUCIRE_LENTO, JOG_CUCIRE_MEDIO, JOG_CUCIRE_VELOCE);
  paginaWeb.aggiungiMotore("MANICI", JOG_MANICI_LENTO, JOG_MANICI_MEDIO, JOG_MANICI_VELOCE);
  paginaWeb.aggiungiSpia("Audio cucire");
  paginaWeb.aggiungiSpia("Audio manici");
  paginaWeb.begin(NOME_RETE, "Cucire e manici");
}

void aggiornaPaginaWeb() {
  char testo[64];
  if (BLOCCO && !inManualeWeb) snprintf(testo, sizeof(testo), "Ferma (BLOCCO)");
  else if (inManualeWeb)       snprintf(testo, sizeof(testo), "Manuale da pagina");
  else snprintf(testo, sizeof(testo), "Cucire: %s - Manici: %s",
                prima_volta_cucire ? "fermo" : "ciclo", prima_volta_manici ? "fermo" : "ciclo");
  paginaWeb.impostaStato(testo);
  paginaWeb.impostaBlocco(BLOCCO);
  paginaWeb.impostaSpia(0, pwnUscita(PIN_AUDIO_CUCIRE));
  paginaWeb.impostaSpia(1, pwnUscita(PIN_AUDIO_MANICI));
}
