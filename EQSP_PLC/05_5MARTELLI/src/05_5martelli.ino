// =====================================================================
// 05 - 5 MARTELLI  (scheda EQSP32)
// Questa scheda NON muove motori: legge il selettore AUTO/MAN e
// comanda la ESP32 "06_MARTELLI" (cartella ESP32/06_MARTELLI):
//  - DIO9  relè che ACCENDE la ESP32 (la ESP32 riparte da zero ogni volta)
//  - DIO10 segnale di avvio verso la ESP32 (letto su GPIO34)
// AUTO: accende il relè, aspetta 2 s, manda l'avvio.
// MAN:  accende il relè senza avvio.
// SICUREZZA: dopo TEMPO_MAX_CICLO_MS spegne il relè (stop della 06).
// PAGINA WEB: WiFi "05_5MARTELLI". Solo la 05 ha la pagina (la 06 no):
// mostra relè/avvio/ciclo e permette AVVIO e FERMA. Non muove i motori
// della 06, perche' la 05 le manda solo il segnale di avvio.
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

#define RS485_DI    18
#define RS485_RO    17 
#define RS485_EN    35

#define LED_RED     38
#define LED_BLUE    37 //INPUT: OFF / OUTPUT_LOW: ON_BT / OUTPUT_HIGH: ON_WIFI

#define SDA_PIN 33
#define SCL_PIN 34

#include <PaginaWebNicchie.h>   // WiFi proprio + pagina comandi (firmware/lib_comune)

// ---- NOMI DEGLI INGRESSI/USCITE ----
#define PIN_SEL_AUTO   ADIO1   // selettore su AUTO
#define PIN_SEL_MAN    ADIO2   // selettore su MANUALE
#define PIN_RELE_ESP   DIO9    // relè alimentazione ESP32 06_MARTELLI
#define PIN_AVVIO_ESP  DIO10   // segnale avvio verso ESP32

// ---- TEMPI ----
#define ATTESA_ACCENSIONE_MS 2000UL    // ms: attesa avvio ESP32 dopo il relè
#define TEMPO_MAX_CICLO_MS   125000UL  // ms: oltre questo tempo il relè si spegne
                                       // (la 06 si ferma da sola a 120 s)

// ---- PAGINA WEB ----
#define NOME_RETE "05_5MARTELLI"   // nome della rete WiFi della scheda

unsigned long tAccensione = 0;   // momento in cui è partito il ciclo AUTO
bool cicloAttivo = false;

uint8_t STATE = 0; // 0 REMOTE - 1 AUTO - 2 MAN

unsigned long t_MAN = 0, tS_MAN = 0;
bool s_MAN = false, p_MAN = true, f_MAN = false;
unsigned long t_AUTO = 0, tS_AUTO = 0;
bool s_AUTO = false, p_AUTO = true, f_AUTO = false;
bool f_REM = false;

void check_STATE();
void check_MAN();
void check_AUTO();
bool leggiAvvio();
void configuraPaginaWeb();
void aggiornaPaginaWeb();

void setup() {

  delay(100);

  // DEBUG
  Serial.begin(115200);
  delay(100);

  // LED OFF
  pinMode(LED_BLUE, INPUT);

  // PANEL INPUT
  pinMode(PIN_SEL_AUTO, INPUT);  // AUTO
  pinMode(PIN_SEL_MAN, INPUT);  // MAN

  // ESP32 OUTPUT
  pinMode(PIN_RELE_ESP, OUTPUT); // relè accensione
  pinMode(PIN_AVVIO_ESP, OUTPUT); // segnale avvio


  // BUZZER 
  ledcSetup(0, 1000, 8);
  ledcAttachPin(BUZZ, 0);

  delay(1000);

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

  check_STATE();

  aggiornaPaginaWeb();

}

void check_STATE() {

  // SICUREZZA: ciclo AUTO troppo lungo -> spengo la ESP32 dei martelli
  if (cicloAttivo && (millis() - tAccensione) > TEMPO_MAX_CICLO_MS) {
    cicloAttivo = false;
    digitalWrite(PIN_AVVIO_ESP, LOW);
    digitalWrite(PIN_RELE_ESP, LOW);
    Serial.println("TEMPO MASSIMO: rele' spento. Per ripartire girare il selettore.");
    paginaWeb.annullaAvvio(0);   // se l'avvio era dalla pagina, si spegne anche li'
  }

  if (f_MAN) {

    f_MAN = false;
    STATE = 2;
    cicloAttivo = false;

    digitalWrite(PIN_RELE_ESP, HIGH); // rele
    digitalWrite(PIN_AVVIO_ESP, LOW);  // segnale

  } else if ((f_AUTO) and (STATE != 2)) {

    f_AUTO = false;
    STATE = 1;

    Serial.println("INIZIO LOOP");

    digitalWrite(PIN_RELE_ESP, HIGH); // rele: accende la ESP32

    // Attesa accensione: se nel frattempo il selettore lascia AUTO
    // l'avvio viene annullato
    unsigned long inizioAttesa = millis();
    while ((millis() - inizioAttesa) < ATTESA_ACCENSIONE_MS) {
      if (!leggiAvvio()) {
        Serial.println("AVVIO ANNULLATO: selettore spostato");
        digitalWrite(PIN_RELE_ESP, LOW);
        STATE = 0;
        return;   // il cambio di selettore verrà gestito da check_AUTO/check_MAN
      }
      delay(10);
    }

    digitalWrite(PIN_AVVIO_ESP, HIGH); // segnale di avvio
    tAccensione = millis();
    cicloAttivo = true;

  } else if (f_REM) {
    f_REM = false;
    STATE = 0;

    if (leggiAvvio()) {

      STATE = 1;
      f_AUTO = true;

    } else {

      STATE = 0;
      cicloAttivo = false;
      digitalWrite(PIN_AVVIO_ESP, LOW);
      delay(2000);
      digitalWrite(PIN_RELE_ESP, LOW);

    }

  }

}

void check_MAN() {
  if (!s_MAN) {
    if ((millis() - t_MAN) > 200) {
      if (digitalRead(PIN_SEL_MAN)) {
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
      if (!digitalRead(PIN_SEL_MAN)) {
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

// ---------------------------------------------------------------------
// PAGINA WEB (vedi firmware/lib_comune/PaginaWebNicchie)
// ---------------------------------------------------------------------

// Avvio "visto" dal programma: filo AUTO/REMOTE OPPURE AVVIO dalla pagina.
// La pagina comanda solo con il selettore fisico su AUTO (lo controlla la libreria).
bool leggiAvvio() { return digitalRead(PIN_SEL_AUTO) || paginaWeb.avvioWeb(0); }

void configuraPaginaWeb() {
  paginaWeb.aggiungiAvvio("MARTELLI", PIN_SEL_AUTO, PIN_SEL_MAN);
  paginaWeb.aggiungiSpia("Rele' accensione ESP32 06");
  paginaWeb.aggiungiSpia("Segnale avvio alla 06");
  paginaWeb.aggiungiSpia("Ciclo in corso");
  paginaWeb.begin(NOME_RETE, "5 martelli (accende la ESP32 06_MARTELLI)");
}

void aggiornaPaginaWeb() {
  char testo[64];
  if (STATE == 2) {
    snprintf(testo, sizeof(testo), "Manuale (06 accesa, senza avvio)");
  } else if (cicloAttivo) {
    unsigned long passati = (millis() - tAccensione) / 1000UL;
    unsigned long massimo = TEMPO_MAX_CICLO_MS / 1000UL;
    snprintf(testo, sizeof(testo), "Ciclo in corso: %lu s di %lu s", passati, massimo);
  } else if (STATE == 1) {
    snprintf(testo, sizeof(testo), "Tempo massimo scaduto: girare il selettore");
  } else {
    snprintf(testo, sizeof(testo), "Ferma, in attesa di avvio");
  }
  paginaWeb.impostaStato(testo);
  paginaWeb.impostaSpia(0, pwnUscita(PIN_RELE_ESP));
  paginaWeb.impostaSpia(1, pwnUscita(PIN_AVVIO_ESP));
  paginaWeb.impostaSpia(2, cicloAttivo);
}
