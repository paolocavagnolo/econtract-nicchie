// =====================================================
// C6_LEDS (scheda A) - Arduino UNO R4 Minima
// Gioco "serpente": una cometa bianca con scia percorre una striscia;
// quando arriva al 70% partono altre due strisce, in ordine casuale.
// Il gioco gira finche' il segnale MARIUS (pin 4) e' alto.
// Questa scheda accende anche il comando AUDIO (pin 5) durante il gioco.
// =====================================================

#define FASTLED_ALLOW_INTERRUPTS 0
#include <FastLED.h>

// ---- DEBUG ----
// Metti 1 per vedere i messaggi sul monitor seriale (rallenta un po' il gioco)
#define DEBUG 0
#if DEBUG
  #define DBG(testo, numero) { Serial.print(testo); Serial.println(numero); }
#else
  #define DBG(testo, numero)
#endif

// ---- NOMI DEGLI INGRESSI/USCITE ----
#define PIN_MARIUS     4    // ingresso: avvio gioco
#define PIN_AUDIO      5    // uscita: avvio audio
#define PIN_STRISCIA_1  2
#define PIN_STRISCIA_2  3
#define PIN_STRISCIA_3  6
#define PIN_STRISCIA_4  7
#define PIN_STRISCIA_5  8
#define PIN_STRISCIA_6  9
#define PIN_STRISCIA_7  10
#define PIN_STRISCIA_8  11

// ---- STRISCE ----
#define LED_TYPE    WS2812B
#define COLOR_ORDER GRB
#define NUM_STRIP   8
#define NUM_LEDS_1  51   // ok
#define NUM_LEDS_2  134  // ok
#define NUM_LEDS_3  164  // dal 17 al 6
#define NUM_LEDS_4  161  // dal 18 al 7
#define NUM_LEDS_5  160  // dal 19 al 8
#define NUM_LEDS_6  159  // dal 7 al 9
#define NUM_LEDS_7  164  // dal 8 al 10
#define NUM_LEDS_8  72   // dal 9 al 11

// ---- TEMPI E GIOCO ----
#define FRAME_MS             33UL    // ms tra un passo del serpente e il successivo (stessa velocita' di prima)
#define FRAME_SFUMATURA_MS   50UL    // ms tra un passo di spegnimento e l'altro a fine gioco
#define TEMPO_SFUMATURA_MS   5000UL  // ms di spegnimento graduale a fine gioco
#define RINFRESCO_SPENTO_MS  1000UL  // ms: ogni quanto si rimanda il "tutto spento" a riposo
#define SCIA_FADE            20      // quanto si accorcia la scia a ogni passo
#define SPEGNIMENTO_FADE     64      // quanto si spegne a ogni passo a fine gioco
#define PASSI_CODA           80      // passi per far sparire la scia a fine striscia (~2,6 s)
#define SOGLIA_PERCENTO      70      // % del percorso a cui partono le strisce successive
#define STRISCE_PER_VOLTA    2       // strisce che partono insieme

// ---- VARIABILI GLOBALI ----
CRGB leds_1[NUM_LEDS_1];
CRGB leds_2[NUM_LEDS_2];
CRGB leds_3[NUM_LEDS_3];
CRGB leds_4[NUM_LEDS_4];
CRGB leds_5[NUM_LEDS_5];
CRGB leds_6[NUM_LEDS_6];
CRGB leds_7[NUM_LEDS_7];
CRGB leds_8[NUM_LEDS_8];

CRGB *leds[NUM_STRIP] = {leds_1, leds_2, leds_3, leds_4, leds_5, leds_6, leds_7, leds_8};
const uint16_t NUM_LEDS[NUM_STRIP] = {NUM_LEDS_1, NUM_LEDS_2, NUM_LEDS_3, NUM_LEDS_4, NUM_LEDS_5, NUM_LEDS_6, NUM_LEDS_7, NUM_LEDS_8};

uint16_t indexVal[NUM_STRIP] = {0};        // posizione della testa del serpente
uint16_t soglia[NUM_STRIP] = {0};          // LED a cui partono le strisce successive
bool ena[NUM_STRIP] = {false};             // striscia con serpente in movimento
bool check_next_fig[NUM_STRIP] = {false};  // soglia del 70% ancora da raggiungere
uint8_t codaPassi[NUM_STRIP] = {0};        // passi rimasti per spegnere la scia

uint8_t seq_figure[NUM_STRIP];  // ordine (casuale) delle strisce
uint8_t idx = 0;                // prossima striscia della sequenza
uint8_t next_fig = STRISCE_PER_VOLTA;  // strisce gia' lanciate in questo gruppo

bool in_gioco = false;
bool seme_fatto = false;
bool spento_inviato = false;
unsigned long tFrame = 0;
unsigned long tSpento = 0;
unsigned long tRinfresco = 0;

// ---- PROTOTIPI ----
void avviaGioco();
void fermaGioco();
void random_sequence();
void lanciaStriscia();
void sequence_snake();

// =====================================================
void setup() {
  delay(300);  // attesa per stabilizzare l'alimentazione

#if DEBUG
  Serial.begin(115200);
  delay(300);
#endif

  FastLED.addLeds<LED_TYPE, PIN_STRISCIA_1, COLOR_ORDER>(leds_1, NUM_LEDS_1).setTemperature(Candle);
  FastLED.addLeds<LED_TYPE, PIN_STRISCIA_2, COLOR_ORDER>(leds_2, NUM_LEDS_2).setTemperature(Candle);
  FastLED.addLeds<LED_TYPE, PIN_STRISCIA_3, COLOR_ORDER>(leds_3, NUM_LEDS_3).setTemperature(Candle);
  FastLED.addLeds<LED_TYPE, PIN_STRISCIA_4, COLOR_ORDER>(leds_4, NUM_LEDS_4).setTemperature(Candle);
  FastLED.addLeds<LED_TYPE, PIN_STRISCIA_5, COLOR_ORDER>(leds_5, NUM_LEDS_5).setTemperature(Candle);
  FastLED.addLeds<LED_TYPE, PIN_STRISCIA_6, COLOR_ORDER>(leds_6, NUM_LEDS_6).setTemperature(Candle);
  FastLED.addLeds<LED_TYPE, PIN_STRISCIA_7, COLOR_ORDER>(leds_7, NUM_LEDS_7).setTemperature(Candle);
  FastLED.addLeds<LED_TYPE, PIN_STRISCIA_8, COLOR_ORDER>(leds_8, NUM_LEDS_8).setTemperature(Candle);
  FastLED.setBrightness(255);
  delay(100);

  pinMode(PIN_MARIUS, INPUT);
  pinMode(PIN_AUDIO, OUTPUT);
  digitalWrite(PIN_AUDIO, LOW);

  for (uint8_t i = 0; i < NUM_STRIP; i++) {
    seq_figure[i] = i;
    soglia[i] = (uint32_t)NUM_LEDS[i] * SOGLIA_PERCENTO / 100;
  }

  // all'accensione: come prima, 5 s di spegnimento poi tutto nero
  tSpento = millis();
}

// =====================================================
void loop() {
  unsigned long ora = millis();

  //if (digitalRead(PIN_MARIUS)) {
  if (true) {
    if (!in_gioco) {
      avviaGioco();
    }
    // un passo del serpente e un invio ai LED, sempre allo stesso ritmo
    if (ora - tFrame >= FRAME_MS) {
      tFrame = ora;
      sequence_snake();
      FastLED.show();
    }
  } else {
    if (in_gioco) {
      fermaGioco();
    }
    if (ora - tSpento < TEMPO_SFUMATURA_MS) {
      // spegnimento graduale
      if (ora - tFrame >= FRAME_SFUMATURA_MS) {
        tFrame = ora;
        for (uint8_t i = 0; i < NUM_STRIP; i++) {
          fadeToBlackBy(leds[i], NUM_LEDS[i], SPEGNIMENTO_FADE);
        }
        FastLED.show();
      }
    } else if (!spento_inviato || (ora - tRinfresco >= RINFRESCO_SPENTO_MS)) {
      // a riposo: tutto nero, rimandato ogni tanto senza bloccare la scheda
      spento_inviato = true;
      tRinfresco = ora;
      FastLED.clear();
      FastLED.show();
    }
  }
}

// =====================================================
// Parte un nuovo gioco: strisce pulite, ordine nuovo, due serpenti subito
void avviaGioco() {
  in_gioco = true;

  // seme casuale vero: dipende dal momento esatto in cui arriva l'avvio
  if (!seme_fatto) {
    randomSeed(micros() ^ analogRead(A0));
    seme_fatto = true;
  }

  for (uint8_t i = 0; i < NUM_STRIP; i++) {
    ena[i] = false;
    check_next_fig[i] = false;
    codaPassi[i] = 0;
    indexVal[i] = 0;
  }
  FastLED.clear();

  random_sequence();
  idx = 0;
  next_fig = 0;
  tFrame = millis() - FRAME_MS;  // primo passo subito
  digitalWrite(PIN_AUDIO, HIGH);
  DBG("avvio gioco", "");
}

// Fine gioco: parte lo spegnimento graduale
void fermaGioco() {
  in_gioco = false;
  spento_inviato = false;
  tSpento = millis();
  digitalWrite(PIN_AUDIO, LOW);
  DBG("fine gioco", "");
}

// Mescola l'ordine delle strisce
void random_sequence() {
  for (int8_t i = NUM_STRIP - 1; i > 0; i--) {
    uint8_t j = random(i + 1);
    uint8_t temp = seq_figure[i];
    seq_figure[i] = seq_figure[j];
    seq_figure[j] = temp;
  }
}

// Fa partire il serpente sulla prossima striscia della sequenza
void lanciaStriscia() {
  uint8_t n = seq_figure[idx];
  DBG("accendo ", n);
  ena[n] = true;
  indexVal[n] = 0;
  check_next_fig[n] = true;
  codaPassi[n] = 0;

  idx++;
  if (idx >= NUM_STRIP) {
    random_sequence();
    idx = 0;
  }
}

// Un passo del gioco per tutte le strisce
void sequence_snake() {
  while (next_fig < STRISCE_PER_VOLTA) {
    next_fig++;
    lanciaStriscia();
  }

  for (uint8_t i = 0; i < NUM_STRIP; i++) {
    if (ena[i]) {
      // testa bianca + scia che sfuma
      leds[i][indexVal[i]] = CRGB::White;
      fadeToBlackBy(leds[i], NUM_LEDS[i], SCIA_FADE);
      indexVal[i]++;

      if (indexVal[i] >= NUM_LEDS[i]) {
        // fine striscia: la scia finisce di sfumare da sola
        indexVal[i] = 0;
        ena[i] = false;
        codaPassi[i] = PASSI_CODA;
        DBG("fine striscia ", i);
      } else if (check_next_fig[i] && indexVal[i] >= soglia[i]) {
        next_fig = 0;  // al prossimo passo partono altre strisce
        check_next_fig[i] = false;
        DBG("70% ", i);
      }
    } else if (codaPassi[i] > 0) {
      fadeToBlackBy(leds[i], NUM_LEDS[i], SCIA_FADE);
      codaPassi[i]--;
      if (codaPassi[i] == 0) {
        fill_solid(leds[i], NUM_LEDS[i], CRGB::Black);
      }
    }
  }
}
