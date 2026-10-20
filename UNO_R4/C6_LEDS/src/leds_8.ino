#define FASTLED_ALLOW_INTERRUPTS 0
#include <FastLED.h>
#define FASTLED_USING_NAMESPACE
 
#define LED_TYPE    WS2812B
#define COLOR_ORDER GRB

#define NUM_STRIP 8

#define MARIUS_PIN 4
#define AUDIO_PIN 5

#define NUM_LEDS_1    51      // ok
#define NUM_LEDS_2    134     // ok
#define NUM_LEDS_3    164     // dal 17 al 6
#define NUM_LEDS_4    161     // dal 18 al 7
#define NUM_LEDS_5    160     // dal 19 al 8
#define NUM_LEDS_6    159     // dal 7 al 9
#define NUM_LEDS_7    164     // dal 8 al 10
#define NUM_LEDS_8    72      // dal 9 al 11

CRGB leds_1[NUM_LEDS_1];
CRGB leds_2[NUM_LEDS_2];
CRGB leds_3[NUM_LEDS_3];
CRGB leds_4[NUM_LEDS_4];
CRGB leds_5[NUM_LEDS_5];
CRGB leds_6[NUM_LEDS_6];
CRGB leds_7[NUM_LEDS_7];
CRGB leds_8[NUM_LEDS_8];

CRGB *leds[] = {&leds_1[0],
                &leds_2[0],
                &leds_3[0],
                &leds_4[0],
                &leds_5[0],
                &leds_6[0],
                &leds_7[0],
                &leds_8[0]}; 

uint8_t NUM_LEDS[] = {NUM_LEDS_1,
                      NUM_LEDS_2,
                      NUM_LEDS_3,
                      NUM_LEDS_4,
                      NUM_LEDS_5,
                      NUM_LEDS_6,
                      NUM_LEDS_7,
                      NUM_LEDS_8};

void setup() {

  delay(300); // 300 ms delay for boot recovery

  Serial.begin(115200);
  delay(300);

  FastLED.addLeds<LED_TYPE,2,COLOR_ORDER>(leds_1, NUM_LEDS_1).setTemperature(Candle);
  FastLED.addLeds<LED_TYPE,3,COLOR_ORDER>(leds_2, NUM_LEDS_2).setTemperature(Candle);
  FastLED.addLeds<LED_TYPE,6,COLOR_ORDER>(leds_3, NUM_LEDS_3).setTemperature(Candle);
  FastLED.addLeds<LED_TYPE,7,COLOR_ORDER>(leds_4, NUM_LEDS_4).setTemperature(Candle);
  FastLED.addLeds<LED_TYPE,8,COLOR_ORDER>(leds_5, NUM_LEDS_5).setTemperature(Candle);
  FastLED.addLeds<LED_TYPE,9,COLOR_ORDER>(leds_6, NUM_LEDS_6).setTemperature(Candle);
  FastLED.addLeds<LED_TYPE,10,COLOR_ORDER>(leds_7, NUM_LEDS_7).setTemperature(Candle);
  FastLED.addLeds<LED_TYPE,11,COLOR_ORDER>(leds_8, NUM_LEDS_8).setTemperature(Candle);

  FastLED.setBrightness(255);

  delay(100);

  pinMode(MARIUS_PIN, INPUT);

  pinMode(AUDIO_PIN, OUTPUT);   // AUDIO PIN
  digitalWrite(AUDIO_PIN, LOW);

}


unsigned long tLed[NUM_STRIP]{0};
int cVal[NUM_STRIP]{0};
bool dir[NUM_STRIP]{true};
bool ena[NUM_STRIP]{false};
int indexVal[NUM_STRIP]{0};

unsigned long tSequence = 0;
unsigned long tShow = 0;

int idx = 0;

uint8_t seq_figure[] = {0,1,2,3,4,5,6,7};

bool prima_volta = true;
bool prima_uscita = true;

unsigned long tSpento = 0;
int next_fig = 0; 
bool check_next_fig[NUM_STRIP]{true};
 
void loop()
{
  
  if (digitalRead(MARIUS_PIN)) {
  //if (true) {
    if (prima_volta) {
      prima_volta = false;
      prima_uscita = true;
      random_sequence();
      digitalWrite(AUDIO_PIN, HIGH);
    }

    sequence_snake();

    if ((millis() - tShow) > 40) {
      tShow = millis();
      FastLED.show();
    }
  } else {

    if (prima_uscita) {
      prima_uscita = false;
      prima_volta = true;
      tSpento = millis();
      digitalWrite(AUDIO_PIN, LOW);
    }

    if ((millis() - tSpento) < 5000) {
      if ((millis() - tShow) > 50) {
        tShow = millis();
        for (uint8_t i=0; i<NUM_STRIP; i++) {
           fadeToBlackBy(leds[i], NUM_LEDS[i], 64);
        }
        FastLED.show();
      }
    } else {
      for (uint8_t i=0; i<NUM_STRIP; i++) {
         fill_solid(leds[i], NUM_LEDS[i], CRGB::Black);
      }
      FastLED.show();
      delay(200);
    }

  }
  

}

void random_sequence() {

  for (int8_t i=NUM_STRIP - 1; i>0; i--) {
    uint8_t j=random(i+1);

    uint8_t temp = seq_figure[i];
    seq_figure[i] = seq_figure[j];
    seq_figure[j] = temp;
  }

}


void sequence_pulse() {

  if ((millis() - tSequence) > 1800) {
    tSequence = millis();

    uint8_t n = seq_figure[idx];
    Serial.print("accendo ");
    Serial.println(n);

    ena[n] = true;
    dir[n] = true;
    cVal[n] = 0;
    indexVal[n] = 0;

    idx++;

    if (idx >= NUM_STRIP) {
      random_sequence();
      idx = 0;
    }

  }


  for (uint8_t i=0; i<NUM_STRIP; i++) {

    if (ena[i]) {
      if ((millis() - tLed[i]) > 16) {
        tLed[i] = millis();

        if (dir[i]) {
          cVal[i]+=3;
          if (cVal[i]>255) {
            cVal[i]=255;
            dir[i] = false;
            Serial.print("apice ");
            Serial.println(i);
          }
        } else {
          cVal[i]-=3;
          if (cVal[i]<0) {
            cVal[i]=0;
            ena[i]=false;
            dir[i]=true;
            Serial.print("Spengo ");
            Serial.println(i);
          }
        }

        fill_solid(leds[i],NUM_LEDS[i],CRGB(cVal[i],cVal[i],cVal[i]));
      }
    }
  }

}

void sequence_snake() {

  if (next_fig < 2) {
    next_fig++;
    
    uint8_t n = seq_figure[idx];
    Serial.print("accendo ");
    Serial.println(n);

    ena[n] = true;
    dir[n] = true;
    cVal[n] = 0;
    indexVal[n] = 0;
    check_next_fig[n] = true;

    idx++;

    if (idx >= NUM_STRIP) {
      random_sequence();
      idx = 0;
    }

  }


  for (uint8_t i=0; i<NUM_STRIP; i++) {

    if (ena[i]) {
      if ((millis() - tLed[i]) > 32) {
        tLed[i] = millis();

        leds[i][indexVal[i]] = CRGB::White;
        fadeToBlackBy(leds[i],NUM_LEDS[i],20);
        indexVal[i]++;
        if (indexVal[i] >= NUM_LEDS[i]) {
          indexVal[i] = 0;
          ena[i] = false;
          Serial.print("spengo ");
          Serial.println(i);
          fill_solid(leds[i],NUM_LEDS[i],CRGB::Black);
        }
        else if (check_next_fig[i]) {
          if (indexVal[i] >= NUM_LEDS[i] * 0.7) {
            next_fig = 0;
            check_next_fig[i] = false;
            Serial.print("3/4 ");
            Serial.println(i);
          }
        }
      }
    }
  }

}

