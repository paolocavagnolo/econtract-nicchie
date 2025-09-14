#define FASTLED_ALLOW_INTERRUPTS 0
#include <FastLED.h>
#define FASTLED_USING_NAMESPACE
 
#define LED_TYPE    WS2812B
#define COLOR_ORDER GRB

#define NUM_STRIP 15

#define MARIUS_PIN 34

#define NUM_LEDS_1    51
#define NUM_LEDS_2    134
#define NUM_LEDS_3    164
#define NUM_LEDS_4    161
#define NUM_LEDS_5    160
#define NUM_LEDS_6    159
#define NUM_LEDS_7    164
#define NUM_LEDS_8    72
#define NUM_LEDS_9    173
#define NUM_LEDS_10   176
#define NUM_LEDS_11   148
#define NUM_LEDS_12   148
#define NUM_LEDS_13   149
#define NUM_LEDS_14   152
#define NUM_LEDS_15   37

CRGB leds_1[NUM_LEDS_1];
CRGB leds_2[NUM_LEDS_2];
CRGB leds_3[NUM_LEDS_3];
CRGB leds_4[NUM_LEDS_4];
CRGB leds_5[NUM_LEDS_5];
CRGB leds_6[NUM_LEDS_6];
CRGB leds_7[NUM_LEDS_7];
CRGB leds_8[NUM_LEDS_8];
CRGB leds_9[NUM_LEDS_9];
CRGB leds_10[NUM_LEDS_10];
CRGB leds_11[NUM_LEDS_11];
CRGB leds_12[NUM_LEDS_12];
CRGB leds_13[NUM_LEDS_13];
CRGB leds_14[NUM_LEDS_14];
CRGB leds_15[NUM_LEDS_15];
CRGB leds_test[200];

CRGB *leds[] = {&leds_1[0],
                &leds_2[0],
                &leds_3[0],
                &leds_4[0],
                &leds_5[0],
                &leds_6[0],
                &leds_7[0],
                &leds_8[0],
                &leds_9[0],
                &leds_10[0],
                &leds_11[0],
                &leds_12[0],
                &leds_13[0],
                &leds_14[0],
                &leds_15[0]}; 

uint8_t NUM_LEDS[] = {NUM_LEDS_1,
                      NUM_LEDS_2,
                      NUM_LEDS_3,
                      NUM_LEDS_4,
                      NUM_LEDS_5,
                      NUM_LEDS_6,
                      NUM_LEDS_7,
                      NUM_LEDS_8,
                      NUM_LEDS_9,
                      NUM_LEDS_10,
                      NUM_LEDS_11,
                      NUM_LEDS_12,
                      NUM_LEDS_13,
                      NUM_LEDS_14,
                      NUM_LEDS_15};

void setup() {

  delay(300); // 3 second delay for boot recovery, and a moment of silence

  Serial.begin(115200);
  delay(300);

  FastLED.addLeds<LED_TYPE,4,COLOR_ORDER>(leds_1, NUM_LEDS_1).setTemperature(Tungsten100W);
  FastLED.addLeds<LED_TYPE,5,COLOR_ORDER>(leds_2, NUM_LEDS_2).setTemperature(Tungsten100W);
  FastLED.addLeds<LED_TYPE,13,COLOR_ORDER>(leds_3, NUM_LEDS_3).setTemperature(Tungsten100W);
  FastLED.addLeds<LED_TYPE,14,COLOR_ORDER>(leds_4, NUM_LEDS_4).setTemperature(Tungsten100W);
  FastLED.addLeds<LED_TYPE,15,COLOR_ORDER>(leds_5, NUM_LEDS_5).setTemperature(Tungsten100W);
  FastLED.addLeds<LED_TYPE,32,COLOR_ORDER>(leds_6, NUM_LEDS_6).setTemperature(Tungsten100W);
  FastLED.addLeds<LED_TYPE,33,COLOR_ORDER>(leds_7, NUM_LEDS_7).setTemperature(Tungsten100W);
  FastLED.addLeds<LED_TYPE,18,COLOR_ORDER>(leds_8, NUM_LEDS_8).setTemperature(Tungsten100W);
  FastLED.addLeds<LED_TYPE,19,COLOR_ORDER>(leds_9, NUM_LEDS_9).setTemperature(Tungsten100W);
  FastLED.addLeds<LED_TYPE,21,COLOR_ORDER>(leds_10, NUM_LEDS_10).setTemperature(Tungsten100W);
  FastLED.addLeds<LED_TYPE,22,COLOR_ORDER>(leds_11, NUM_LEDS_11).setTemperature(Tungsten100W);
  FastLED.addLeds<LED_TYPE,23,COLOR_ORDER>(leds_12, NUM_LEDS_12).setTemperature(Tungsten100W);
  FastLED.addLeds<LED_TYPE,25,COLOR_ORDER>(leds_13, NUM_LEDS_13).setTemperature(Tungsten100W);
  FastLED.addLeds<LED_TYPE,26,COLOR_ORDER>(leds_14, NUM_LEDS_14).setTemperature(Tungsten100W);
  FastLED.addLeds<LED_TYPE,27,COLOR_ORDER>(leds_15, NUM_LEDS_15).setTemperature(Tungsten100W);
  FastLED.addLeds<LED_TYPE,32,COLOR_ORDER>(leds_test, 200).setTemperature(Tungsten100W);

  FastLED.setBrightness(255);

  delay(100);

  pinMode(MARIUS_PIN, INPUT);
  fill_solid(leds_test, 200, CRGB::Red);

}


unsigned long tLed[NUM_STRIP]{0};
int cVal[NUM_STRIP]{0};
bool dir[NUM_STRIP]{true};
bool ena[NUM_STRIP]{false};
int indexVal[NUM_STRIP]{0};

unsigned long tSequence = 0;
unsigned long tShow = 0;

int idx = 0;

uint8_t seq_figure[] = {0,1,2,3,4,5,6,7,8,9,10,11,12,13,14};

bool prima_volta = true;
bool prima_uscita = true;

unsigned long tSpento = 0;
 
void loop()
{
  
  if (digitalRead(MARIUS_PIN)) {
  //if (true) {
    if (prima_volta) {
      prima_volta = false;
      prima_uscita = true;
      random_sequence();
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

  for (int8_t i=14; i>0; i--) {
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

    if (idx >= 15) {
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

  if ((millis() - tSequence) > 1800) {
    tSequence = millis();

    uint8_t n = seq_figure[idx];
    Serial.print("accendo ");
    Serial.println(n);

    ena[n] = true;
    dir[n] = true;
    cVal[n] = 0;

    idx++;

    if (idx >= 15) {
      random_sequence();
      idx = 0;
    }

  }


  for (uint8_t i=0; i<NUM_STRIP; i++) {

    if (ena[i]) {
      if ((millis() - tLed[i]) > 16) {
        tLed[i] = millis();

        leds[i][indexVal[i]] = CRGB::White;
        fadeToBlackBy(leds[i],NUM_LEDS[i],20);
        indexVal[i]++;
        if (indexVal[i] >= NUM_LEDS[i] - 1) {
          indexVal[i] = 0;
        }
      }
    }
  }

}

