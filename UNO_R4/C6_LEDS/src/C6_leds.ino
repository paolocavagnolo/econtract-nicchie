#define FASTLED_ALLOW_INTERRUPTS 0
#include <FastLED.h>
#define FASTLED_USING_NAMESPACE
 
#define LED_TYPE    WS2812B
#define COLOR_ORDER GRB

#define NUM_STRIP 15

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

  Serial.begin(9600);
  delay(300);

  FastLED.addLeds<LED_TYPE,4,COLOR_ORDER>(leds_1, NUM_LEDS_1).setTemperature(Candle);
  FastLED.addLeds<LED_TYPE,5,COLOR_ORDER>(leds_2, NUM_LEDS_2).setTemperature(Candle);
  FastLED.addLeds<LED_TYPE,18,COLOR_ORDER>(leds_3, NUM_LEDS_3).setTemperature(Candle);
  FastLED.addLeds<LED_TYPE,19,COLOR_ORDER>(leds_4, NUM_LEDS_4).setTemperature(Candle);
  FastLED.addLeds<LED_TYPE,20,COLOR_ORDER>(leds_5, NUM_LEDS_5).setTemperature(Candle);
  FastLED.addLeds<LED_TYPE,21,COLOR_ORDER>(leds_6, NUM_LEDS_6).setTemperature(Candle);
  FastLED.addLeds<LED_TYPE,22,COLOR_ORDER>(leds_7, NUM_LEDS_7).setTemperature(Candle);
  FastLED.addLeds<LED_TYPE,23,COLOR_ORDER>(leds_8, NUM_LEDS_8).setTemperature(Candle);
  FastLED.addLeds<LED_TYPE,25,COLOR_ORDER>(leds_9, NUM_LEDS_9).setTemperature(Candle);
  FastLED.addLeds<LED_TYPE,26,COLOR_ORDER>(leds_10, NUM_LEDS_10).setTemperature(Candle);
  FastLED.addLeds<LED_TYPE,27,COLOR_ORDER>(leds_11, NUM_LEDS_11).setTemperature(Candle);
  FastLED.addLeds<LED_TYPE,32,COLOR_ORDER>(leds_12, NUM_LEDS_12).setTemperature(Candle);
  FastLED.addLeds<LED_TYPE,33,COLOR_ORDER>(leds_13, NUM_LEDS_13).setTemperature(Candle);
  FastLED.addLeds<LED_TYPE,16,COLOR_ORDER>(leds_14, NUM_LEDS_14).setTemperature(Candle);
  FastLED.addLeds<LED_TYPE,17,COLOR_ORDER>(leds_15, NUM_LEDS_15).setTemperature(Candle);

}


unsigned long tLed[NUM_STRIP]{0};
int cVal[NUM_STRIP]{0};
bool dir[NUM_STRIP]{true};
bool ena[NUM_STRIP]{false};

unsigned long tSequence = 0;

int idx = 0;
 
void loop()
{

  // test_all_leds_red();

  // sequence();

}

void test_all_leds_red() {

  FastLED.setBrightness(100);

  fill_solid(leds_1, NUM_LEDS_1, CRGB::Red);
  fill_solid(leds_2, NUM_LEDS_2, CRGB::Red);
  fill_solid(leds_3, NUM_LEDS_3, CRGB::Red);
  fill_solid(leds_4, NUM_LEDS_4, CRGB::Red);
  fill_solid(leds_5, NUM_LEDS_5, CRGB::Red);
  fill_solid(leds_6, NUM_LEDS_6, CRGB::Red);
  fill_solid(leds_7, NUM_LEDS_7, CRGB::Red);
  fill_solid(leds_8, NUM_LEDS_8, CRGB::Red);
  fill_solid(leds_9, NUM_LEDS_9, CRGB::Red);
  fill_solid(leds_10, NUM_LEDS_10, CRGB::Red);
  fill_solid(leds_11, NUM_LEDS_11, CRGB::Red);
  fill_solid(leds_12, NUM_LEDS_12, CRGB::Red);
  fill_solid(leds_13, NUM_LEDS_13, CRGB::Red);
  fill_solid(leds_14, NUM_LEDS_14, CRGB::Red);
  fill_solid(leds_15, NUM_LEDS_15, CRGB::Red);

  delay(500);

  fill_solid(leds_1, NUM_LEDS_1, CRGB::Black);
  fill_solid(leds_2, NUM_LEDS_2, CRGB::Black);
  fill_solid(leds_3, NUM_LEDS_3, CRGB::Black);
  fill_solid(leds_4, NUM_LEDS_4, CRGB::Black);
  fill_solid(leds_5, NUM_LEDS_5, CRGB::Black);
  fill_solid(leds_6, NUM_LEDS_6, CRGB::Black);
  fill_solid(leds_7, NUM_LEDS_7, CRGB::Black);
  fill_solid(leds_8, NUM_LEDS_8, CRGB::Black);
  fill_solid(leds_9, NUM_LEDS_9, CRGB::Black);
  fill_solid(leds_10, NUM_LEDS_10, CRGB::Black);
  fill_solid(leds_11, NUM_LEDS_11, CRGB::Black);
  fill_solid(leds_12, NUM_LEDS_12, CRGB::Black);
  fill_solid(leds_13, NUM_LEDS_13, CRGB::Black);
  fill_solid(leds_14, NUM_LEDS_14, CRGB::Black);
  fill_solid(leds_15, NUM_LEDS_15, CRGB::Black);

  delay(500);

}

void sequence() {

  if ((millis() - tSequence) > 4000) {
    tSequence = millis();

    ena[idx] = true;
    dir[idx] = true;
    cVal[idx] = 0;

    idx++;

    if (idx >= 15) {
      idx = 0;
    }

  }


  for (uint8_t n=0; n<NUM_STRIP; n++) {
    if (ena[n]) {
      if ((millis() - tLed[n]) > 8) {
        tLed[n] = millis();

        if (dir[n]) {
          if (cVal[n] < 256) {
            cVal[n] = cVal[n] + 1;
          } else {
            cVal[n] = 255;
            dir[n] = !dir[n];
          }
        } else {
          if (cVal[n] > 0) {
            cVal[n] = cVal[n] - 1;
          } else {
            cVal[n] = 0;
            ena[n] = false;
          }
        }

        fill_solid(leds[n],NUM_LEDS[n],CRGB(cVal[n],cVal[n],cVal[n]));
      }
    }
  }

}

