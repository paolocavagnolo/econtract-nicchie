// // GAMMA 2.5
const uint8_t gamma8[] = {
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
  1, 2, 2, 2, 2, 2, 2, 2, 2, 3, 3, 3, 3, 3, 4, 4,
  4, 4, 4, 5, 5, 5, 5, 6, 6, 6, 6, 7, 7, 7, 7, 8,
  8, 8, 9, 9, 9, 10, 10, 10, 11, 11, 12, 12, 12, 13, 13, 14,
  14, 15, 15, 15, 16, 16, 17, 17, 18, 18, 19, 19, 20, 20, 21, 22,
  22, 23, 23, 24, 25, 25, 26, 26, 27, 28, 28, 29, 30, 30, 31, 32,
  33, 33, 34, 35, 36, 36, 37, 38, 39, 40, 40, 41, 42, 43, 44, 45,
  46, 46, 47, 48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60,
  61, 62, 63, 64, 65, 67, 68, 69, 70, 71, 72, 73, 75, 76, 77, 78,
  80, 81, 82, 83, 85, 86, 87, 89, 90, 91, 93, 94, 95, 97, 98, 99,
  101, 102, 104, 105, 107, 108, 110, 111, 113, 114, 116, 117, 119, 121, 122, 124,
  125, 127, 129, 130, 132, 134, 135, 137, 139, 141, 142, 144, 146, 148, 150, 151,
  153, 155, 157, 159, 161, 163, 165, 166, 168, 170, 172, 174, 176, 178, 180, 182,
  184, 186, 189, 191, 193, 195, 197, 199, 201, 204, 206, 208, 210, 212, 215, 217,
  219, 221, 224, 226, 228, 231, 233, 235, 238, 240, 243, 245, 248, 250, 253, 255
};

#define FASTLED_ALLOW_INTERRUPTS 0
#include <FastLED.h>
FASTLED_USING_NAMESPACE
 
#define DATA_PIN            7
#define NUM_LEDS_A          168
#define NUM_LEDS_B          146
#define LED_TYPE            WS2812
#define COLOR_ORDER         GRB
 
CRGB ledsA[NUM_LEDS_A];
CRGB ledsB[NUM_LEDS_B];


void setup() {
  delay( 3000); // 3 second delay for boot recovery, and a moment of silence
  FastLED.addLeds<LED_TYPE,7,COLOR_ORDER>(ledsA, NUM_LEDS_A);
  FastLED.addLeds<LED_TYPE,8,COLOR_ORDER>(ledsB, NUM_LEDS_B);

  FastLED.setBrightness(255);
  delay(100);
  FastLED.clear();
  FastLED.show();
  delay(100);

  FastLED.show();

}

CRGB whiteA = CRGB(255,200,140);
CRGB whiteB = CRGB(200,200,210);

unsigned long tSnakeA = 0;
int indexA = 0;
bool dirA = true;

unsigned long tSnakeB = 0;
int indexB = 0;
bool dirB = true;

unsigned long tFPS = 0;
 
int S = 0;
bool P = true;
bool E = false;
unsigned long T = 0;
int C = 0;

void loop()
{

	if (S == 0) {

		if (P) {
			P = false;
			E = false;
			T = millis();
			C = 0;
			FastLED.clear();
			indexA = 0;
			indexB = 0;
		}

		if ((millis() - tSnakeA) > 30) {
			tSnakeA = millis();
			ledsA[indexA] = whiteA;
			fadeToBlackBy(ledsA,NUM_LEDS_A,20);
			indexA++;
			if (indexA >= NUM_LEDS_A - 1) {
				indexA = 0;
			}
		}

		if ((millis() - tSnakeB) > 10) {
			tSnakeB = millis();
			ledsB[indexB] = whiteB;
			fadeToBlackBy(ledsB,NUM_LEDS_B,20);
			indexB++;
			if (indexB >= NUM_LEDS_B - 1) {
				indexB = 0;
				C++;
			}
		}

		if (C >= 5) {
			E = true;
		}

		if (E) {
			E = false;
			P = true;
			S = 1;
		}

	} else if (S == 1) {

		if (P) {
			P = false;
			E = false;
			T = millis();
			FastLED.clear();
			indexB = 0;
			indexA = 0;
			C = 0;
		}

		if ((millis() - tSnakeA) > 5) {
			tSnakeA = millis();
			fill_solid(ledsA,NUM_LEDS_A,blend(CRGB(0,0,0),whiteA,indexA));
			if (dirA) {
				indexA++;
				if (indexA >= 255) {
					dirA = false;
					indexA = 255;
					C++;
				}
			} else {
				indexA--;
				if (indexA <= 0) {
					dirA = true;
					indexA = 0;
				}
			}
			
		}

		if ((millis() - tSnakeB) > 8) {
			tSnakeB = millis();
			fill_solid(ledsB,NUM_LEDS_B,blend(CRGB(0,0,0),whiteB,indexB));
			if (dirB) {
				indexB++;
				if (indexB >= 255) {
					dirB = false;
					indexB = 255;
				}
			} else {
				indexB--;
				if (indexB <= 0) {
					dirB = true;
					indexB = 0;
				}
			}
			
		}

		if (C>=5) {
			E = true;
		}

		if (E) {
			E = false;
			P = true;
			S = 0;
		}
	}

	if ((millis() - tFPS) > 1000/60) {
		tFPS = millis();
		FastLED.show();
	}


}
