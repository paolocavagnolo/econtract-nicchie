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

uint8_t STATE = 0; // 0 REMOTE - 1 AUTO - 2 MAN

void setup() {

  delay(100);

  // DEBUG
  Serial.begin(115200);
  delay(100);

  // LED OFF
  pinMode(37, INPUT);

  // PANEL INPUT
  pinMode(ADIO1, INPUT);  // AUTO
  pinMode(ADIO2, INPUT);  // MAN

  // ESP32 OUTPUT
  pinMode(DIO9, OUTPUT); // relè accensione
  pinMode(DIO10, OUTPUT); // segnale avvio


  // BUZZER 
  ledcSetup(0, 1000, 8);
  ledcAttachPin(BUZZ, 0);

  delay(1000);

  // INITIAL STATE
  if (digitalRead(ADIO2)) {
    STATE = 2;
  } else {
    if (digitalRead(ADIO1)) {
      STATE = 1;
    } else {
      STATE = 0;
    }
  }

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

unsigned long t_MAN = 0, tS_MAN = 0;
bool s_MAN = false, p_MAN = true, f_MAN = false;
unsigned long t_AUTO = 0, tS_AUTO = 0;
bool s_AUTO = false, p_AUTO = true, f_AUTO = false;
bool f_REM = false;

void loop() {

  check_MAN();
  check_AUTO();

  check_STATE();

}

void check_STATE() {

  if (f_MAN) {

    f_MAN = false;
    STATE = 2;

    digitalWrite(DIO9, HIGH); // rele
    digitalWrite(DIO10, LOW);  // segnale

  } else if ((f_AUTO) and (STATE != 2)) {

    f_AUTO = false;
    STATE = 1;

    Serial.println("INIZIO LOOP");

    digitalWrite(DIO9, HIGH); // rele
    delay(1500);
    digitalWrite(DIO10, HIGH); // segnale

  } else if (f_REM) {
    f_REM = false;
    STATE = 0;

    if (digitalRead(ADIO1)) {

      STATE = 1;
      f_AUTO = true;

    } else {

      STATE = 0;
      digitalWrite(DIO10, LOW);
      delay(4000);
      digitalWrite(DIO9, LOW);

    }

  }

}

void check_MAN() {
  if (!s_MAN) {
    if ((millis() - t_MAN) > 200) {
      if (digitalRead(ADIO2)) {
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
      if (!digitalRead(ADIO2)) {
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
      if (digitalRead(ADIO1)) {
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
      if (!digitalRead(ADIO1)) {
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

