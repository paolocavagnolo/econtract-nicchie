#include "FastAccelStepper.h"

#define BASE_SPEED  40*4
#define BASE_ACC    160*4

#define TOP_SPEED  500*4
#define TOP_ACC    5000*4

#define ZERO_OFFSET_BASE 5*4
#define ZERO_OFFSET_TOP 5*4

#define CORSA_BASE 50*4
#define CORSA_TOP 700*4

FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *BASE = NULL;
FastAccelStepper *TOP = NULL;

uint8_t STATE = 0; // 0 REMOTE - 1 AUTO - 2 MAN
unsigned long tSequenza = 0;

void setup() {

  delay(1000);

  // DEBUG
  Serial.begin(115200);
  delay(100);

  // DRIVER OUTPUT

  pinMode(21, OUTPUT);
  pinMode(27, OUTPUT);

  // FC

  pinMode(34, INPUT);
  pinMode(35, INPUT);

  disableMotor();

  engine.init();

  BASE = engine.stepperConnectToPin(25);

  if (BASE) {
      BASE->setDirectionPin(26);  
      BASE->setSpeedInHz(BASE_SPEED);  
      BASE->setAcceleration(BASE_ACC);

  }

  TOP = engine.stepperConnectToPin(18);

  if (TOP) {
      TOP->setDirectionPin(19, false);  
      TOP->setSpeedInHz(TOP_SPEED);  
      TOP->setAcceleration(TOP_ACC);   
  } 

  delay(1000);

  pinMode(4, INPUT);

  delay(1000);

  tSequenza = millis();

  enableMotor();
  goToHome();

}


int iS, old_iS;
bool goAgo = false;

#define TIME_ON 100000;
#define TIME_OFF 45000;

unsigned long tLoop = 0, tWait = TIME_ON;
int ago_counter = 0;
bool eL = true, old_eL = true;

unsigned long tDebug = 0;

void loop() {


  check_input();

  logic();

  if ((millis() - tLoop) > tWait) {
    tLoop = millis();

    if (eL) {
      tWait = TIME_OFF;
      eL = false;
    } else {
      eL = true;
      tWait = TIME_ON;
    }

  }

}

void check_input() {

  iS = !digitalRead(4);

  if ((iS != old_iS) || (eL != old_eL)) {

    if ((iS) && (eL)) {

      enableMotor();
      tSequenza = millis();
      goAgo = true;
      delay(300);

    } else {

      goAgo = false;

      goToHome();

      disableMotor();

    }

    old_iS = iS;
    old_eL = eL;
  }

}

void logic() {

  if (goAgo) {
    if (!BASE->isRunning()) {

      if (BASE->getCurrentPosition() == CORSA_BASE) {
        act_BASE(-CORSA_BASE,12);
      }
      else if (BASE->getCurrentPosition() == 0) {
        act_BASE(CORSA_BASE,12);
      } else {
        long pos_base = BASE->getCurrentPosition();
        act_BASE(0 - pos_base,12);
      }

    }

    if ((millis() - tSequenza) > 6000){
      if (!TOP->isRunning()) {

        if (TOP->getCurrentPosition() == CORSA_TOP) {
          act_TOP(-CORSA_TOP,12);
        }
        else if (TOP->getCurrentPosition() == 0) {
          act_TOP(CORSA_TOP,12);
        } else {
          long pos_top = TOP->getCurrentPosition();
          act_BASE(0 - pos_top,12);
        }

      }
    }
  }

}

void disableMotor() {
  delay(200);  // DA TESTARE
  digitalWrite(21, HIGH);
  digitalWrite(27, HIGH);
}

void enableMotor() {
  digitalWrite(21, LOW);
  digitalWrite(27, LOW);
  delay(200);  // DA TESTARE
}

void act_BASE(int32_t pos, float tempo) {

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

  long V_max, A_cal;

  float V_media = (float)abs(pos) / tempo;

  V_max = V_media * 2;   // step / s
  A_cal = V_max / (tempo / 2);   // step / s / s

  TOP->setSpeedInHz(V_max);  
  TOP->setAcceleration(A_cal);
  TOP->applySpeedAcceleration();

  TOP->move(pos);

}

void goToHome() {

  Serial.println("GOTOHOME");

  if ((BASE->isRunning()) || (TOP->isRunning())) {
    BASE->stopMove();
    TOP->stopMove();
    while ((BASE->isRunning()) || (TOP->isRunning())) {}
  }

  bool exit_flag_base = false;
  bool exit_flag_top = false;

  BASE->setSpeedInHz(BASE_SPEED/3);  
  BASE->setAcceleration(BASE_ACC/3);
  BASE->applySpeedAcceleration();

  TOP->setSpeedInHz(TOP_SPEED/2);  
  TOP->setAcceleration(TOP_ACC/2);
  TOP->applySpeedAcceleration();

  BASE->runBackward();

  unsigned long tFC = millis();

  while (!exit_flag_base) {

    if (digitalRead(34)) {
      BASE->forceStop();
      exit_flag_base = true;
      Serial.println("OK BASE");
    }

    if ((millis() - tFC) > 15000) {
      BASE->forceStop();
      exit_flag_base = true;
      Serial.println("TIMEOUT BASE");
    }

  }

  TOP->runBackward();
  tFC = millis();

  while (!exit_flag_top) {

    if (digitalRead(35)) {
      TOP->forceStop();
      exit_flag_top = true;
      Serial.println("OK TOP");
    }

    if ((millis() - tFC) > 15000) {
      TOP->forceStop();
      exit_flag_top = true;
      Serial.println("TIMEOUT TOP");
    }
    

  }

  delay(100);

  BASE->move(ZERO_OFFSET_BASE);
  TOP->move(ZERO_OFFSET_TOP);

  while ((BASE->isRunning()) || (TOP->isRunning())) {}

  BASE->setCurrentPosition(0);
  TOP->setCurrentPosition(0);

  BASE->setSpeedInHz(BASE_SPEED);  
  BASE->setAcceleration(BASE_ACC);
  BASE->applySpeedAcceleration();

  TOP->setSpeedInHz(TOP_SPEED);  
  TOP->setAcceleration(TOP_ACC);
  TOP->applySpeedAcceleration();
}
