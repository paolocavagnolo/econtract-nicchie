void setup()  
{
  delay(100);
  pinMode(7, OUTPUT);
  pinMode(6, OUTPUT);
}

int old_id, id, tempo, pausa;

void loop()
{   

  for (int i=30; i<255; i++) {
    analogWrite(7,i);
    delay(10);
  }
  digitalWrite(7,LOW);
  delay(250);
  digitalWrite(6,HIGH);
  delay(1000);
  digitalWrite(6,LOW);
  delay(random(500,5000));
  
}