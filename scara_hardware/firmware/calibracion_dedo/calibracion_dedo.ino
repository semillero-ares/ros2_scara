#define HPulso  34
#define HDir    35
#define HSensor 16
#define HPeriod 50

long initial_time = 0;
long cont = 0;

void setup() {
  pinMode(HSensor, INPUT);
  pinMode(HPulso, OUTPUT);
  pinMode(HDir, OUTPUT);
  Serial.begin(9600);
  Serial.println("ON");
}

void loop() {
  digitalWrite(HDir, LOW);
  initial_time = millis();
  while (millis() - initial_time < 9000) {
    digitalWrite(HPulso, HIGH);
    delayMicroseconds(HPeriod);
    digitalWrite(HPulso, LOW);
    delayMicroseconds(HPeriod);
  }
  cont = 0;
  digitalWrite(HDir, HIGH);
  while (!digitalRead(HSensor)) {
    digitalWrite(HPulso, HIGH);
    delayMicroseconds(HPeriod);
    digitalWrite(HPulso, LOW);
    delayMicroseconds(HPeriod);
    cont++;
  }
  Serial.println(cont);
  delay(5000);
}
