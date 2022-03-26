int pin = 3;

void setup() {
  Serial.begin(9600);
}

void loop() {

  delay(5000);
  digitalWrite(pin, HIGH);
  delay(5000);
  digitalWrite(pin, LOW);
}
