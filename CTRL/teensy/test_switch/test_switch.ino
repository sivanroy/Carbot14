const int switchPin = 2;         // the number of the switch pin

// variables will change:
int switchState = 0;         // variable for reading the pushbutton status


void setup() {
  Serial.begin(57600);
  // initialize the pushbutton pin as an input:
  pinMode(switchPin, INPUT_PULLUP);
}

void loop() {
  // read the state of the pushbutton value:
  switchState = digitalRead(switchPin);
  /*
  if (Serial.available() > 0) {
    String data = Serial.readStringUntil('\n');
    Serial.print("pi sent me: ");
    Serial.println(data);
  }
  */
  
  // check if the switch is pressed. If it is, the switchState is HIGH:
  if (switchState == HIGH) {
    Serial.println("Switch is ON");
  }
  
  else if (switchState == LOW) {
    Serial.println("Switch is OFF");
  }
  
  // check if RPi sent a message
}
