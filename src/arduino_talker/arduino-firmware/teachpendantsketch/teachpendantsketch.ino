

void setup() {
  // put your setup code here, to run once:
   pinMode(3, INPUT_PULLUP);
   Serial.begin(115200);

}

void loop() {
  // put your main code here, to run repeatedly:
  delay(20);
  if (digitalRead(3) == LOW) {
    Serial.print(1);
  }else{
    Serial.print(0);
  }
}
