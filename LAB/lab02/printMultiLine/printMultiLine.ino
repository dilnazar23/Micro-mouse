// Print and label multiple lines in the serial plotter
void setup() {
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  for (int i=0; i<360; i++){
    Serial.print(i);
  }
}
