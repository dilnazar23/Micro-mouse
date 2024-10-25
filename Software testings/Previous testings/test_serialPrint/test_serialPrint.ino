void setup()
{
  Serial.begin(9600);
}

void dataprint(char *parameterName, float dataNum, char *unitName)
{
  Serial.print(parameterName);
  Serial.print(",");
  Serial.print(dataNum);
  Serial.print(",");
  Serial.println(unitName);
}

void loop() {
  char *temp = "temperature";
  float t = 123.4;
  char *unit = "C";
  dataprint(temp,t,unit);

  while(1)
  {
    //do nothing
  }
}
