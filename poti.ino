int throttlePin = 0;
 
void setup()
{
  Serial.begin( 115200);

}
 
void loop()
{
int throttle = analogRead(throttlePin);

Serial.println(throttle);

}
