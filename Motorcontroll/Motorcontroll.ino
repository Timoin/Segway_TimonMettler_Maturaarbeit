#include <Servo.h>
 
Servo esc1;
Servo esc2;
int poti = 0;
int motor;
 
void setup()
{
Serial.begin( 115200);
  
esc1.attach(9);
esc2.attach(8);
}
 
void loop()
{
//reading poti value
int motor = analogRead(poti);
//Poti values 0-1023,
//ESC values 0-180
motor = map(motor, 0, 1023, 0 , 178);

//sending motor value to ESCs
esc1.write(motor);
esc2.write(motor+2);

//Showing on Serial Monitor the motor value
Serial.println(motor);
}
