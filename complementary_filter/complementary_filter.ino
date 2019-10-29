#include "I2Cdev.h"
#include "MPU6050.h"
#include "Wire.h"
MPU6050 accelgyro;



int16_t ax, ay, az;
int16_t gx, gy, gz;
float x, y, z;
double total_acc;
float ang, acc;
float angle;
long loop_timer;

void setup() {
    Wire.begin();
    Serial.begin(38400);

    //initializing MPU-6050
    accelgyro.initialize();


    //supplying offset values
    accelgyro.setXGyroOffset(220);
    accelgyro.setYGyroOffset(76);
    accelgyro.setZGyroOffset(-85);
    accelgyro.setZAccelOffset(1644);

    //timer
    loop_timer = micros();
}

void loop() {

    //getting values from MPU-6050
    accelgyro.getMotion6(&gx, &gy, &gz, &ax, &ay, &az);

    //complementary filter
    acc = ay*0.0000611;
    ang = 0.98*(gx*0.0000611)+0.02*(acc); //Angle in rad
    angle = 0.9*angle + 0.1*ang;


 while(micros() - loop_timer < 400);                                 //Wait until the loop_timer reaches 400us (2500Hz) before starting the next loop
 loop_timer = micros();//Reset the loop timer
}
