
#include "I2Cdev.h"
#include <PID_v1.h>
#include "MPU6050_6Axis_MotionApps20.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#include <Servo.h>
#endif

MPU6050 mpu;


#define OUTPUT_READABLE_YAWPITCHROLL

bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };

// ============================BLDC=======================================

Servo esc1;
Servo esc2;

// =======================================================================
// =============================PID=======================================
// Create variables for the PID
double Setpoint= 0; 
double Kp = 0;
double Kd = 0;
double Ki = 0;


double Input, Output;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

// ========================STEERING============================
int steeringPin = 0;
int steering;
int steering2;

// =======================Options===========================
float millis_button;
int mil = 0;
int Abweichung = 1.3;
int modi = 1;


// =============INTERRUPT DETECTION ROUTINE=================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
    }

// Both Motors running backwards
void Reverse (){
    esc1.write(Output+77-steering2);
    esc2.write(Output+3+77+steering2);
}

 // Both Motors running forwards
void Forward () {
    esc1.write(Output+77-steering2*modi);
    esc2.write(Output+3+77+steering2*modi);
}

// Both Motors stop
void Stop (){
    esc1.write(78);
    esc2.write(80);
}


// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {
    // =========================Modusbutton=================================
    pinMode(4, INPUT_PULLUP);

    while (mil == 0){
      while (digitalRead(4) == 0){
       delay (100);
       millis_button = millis_button + 100;
       }
       // Boss Mode
      if ( millis_button >= 5000){
        Kp = 1.8;
        Kd = 0.1;
        Ki = 0.08;
        ++mil;
        }
        // Tricking Mode
       else if ( millis_button >= 3000){
        Kp = 1.6;
        Kd = 0.0;
        Ki = 0.00; 
        modi = -1;
        Abweichung = -1;
        ++mil;    
       }

       else if ( millis_button >= 1000){
        Kp = 1.6;
        Kd = 0.0;
        Ki = 0.05; 
        ++mil;    
       }
       // Normal Mode
       else if ( millis_button >= 100){
        Kp = 1.3;
        Kd = 0.0;
        Ki = 0.0; 
        ++mil;    
       }
    }
  
    // ========================BLDC=======================
    //Activates the Motors
    esc1.attach(9);
    esc2.attach(8);

    delay(1);
    esc1.write(78);
    esc2.write(80);
    delay(1);

    // ==========================Safety Button==================================
    pinMode(3, INPUT_PULLUP);

    // join I2C bus
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        TWBR = 24; //400kHz I2C clock (200kHz if CPU is 8MHz)
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif


    Serial.begin(115200);
    while (!Serial); // wait for Leonardo enumeration, others continue immediately

    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
    // wait for ready
    
    while (Serial.available() && Serial.read()); // empty buffer again

    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // Gyro Offset
    mpu.setXGyroOffset(118);
    mpu.setYGyroOffset(65);
    mpu.setZGyroOffset(15);
    mpu.setZAccelOffset(1630);

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        attachInterrupt(0, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }
        // setup PID
        myPID.SetMode(AUTOMATIC);
        myPID.SetOutputLimits(-77, 70);
        myPID.SetTunings(Kp, Ki, Kd);
}

void loop() {
    // if programming failed, don't try to do anything
    if (!dmpReady) return;

    // wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt && fifoCount < packetSize) {
      // Calculate angle in degrees
        Input = ypr[1] * 180/M_PI+Abweichung;
        
        // PID calculating values
        myPID.Compute();   

        // Print the value of Input and Output on serial monitor to check how it is working.
        Serial.print(Input); Serial.print(" =>"); Serial.println(Output);

//=============================SteeringPin==============================
        // Reading potentiometer data and saving it
        steering = analogRead(steeringPin);
        steering2 = (steering-335)*0.2;   

//========================Safety Button=========================
     int test = digitalRead(3);
     if( test == 0){ 
        
        if (Input>-30 && Input<30){// Safety if Bot is between 30 and -30 degrees
          
            if (Input>0) // Falling towards front 
            Forward(); // Rotate the wheels forward 
            else if (Input<0) // Falling towards back
            Reverse(); // Rotate the wheels backward 
            }
        else // If Bot has fallen
        Stop(); // Hold the wheels still
        // Reset the PID
        myPID.SetMode(AUTOMATIC);
        myPID.SetOutputLimits(-77, 70);
        myPID.SetTunings(Kp, Ki, Kd);
     }
     // if Safety Button is not pressed
     if( test == 1){
     Stop();
        }
     }

    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & 0x02) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;
        
        #ifdef OUTPUT_READABLE_YAWPITCHROLL
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        #endif
    


    }
}
