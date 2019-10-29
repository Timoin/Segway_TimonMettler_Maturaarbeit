
#include "I2Cdev.h"
#include <PID_v1.h>
#include "MPU6050_6Axis_MotionApps20.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif
#define OUTPUT_READABLE_YAWPITCHROLL

MPU6050 mpu;


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

float Motor = 0.85;

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };

//=======================================================================
//=============================PID=======================================
double Setpoint= 0; 
double Kp = 22; //Set this first
double Kd = 1.5; //Set this secound
double Ki = 0; //Finally set this 


double Input, Output;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);


// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}


void Reverse (){ 
// Both motors running backwards
// "Motor" corrects the differently calibrated motors
    digitalWrite (12, HIGH); 
    digitalWrite(9, LOW);   
    analogWrite(3, Output*Motor);  


    digitalWrite(13, HIGH);  
    digitalWrite(8, LOW);   
    analogWrite(11, Output);
}
void Forward () { 
// Both motors running forward
    digitalWrite(12, LOW); 
    digitalWrite(9, LOW);   
    analogWrite(3, -Output*Motor);  


    digitalWrite(13, LOW);  
    digitalWrite(8, LOW);   
    analogWrite(11, -Output); 
}

void Stop (){ 
// Both Motors stop
    digitalWrite(12, HIGH); 
    digitalWrite(9, LOW);   
    analogWrite(3, 0*Motor);  


    digitalWrite(13, HIGH);  
    digitalWrite(8, LOW);   
    analogWrite(11, 0); 
}


// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
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

    // supply offset values
    mpu.setXGyroOffset(158);
    mpu.setYGyroOffset(29);
    mpu.setZGyroOffset(28);
    mpu.setZAccelOffset(1644);
    
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
        //setup PID
        myPID.SetMode(AUTOMATIC);
        myPID.SetOutputLimits(-255, 255); 
        myPID.SetTunings(Kp, Ki, Kd);
        

        // Setup Channel A
        pinMode(12, OUTPUT); //Initiates Motor Channel A pin
        pinMode(9, OUTPUT); //Initiates Brake Channel A pin

        // Setup Channel B
        pinMode(13, OUTPUT); //Initiates Motor Channel A pin
        pinMode(8, OUTPUT);  //Initiates Brake Channel A pin


        // Turn off the motors by default
        digitalWrite(12, LOW); 
        digitalWrite(9, LOW);    
        digitalWrite(13, LOW);  
        digitalWrite(8, LOW);    
}

void loop() {
    // if programming failed, don't try to do anything
    if (!dmpReady) return;

    // wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt && fifoCount < packetSize) {


     
// ======Calculating angle in degree========
        Input = ypr[1] * 180/M_PI-3.6;

// ======PID calculates "Output"======
        myPID.Compute();   

// ======Dies zeigt die Werte live in einem anderen Tab===================
        Serial.print(Input); Serial.print(" =>"); Serial.println(Output);

// Safety: if angle is between -30° and 30°
        if (Input>-30 && Input<30){ .

        if (Input>0) //falling forwards
        Forward(); //wheels turn forwards
        else if (Input<0) //falling backwards
        Reverse(); //wheels turn backwards
        }
        
        else //Angle is bigger than 30°
        Stop(); //both wheels stop
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
