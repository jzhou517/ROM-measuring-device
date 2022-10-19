// I2C device class (I2Cdev) demonstration Arduino sketch for MPU6050 class using DMP (MotionApps v2.0)
// 6/21/2012 by Jeff Rowberg <jeff@rowberg.net>
// Updates should (hopefully) always be available at https://github.com/jrowberg/i2cdevlib
//
// Changelog:
//      2019-07-08 - Added Auto Calibration and offset generator
//       - and altered FIFO retrieval sequence to avoid using blocking code
//      2016-04-18 - Eliminated a potential infinite loop
//      2013-05-08 - added seamless Fastwire support
//                 - added note about gyro calibration
//      2012-06-21 - added note about Arduino 1.0.1 + Leonardo compatibility error
//      2012-06-20 - improved FIFO overflow handling and simplified read process
//      2012-06-19 - completely rearranged DMP initialization code and simplification
//      2012-06-13 - pull gyro and accel data from FIFO packet instead of reading directly
//      2012-06-09 - fix broken FIFO read sequence and change interrupt detection to RISING
//      2012-06-05 - add gravity-compensated initial reference frame acceleration output
//                 - add 3D math helper file to DMP6 example sketch
//                 - add Euler output and Yaw/Pitch/Roll output formats
//      2012-06-04 - remove accel offset clearing for better results (thanks Sungon Lee)
//      2012-06-01 - fixed gyro sensitivity to be 2000 deg/sec instead of 250
//      2012-05-30 - basic DMP initialization working

/* ============================================
I2Cdev device library code is placed under the MIT license
Copyright (c) 2012 Jeff Rowberg
Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:
The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
===============================================
*/

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"
//#include "MPU6050.h" // not necessary if using MotionApps include file

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu(0x68);
MPU6050 mptwo(0x69); // <-- use for AD0 high

/* =========================================================================
   NOTE: In addition to connection 3.3v, GND, SDA, and SCL, this sketch
   depends on the MPU-6050's INT pin being connected to the Arduino's
   external interrupt #0 pin. On the Arduino Uno and Mega 2560, this is
   digital I/O pin 2.
 * ========================================================================= */

/* =========================================================================
   NOTE: Arduino v1.0.1 with the Leonardo board generates a compile error
   when using Serial.write(buf, len). The Teapot output uses this method.
   The solution requires a modification to the Arduino USBAPI.h file, which
   is fortunately simple, but annoying. This will be fixed in the next IDE
   release. For more info, see these links:
   http://arduino.cc/forum/index.php/topic,109987.0.html
   http://code.google.com/p/arduino/issues/detail?id=958
 * ========================================================================= */



// uncomment "OUTPUT_READABLE_QUATERNION" if you want to see the actual
// quaternion components in a [w, x, y, z] format (not best for parsing
// on a remote host such as Processing or something though)
#define OUTPUT_READABLE_QUATERNION

// uncomment "OUTPUT_READABLE_EULER" if you want to see Euler angles
// (in degrees) calculated from the quaternions coming from the FIFO.
// Note that Euler angles suffer from gimbal lock (for more info, see
// http://en.wikipedia.org/wiki/Gimbal_lock)
//#define OUTPUT_READABLE_EULER

// uncomment "OUTPUT_READABLE_YAWPITCHROLL" if you want to see the yaw/
// pitch/roll angles (in degrees) calculated from the quaternions coming
// from the FIFO. Note this also requires gravity vector calculations.
// Also note that yaw/pitch/roll angles suffer from gimbal lock (for
// more info, see: http://en.wikipedia.org/wiki/Gimbal_lock)
#define OUTPUT_READABLE_YAWPITCHROLL

// uncomment "OUTPUT_READABLE_REALACCEL" if you want to see acceleration
// components with gravity removed. This acceleration reference frame is
// not compensated for orientation, so +X is always +X according to the
// sensor, just without the effects of gravity. If you want acceleration
// compensated for orientation, us OUTPUT_READABLE_WORLDACCEL instead.
//#define OUTPUT_READABLE_REALACCEL

// uncomment "OUTPUT_READABLE_WORLDACCEL" if you want to see acceleration
// components with gravity removed and adjusted for the world frame of
// reference (yaw is relative to initial orientation, since no magnetometer
// is present in this case). Could be quite handy in some cases.
//#define OUTPUT_READABLE_WORLDACCEL

// uncomment "OUTPUT_TEAPOT" if you want output that matches the
// format used for the InvenSense teapot demo
//#define OUTPUT_TEAPOT



#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards
#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t mptwoIntStatus; 
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint8_t devStatus2;
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t packetSize2;
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion queasy;           // [w, x, y, z]         quaternion container
Quaternion queasy2;
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// vector multiplication vars 
VectorInt16 vec;
VectorInt16 crossvec;
VectorInt16 cross1;
VectorInt16 cross2;
VectorInt16 newvec; 
VectorInt16 negvec;
VectorInt16 quatvec;
VectorInt16 devicevec;
VectorInt16 forward0; 
VectorInt16 forward1;
VectorInt16 up0;
float neg_qx; 
float neg_qy; 
float neg_qz; 
float neg_qw;
float x_cross;
float y_cross;
float z_cross; 
float w_new;
float x_new;
float y_new;
float z_new; 
float vw = 0.0;
float dotproduct;
float angle;
float hinge_angle;
// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };



// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}



// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================
void init_mpu1(){
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);
    
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU1 connection successful") : F("MPU1 connection failed"));
    
    Serial.println(F("\nSend any character to begin DMP programming and demo: "));
    while (Serial.available() && Serial.read()); // empty buffer
    while (!Serial.available());                 // wait for data
    while (Serial.available() && Serial.read()); // empty buffer again
    
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();
    if (devStatus == 0) {
        // Calibration Time: generate offsets and calibrate our MPU6050
        mpu.CalibrateAccel(6);
        mpu.CalibrateGyro(6);
        mpu.PrintActiveOffsets();
        mpu.PrintActiveOffsets();
        
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);
        
        // enable Arduino interrupt detection
        Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
        Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
        Serial.println(F(")..."));
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
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
}

void init_mpu2(){
    Serial.println(F("Initializing I2C devices..."));
    mptwo.initialize();
    pinMode(INTERRUPT_PIN, INPUT);
    
    Serial.println(F("Testing device connections..."));
    Serial.println(mptwo.testConnection() ? F("MPU2 connection successful") : F("MPU2 connection failed"));
    
    Serial.println(F("\nSend any character to begin DMP programming and demo: "));
    while (Serial.available() && Serial.read()); // empty buffer
    while (!Serial.available());                 // wait for data
    while (Serial.available() && Serial.read()); // empty buffer again
    Serial.println(F("Initializing DMP..."));
    devStatus = mptwo.dmpInitialize();
    
    if (devStatus == 0) {
        // Calibration Time: generate offsets and calibrate our MPU6050
        
        mptwo.CalibrateAccel(6);
        mptwo.CalibrateGyro(6);
        mptwo.PrintActiveOffsets();
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        
        mptwo.setDMPEnabled(true);
        
        // enable Arduino interrupt detection
        Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
        Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
        Serial.println(F(")..."));
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mptwoIntStatus = mptwo.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize2 = mptwo.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }

    
}
void setup() {
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif


    // initialize serial communication
    Serial.begin(115200);
    while (!Serial); // wait for Leonardo enumeration, others continue immediately

    init_mpu1();
    init_mpu2();

    // configure LED for output
    pinMode(LED_PIN, OUTPUT);
}


//Hinge joint code here
//calculateAngle DONE
//vectorDot DONE
//vectorCross DONE
//vectorNormalize

// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

VectorInt16 vectorCross(Quaternion q, VectorInt16 vec)
{
  crossvec.x = q.y * vec.z - q.z * vec.y;
  crossvec.y = q.z * vec.x - q.x * vec.z;
  crossvec.z = q.x * vec.y - q.y * vec.x;
  return crossvec;
}
VectorInt16 vectorCross2(VectorInt16 vec1, VectorInt16 vec2)
{
  crossvec.x = vec1.y * vec2.z - vec1.z * vec2.y;
  crossvec.y = vec1.z * vec2.x - vec1.x * vec2.z;
  crossvec.z = vec1.x * vec2.y - vec1.y * vec2.x;
  return crossvec;
}
float vectorDot(Quaternion q, VectorInt16 vec)
{
  dotproduct = q.x * vec.x + q.y * vec.y + q.z * vec.z; 
  return dotproduct;
}
float vectorDot2(VectorInt16 vec1, VectorInt16 vec2)
{
  dotproduct = vec1.x * vec2.x + vec1.y * vec2.y + vec1.z * vec2.z; 
  return dotproduct;
}

VectorInt16 vectorQuatMult(Quaternion q, VectorInt16 vec)
{
        negvec.x = -1 * q.x;
        negvec.y = -1 * q.y; 
        negvec.z = -1 * q.z;
        neg_qw = q.w;

        cross1 = vectorCross(q, vec);
        w_new = neg_qw * vw - vectorDot(q, vec);
        newvec.x = vec.x * q.w + q.x * vw + cross1.x; 
        newvec.y = vec.y * q.w + q.y * vw + cross1.y; 
        newvec.z = vec.z * q.w + q.z * vw + cross1.z; 
        
        cross2 = vectorCross2(newvec,negvec);
        quatvec.x = negvec.x * w_new + newvec.x * neg_qw + cross1.x;
        quatvec.y = negvec.y * w_new + newvec.y * neg_qw + cross1.y;
        quatvec.z = negvec.z * w_new + newvec.z * neg_qw + cross1.z;
        
        return quatvec;
}

VectorInt16 calculateDeviceVector(Quaternion q, VectorInt16 vec)
{
  devicevec = vectorQuatMult(q, vec); 
  return devicevec;
}

float calculateAngle(VectorInt16 vec1, VectorInt16 vec2)
{
  dotproduct = max(min(vectorDot2(vec1,vec2), 1.0), -1.0);
  angle = acos(dotproduct);
  return angle;
}


void loop() {

  //This is where the important part of the loop starts
    // if programming failed, don't try to do anything
    if (!dmpReady) return;
    // read a packet from FIFO
    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet 
        mpu.dmpGetQuaternion(&queasy, fifoBuffer);
        mptwo.dmpGetQuaternion(&queasy2, fifoBuffer);
        
        forward0 = calculateDeviceVector(queasy, {0.0, 0.0, 1.0});
        forward1 = calculateDeviceVector(queasy2, {0.0, 0.0, 1.0});
        //up0 = calculateDeviceVector(queasy, {0.0, 1.0, 0.0});

        hinge_angle = calculateAngle(forward1, forward0); //add up0 if things aren't accurate
        Serial.print("Knee Angle: "); 
        Serial.print(hinge_angle);
        Serial.print("\n"); 
        
  //everything else below is useless 
  
        #ifdef OUTPUT_READABLE_QUATERNION
            // display quaternion values in easy matrix form: w x y z
            mpu.dmpGetQuaternion(&queasy, fifoBuffer);
            Serial.print("quat\t");
            Serial.print(queasy.w);
            Serial.print("\t");
            Serial.print(queasy.x);
            Serial.print("\t");
            Serial.print(queasy.y);
            Serial.print("\t");
            Serial.println(queasy.z);
        #endif

        
        #ifdef OUTPUT_READABLE_EULER
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&queasy, fifoBuffer);
            mpu.dmpGetEuler(euler, &queasy);
            Serial.print("euler\t");
            Serial.print(euler[1] * 180/M_PI);
            Serial.print("\t");
            Serial.println(euler[2] * 180/M_PI);
        #endif

        #ifdef OUTPUT_READABLE_YAWPITCHROLL
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&queasy, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &queasy);
            mpu.dmpGetYawPitchRoll(ypr, &queasy, &gravity);
            Serial.print("ypr\t");
            Serial.print(ypr[1] * 180/M_PI);
            Serial.print("\t");
            Serial.println(ypr[2] * 180/M_PI);
        #endif

        #ifdef OUTPUT_READABLE_REALACCEL
            // display real acceleration, adjusted to remove gravity
            mpu.dmpGetQuaternion(&queasy, fifoBuffer);
            mpu.dmpGetAccel(&aa, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &queasy);
            mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
            Serial.print("areal\t");
            Serial.print(aaReal.x);
            Serial.print("\t");
            Serial.print(aaReal.y);
            Serial.print("\t");
            Serial.println(aaReal.z);
        #endif

        #ifdef OUTPUT_READABLE_WORLDACCEL
            // display initial world-frame acceleration, adjusted to remove gravity
            // and rotated based on known orientation from quaternion
            mpu.dmpGetQuaternion(&queasy, fifoBuffer);
            mpu.dmpGetAccel(&aa, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &queasy);
            mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
            mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &queasy);
            Serial.print("aworld\t");
            Serial.print(aaWorld.x);
            Serial.print("\t");
            Serial.print(aaWorld.y);
            Serial.print("\t");
            Serial.println(aaWorld.z);
        #endif

        //sorry bro I'll make this look less ugly... 
        
        
        
        // blink LED to indicate activity
        blinkState = !blinkState;
        digitalWrite(LED_PIN, blinkState);
    }
}
