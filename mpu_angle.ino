//include libraries

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"
#include <ros.h>
#include <std_msgs/Float32.h>
ros::NodeHandle nh;
std_msgs::Float32 yaw_angel;
ros::Publisher Yaw("Yaw", &yaw_angel);

//hardware serial 
HardwareSerial Serial1(PA_7, PA_6);
HardwareSerial Serial3(PA_11, PA_10);

MPU6050 mpu;
const int MPU_addr=0x68;

// MPU control/status vars
Quaternion q;  
VectorFloat gravity;    // [x, y, z]            gravity vector
bool dmpReady = false;  // set true if DMP init was successful
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
//uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };


void setup() {
  //start i2c communication
  nh.initNode();
 nh.advertise(Yaw);
 /*Wire.begin();
Wire.beginTransmission(MPU_addr);
Wire.write(0x6B);
Wire.write(0);
Wire.endTransmission(true);*/
 Serial1.begin(115200);
 Serial3.begin(115200);
 (nh.getHardware())->setPort(&Serial3);
 (nh.getHardware())->setBaud(1511200);
    while (!Serial); 
    mpu.initialize();


    // verify connection
    Serial3.println(F("Testing device connections..."));
    Serial3.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    while (Serial3.available() && Serial3.read()); // empty buffer
    while (!Serial3.available());                 // wait for data
    while (Serial3.available() && Serial3.read()); // empty buffer again

    // load and configure the DMP
    Serial3.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    //set the offset
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

    //return 0 if the operations work successfully
    if (devStatus == 0) {
        // Calibration Time: generate offsets and calibrate our MPU6050
        mpu.CalibrateAccel(6);
        mpu.CalibrateGyro(6);
        mpu.PrintActiveOffsets();
        // turn on the DMP, now that it's ready
        Serial3.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        Serial3.print(F("DMP Initialization failed (code "));
        Serial3.print(devStatus);
        Serial3.println(F(")"));
    }

 
}


void loop() {
    // if programming failed, don't try to do anything
    if (!dmpReady) return;
    // read a packet from FIFO
    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet 
     

            // display  angles in degrees
           
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            //Serial.print("YAW \t");
            //Serial.print(ypr[0] * 180/M_PI);

 yaw_angel.data = ypr[0] * 180/M_PI;
 Yaw.publish( &yaw_angel);
 nh.spinOnce();
  
delay(400);

        
     
    }
}
