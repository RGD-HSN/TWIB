// Viral Science www.youtube.com/c/viralscience  www.viralsciencecreativity.com
// Self Balancing Robot
#include <PID_v1.h>
#include <LMotorController.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
 #include "Wire.h"
#endif

#define MIN_ABS_SPEED 35
  
MPU6050 mpu;

// MPU control/status vars
bool dmpReady = false; // set true if DMP init was successful
uint8_t mpuIntStatus; // holds actual interrupt status byte from MPU
uint8_t devStatus; // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize; // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount; // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q; // [w, x, y, z] quaternion container
VectorFloat gravity; // [x, y, z] gravity vector
float ypr[3]; // [yaw, pitch, roll] yaw/pitch/roll container and gravity vector

//PID
double originalSetpoint = 176.50;
double setpoint = originalSetpoint;
double movingAngleOffset = 0.1;
double input, output;
  int bitValue = 0;

// adjust these values to fit your own design
double Kp = 120;   
double Kd = 6.2;
double Ki = 470;
int n=0;
PID pid(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

double motorSpeedFactorLeft = 0.9;
double motorSpeedFactorRight = 0.9;

// MOTOR CONTROLLER
int ENA = 9;
int IN1 = 4;
int IN2 = 3;
int IN3 = 7;
int IN4 = 5;
int ENB = 10;
LMotorController motorController(ENA, IN1, IN2, ENB, IN3, IN4, motorSpeedFactorLeft, motorSpeedFactorRight);

volatile bool mpuInterrupt = false; // indicates whether MPU interrupt pin has gone high
void dmpDataReady()
{
    mpuInterrupt = true;
}

void setup()
{Serial.begin(115200);
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin();
    TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400, true);
    #endif

    mpu.initialize();

    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0)
    {
        // turn on the DMP, now that it's ready
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        attachInterrupt(0, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
        
        // setup PID
        pid.SetMode(AUTOMATIC);
        pid.SetSampleTime(10);
        pid.SetOutputLimits(-255, 255); 

        pinMode(0,INPUT);
        pinMode(1,INPUT);
        pinMode(6,INPUT);
        pinMode(8,INPUT);
        pinMode(11,INPUT);
        pinMode(12,INPUT);
     }
    else
    {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }
      pinMode(12, INPUT); // Channel A
  pinMode(13, INPUT); // Channel B

}

void loop()
{

    if (!dmpReady) return;

    while (!mpuInterrupt && fifoCount < packetSize)
    {
        pid.Compute();
 
        motorController.move(output, MIN_ABS_SPEED);
    }

    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024)
    {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        Serial.println(F("FIFO overflow!"));
    }
    
    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    else if (mpuIntStatus & 0x02)
    {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;

        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        input = ypr[1] * 180/M_PI + 180;
    }
if (n>20)
{
//Serial.println(input);

Kp= analogRead(A0);
Ki= (512-analogRead(A1));
Kd= (512-analogRead(A2))/30;
pid.SetTunings(Kp, Ki, Kd);

n=0;

}
else
{
  n=n+1;
}
int stateA = digitalRead(8);
  int stateB = digitalRead(11);
  int stateC = digitalRead(12);
 
  int bitValue = 0;
  bitValue |= (stateA << 0); // Read pinA and shift by 0 bits
  bitValue |= (stateB << 1); // Read pinB and shift by 1 bit
  bitValue |= (stateC << 2); 
 // Check if pin 6 is HIGH
if (digitalRead(6)) {
  // Update setpoint based on the 3-bit value
  switch (bitValue) {
    case 0:
      setpoint = originalSetpoint + 0;
      break;
    case 1:
      setpoint = originalSetpoint + 0.25;
      break;
    case 2:
      setpoint = originalSetpoint + 0.05;
      break;
    case 3:
      setpoint = originalSetpoint + 0.075;
      break;
    case 4:
      setpoint = originalSetpoint + 0.1;
      break;
    case 5:
      setpoint = originalSetpoint + 0.125;
      break;
    case 6:
      setpoint = originalSetpoint + 0.15;
      break;
    case 7:
      setpoint = originalSetpoint + 0.2;
      break;
    default:
      // Handle unexpected values (shouldn't occur with valid 3-bit values)
      break;
  }
}
else
{
  switch (bitValue) {
    case 0:
      setpoint = originalSetpoint + 0;
      break;
    case 1:
      setpoint = originalSetpoint - 0.025;
      break;
    case 2:
      setpoint = originalSetpoint - 0.05;
      break;
    case 3:
      setpoint = originalSetpoint - 0.075;
      break;
    case 4:
      setpoint = originalSetpoint - 0.1;
      break;
    case 5:
      setpoint = originalSetpoint - 0.125;
      break;
    case 6:
      setpoint = originalSetpoint - 0.15;
      break;
    case 7:
      setpoint = originalSetpoint - 0.2;
      break;
    default:
      // Handle unexpected values (shouldn't occur with valid 3-bit values)
      break;

}
}
Serial.println(setpoint);
Serial.println(bitValue);
Serial.println(input);
Serial.println(Kp);
Serial.println(Ki);
Serial.println(Kd);


}
