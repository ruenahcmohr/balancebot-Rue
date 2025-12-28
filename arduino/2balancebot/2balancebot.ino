/*
 * I'm running this on an atmega328P at 16MHz
 *  The sample rate is about 100Hz 
 *  The i2c clock rate is 1MHz
 *  PWM is 62.5kHz (check your motor drivers are ok with this)
 *  
 */

#include <Wire.h>
#include <PID_v1.h> //From https://github.com/br3ttb/Arduino-PID-Library/blob/master/PID_v1.h
#include "MPU6050_6Axis_MotionApps20.h" //https://github.com/jrowberg/i2cdevlib/tree/master/Arduino/MPU6050


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

/*********Tune these 4 values for your BOT*********/
double setpoint = 13; //17;  //set the value when the bot is perpendicular to ground using serial monitor. 

//Read the project documentation on circuitdigest.com to learn how to set these values
double Kp = 16; //16; //40; //15; // Set this first
double Kd = 0.3; //0.6; Set this secound
double Ki = 64; //120; // Finally set this 
/******End of values setting*********/

float av;

double input, output;

PID pid(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);


// arduino pins 3 and 11

void pwm2Init() {

//Initialise the Motor outpu pins
    pinMode (3, OUTPUT);
    pinMode (11, OUTPUT);
  
  // clear pwm levels
  OCR2A = 128;
  OCR2B = 128;  
  
  // set up WGM, clock, and mode for timer 0
  TCCR2A = 1 << COM2A1 | /* ** normal polarity */
           0 << COM2A0 | /*   this bit 1 for interted, 0 for normal  */
           1 << COM2B1 | /* ** normal polarity */
           0 << COM2B0 | /*   this bit 1 for interted, 0 for normal  */
           1 << WGM20 | /* fast pwm */
           1 << WGM21  ;
          
  TCCR2B = 1 << CS20  ;  /* CLKio /1 */


 }

void setup() {

  pwm2Init(); // init motor controls.
   
  Serial.begin(115200);
  // initialize device
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();
     // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
    // load and configure the DMP
    devStatus = mpu.dmpInitialize();
    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(  33);  // girox
    mpu.setYGyroOffset(  -1);  // giroy
    mpu.setZGyroOffset(  15);  // giroz
    mpu.setZAccelOffset( 885); // acelz 
    
     // make sure it worked (returns 0 if so)
    if (devStatus == 0)    {
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);
        dmpReady = true;
        // get DMP packet size 
        packetSize = mpu.dmpGetFIFOPacketSize();
        //setup PID
        pid.SetMode(AUTOMATIC);
        pid.SetSampleTime(8); // data from the imu only comes in at 98Hz anyhow.
        // and this is still done all wrong, the samples should be processed immediatly.
        // but they aren't synchronized (yet)
        pid.SetOutputLimits(-128, 127); // this is the 8 bit range of my motor drivers 127/128 is stop.
  
    }    else    {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
        Serial.println(F("Is the i2c scope probe set to 10x?"));
    }        

}


void loop() {
    // if programming failed, don't try to do anything
    if (!dmpReady) return;
    
    // wait for MPU interrupt or extra packet(s) available
    while (!(mpuIntStatus = mpu.getIntStatus()))    {  // this is a lot more i2c traffic but *I dont think* it matters    
      //no mpu data - call PID update, at some point it will.
      // (PID set for 8ms, our updates are 10.2ms)
      pid.Compute() ;     
    }
    
    // check for overflow (this should never happen unless our code is too inefficient)
    if (mpuIntStatus & (1<<MPU6050_INTERRUPT_FIFO_OFLOW_BIT))    {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        Serial.println(F("FIFO overflow!"));
  
    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    }  else if (mpuIntStatus & (1<<MPU6050_INTERRUPT_DMP_INT_BIT))   {      
       
       // wait for correct available data length, should be a VERY short wait
       while ((fifoCount = mpu.getFIFOCount()) < packetSize);

       // read a packet from FIFO
       mpu.getFIFOBytes(fifoBuffer, packetSize);
       mpu.resetFIFO(); // we should not be losing samples, so make sure its reset after each transaction.

       mpu.dmpGetQuaternion(&q, fifoBuffer); //get value for q             
          
       input = asin(-2.0f * (q.x*q.z - q.w*q.y)) * RAD_TO_DEG; // less maths is better!

       if ((input<(setpoint+20)) &&( input > (setpoint-20))){ // give up if you fell over!
         SetSpeed(); // update motor speeds
       } else {
         Stop(); // you dun fell over.
       }   
       
   }
   
}


void SetSpeed() {  //Code to rotate the wheel forward 
    analogWrite(3,  output+128);
    analogWrite(11, output+128);
  //  Serial.print("F "); //Debugging information 
}


void Stop() {  //Code to rotate the wheel forward 
    analogWrite(3,128);
    analogWrite(11,128);
   // Serial.print("S "); //Debugging information 
}
