/*

  balancing robot modified internet code (oh so lazy)
    control from i2c mpu6050 via arduino i2c ()
    PWM output for motors on arduino 3 and 11, which is timer 2A/B
      using "tompwm" where side A is inverted from side B, 50% is "stop"

  "I promise to weed out the garbage and fit it all later"
  
  Rue.

*/


#include "I2Cdev.h"
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
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

/*********Tune these 4 values for your BOT*********/
double setpoint= 169.3; // 169.5; //set the value when the bot is perpendicular to ground using serial monitor. 

//Read the project documentation on circuitdigest.com to learn how to set these values
double Kp = 32; //40; //15; // Set this first
double Kd = 0.6; //0.8; Set this secound
double Ki = 0; //120; // Finally set this 
/******End of values setting*********/

float av;

double input, output;

PID pid(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high

void dmpDataReady() {
    mpuInterrupt = true;
}



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
        // enable Arduino interrupt detection
        Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        attachInterrupt(0, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();
        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;
        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
        //setup PID
        pid.SetMode(AUTOMATIC);
        pid.SetSampleTime(8);
        pid.SetOutputLimits(-128, 127);
  
    }    else    {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }        

}


void loop() {
    // if programming failed, don't try to do anything
    if (!dmpReady) return;
    
    // wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt && (fifoCount < packetSize))    {
   // while ((fifoCount < packetSize))    { // damnit why is that interrupt needed? GIVE ME THE DATA.
    
        //no mpu data - performing PID calculations and output to motors     
        pid.Compute();
  
      // DO NOT SLOW DOWN THE LOOP WITH STUPID SLOW SERIAL DATA!!!
        //Print the value of Input and Output on serial monitor to check how it is working.
      //  Serial.print(input); Serial.print(", "); Serial.println(output);
       // av = (av+input)/2.0; // never average a signal like this :]
       // Serial.println(input);
        
      //  if ((input > 150) && (input < 200)){//If the Bot is falling // WTF? - Rue.
      
      if ((input<(setpoint+30)) &&( input > (setpoint-30))){ // if you fell over +- 30 degrees GIVE UP!!!!!
          SetSpeed(); //Rotate the wheels backward 
      } else {
          Stop(); // you fell over.
      }
    }
    
    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();
    
    // get current FIFO count
    fifoCount = mpu.getFIFOCount();
    
    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024)    {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        Serial.println(F("FIFO overflow!"));
  
    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    }  else if (mpuIntStatus & 0x02)   {
    
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
  
        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
  
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
  
        fifoCount -= packetSize;
        mpu.dmpGetQuaternion(&q, fifoBuffer); //get value for q
        mpu.dmpGetGravity(&gravity, &q); //get value for gravity
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity); //get value for ypr
  
        input = ypr[1] * 180/M_PI + 180;
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





