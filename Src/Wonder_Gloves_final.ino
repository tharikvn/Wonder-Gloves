#include <SPI.h>
#include <Wire.h>
#include <SoftwareSerial.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

MPU6050 mpu;

#define OUTPUT_READABLE_YAWPITCHROLL

#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards
#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;

bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

int pitch,roll;

// constants
const int sensorPinPINKEY = A4;
const int sensorPinTHUMB = A0;
const int sensorPinMIDDLE = A2;
const int sensorPinINDEX = A1;
const int sensorPinRING = A3;

// variables:
int sensorValueTHUMB = 0;
int sensorValueINDEX = 0;
int sensorValueMIDDLE = 0;
int sensorValueRING = 0;
int sensorValuePINKEY = 0;

int sensorMinTHUMB = 1023;
int sensorMinINDEX = 1023;
int sensorMinMIDDLE = 1023;
int sensorMinRING = 1023;
int sensorMinPINKEY = 1023;

int sensorMaxTHUMB = 0;           
int sensorMaxINDEX = 0;          
int sensorMaxMIDDLE = 0;          
int sensorMaxRING = 0;
int sensorMaxPINKEY = 0;

char str[100],ch;
int i=0;
int j;

SoftwareSerial BTSerial(11, 12); //RX|TX



void setup(){

  Serial.begin(115200);
  BTSerial.begin(9600); // default baud rate

    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    while (!Serial); // wait for Leonardo enumeration, others continue immediately

 //   Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);

 //   Serial.println(F("Testing device connections..."));
 //   Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

 //   Serial.println(F("\nSend any character to begin DMP programming and demo: "));
    while (Serial.available() && Serial.read()); // empty buffer
    while (!Serial.available());                 // wait for data
    while (Serial.available() && Serial.read()); // empty buffer again

 //   Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    mpu.setXGyroOffset(-301);
    mpu.setYGyroOffset(-95);
    mpu.setZGyroOffset(-58);
    mpu.setZAccelOffset(1735); // 1688 factory default for my test chip
    mpu.setYAccelOffset(-2313);
    mpu.setXAccelOffset(2334);

    if (devStatus == 0) {
 //       Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

 //       Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

 //       Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
 //       Serial.print(F("DMP Initialization failed (code "));
 //       Serial.print(devStatus);
 //       Serial.println(F(")"));
    }
    pinMode(LED_PIN, OUTPUT);


  }
  
  
void write(){
     ///for(j=0;j<i;j++){
     ///Serial.print(str[j]);
     BTSerial.write(str);   
     //}
     
}



void loop(){
  

//Begin infinite loop by reading each sensor value
  sensorValueTHUMB = analogRead(sensorPinTHUMB);
  sensorValueINDEX = analogRead(sensorPinINDEX);
  sensorValueMIDDLE = analogRead(sensorPinMIDDLE);
  sensorValueRING = analogRead(sensorPinRING);
  sensorValuePINKEY = analogRead(sensorPinPINKEY);

  // Map the range of values from each sensor into 255 discrete levels
  sensorValueTHUMB = map(sensorValueTHUMB, sensorMinTHUMB, sensorMaxTHUMB, 1, 255);
  sensorValueINDEX = map(sensorValueINDEX, sensorMinINDEX, sensorMaxINDEX, 1, 255);
  sensorValueMIDDLE = map(sensorValueMIDDLE, sensorMinMIDDLE, sensorMaxMIDDLE, 1, 255);
  sensorValueRING = map(sensorValueRING, sensorMinRING, sensorMaxRING, 1, 255);
  sensorValuePINKEY = map(sensorValuePINKEY, sensorMinPINKEY, sensorMaxPINKEY, 1, 255);
  

  // Set a floor and ceiling of 1 and 255 to avoid out-of-bounds values
  sensorValueTHUMB = constrain(sensorValueTHUMB, 1, 255);
  sensorValueINDEX = constrain(sensorValueINDEX, 1, 255);
  sensorValueMIDDLE = constrain(sensorValueMIDDLE, 1, 255);
  sensorValueRING = constrain(sensorValueRING, 1, 255);
  sensorValuePINKEY = constrain(sensorValuePINKEY, 1, 255);

    if (!dmpReady) return;
    while (!mpuInterrupt && fifoCount < packetSize) {
    }

    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    fifoCount = mpu.getFIFOCount();

    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        mpu.resetFIFO();
 //       Serial.println(F("FIFO overflow!"));
    } else if (mpuIntStatus & 0x02) {
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        fifoCount -= packetSize;

        #ifdef OUTPUT_READABLE_YAWPITCHROLL
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            Serial.print("ypr\t");
           Serial.print(ypr[0] * 180/M_PI);
           Serial.print("\t");
          Serial.print(ypr[1] * 180/M_PI);
          Serial.print("\t");
        Serial.println(ypr[2] * 180/M_PI);
        #endif

        pitch=(ypr[1] * 180/M_PI);
        roll=(ypr[2] * 180/M_PI);
        
        Serial.print(pitch);
      //  Serial.print(roll);
        
        blinkState = !blinkState;
        digitalWrite(LED_PIN, blinkState);
    }

  
  // Diagnostic code that displays current values for all sensors when needed
  
    
    Serial.println ("-------------------");
    Serial.print ("Thumb Value = ");
    Serial.println (sensorValueTHUMB);

    Serial.print ("Index Finger Value = ");
    Serial.println (sensorValueINDEX);

    Serial.print ("Middle Finger Value = ");
    Serial.println (sensorValueMIDDLE);

    Serial.print ("Ring Finger Value = ");
    Serial.println (sensorValueRING);  

    Serial.print ("PINKEY Value = ");
    Serial.println (sensorValuePINKEY);

    
    
    
    
    

  // Begin conditionals that check for specific gestures
  // Check for 'mute'
if (sensorValueTHUMB < 190 && sensorValueINDEX < 190 && sensorValueMIDDLE > 190 && sensorValueRING < 190 && sensorValuePINKEY < 190) {
  
}
else{

  // Check for 'A'
  if (sensorValueTHUMB < 190 && sensorValueINDEX > 190 && sensorValueMIDDLE > 190 && sensorValueRING > 190 && sensorValuePINKEY > 190) {
      str[i]='A';
      Serial.print ("A"); 
      
      
    }
  
  


  // Check for 'B'
  else if (sensorValueTHUMB > 190 && sensorValueINDEX < 190 && sensorValueMIDDLE < 190 && sensorValueMIDDLE < 190 && sensorValuePINKEY < 190) {
    str[i]='B';
    Serial.print ("B"); 
   
    }


  // Check for 'C'
  else if (sensorValueTHUMB < 190 && sensorValueINDEX < 190 && sensorValueMIDDLE < 190 && sensorValueRING < 190 && sensorValuePINKEY < 190) {
    str[i]='C';
    Serial.print ("C"); 
    
    }
 
  

  // Check for 'D'
  else if (sensorValueTHUMB > 190 && sensorValueINDEX < 190 && sensorValueMIDDLE > 190 && sensorValueRING > 190 && sensorValuePINKEY > 190) {
    str[i]='D';
    Serial.print ("D"); 

      }
  


  // Check for 'E'
  else if (sensorValueTHUMB > 190 && sensorValueINDEX > 190 && sensorValueMIDDLE > 190 && sensorValueRING > 190 && sensorValuePINKEY > 190){
    str[i]='E';
    Serial.print  ("E");

  }


  // Check for 'F'
  else if (sensorValueTHUMB > 190 && sensorValueINDEX > 190 && sensorValueMIDDLE < 190 && sensorValueRING < 190 && sensorValuePINKEY < 190) {
    str[i]='F';
    Serial.print ("F"); 

      }
  

  // Check for 'G'
  else if (pitch > -10 && pitch < 40 && sensorValueTHUMB < 190 && sensorValueINDEX < 190 && sensorValueMIDDLE > 190 && sensorValueRING > 190 && sensorValuePINKEY > 190) {
    str[i]='G';
    Serial.print ("G"); 

      }


  // Check for 'H'
  else if (pitch > -10 && pitch < 40 && sensorValueTHUMB > 190 && sensorValueINDEX < 190 && sensorValueMIDDLE < 190 && sensorValueRING > 190 && sensorValuePINKEY > 190) {
    str[i]='H';
    Serial.print ("H"); 

      }

  // Check for 'I'
  else if (sensorValueTHUMB > 190 && sensorValueINDEX > 190 && sensorValueMIDDLE > 190 && sensorValueRING > 190 && sensorValuePINKEY < 190) {
    str[i]='I';
    Serial.print ("I"); 

      }

  // Check for 'J'
  else if (pitch > -10 && pitch < 40 && sensorValueTHUMB > 190 && sensorValueINDEX > 190 && sensorValueMIDDLE > 190 && sensorValueRING > 190 && sensorValuePINKEY < 190) {
    str[i]='J';
    Serial.print ("J"); 

  }

   // Check for 'K'
  else if (pitch > -10 && pitch < 40 && sensorValueTHUMB < 190 && sensorValueINDEX < 190 && sensorValueMIDDLE < 190 && sensorValueRING > 190 && sensorValuePINKEY > 190) {
    str[i]='K';
    Serial.print ("K"); 

  }
  

  // Check for 'L'
  else if (sensorValueTHUMB < 190 && sensorValueINDEX < 190 && sensorValueMIDDLE > 190 && sensorValueRING > 190 && sensorValuePINKEY > 190) {
    str[i]='L';
    Serial.print ("L"); 
  }
  

  // Check for 'M'
  else if (pitch > -100 && pitch < -30 && sensorValueTHUMB < 190 && sensorValueINDEX < 190 && sensorValueMIDDLE < 190 && sensorValueRING < 190 && sensorValuePINKEY > 190) {
    str[i]='M';
    Serial.print ("M"); 

  }
 
  // Check for 'N'
  else if (pitch > -100 && pitch < -30 && sensorValueTHUMB > 190 && sensorValueINDEX < 190 && sensorValueMIDDLE < 190 && sensorValueRING > 190 && sensorValuePINKEY > 190) {
    str[i]='N';
    Serial.print ("N"); 

  }
  

  // Check for 'O'
  else if (pitch > -10 && pitch < 40 && sensorValueTHUMB > 190 && sensorValueINDEX > 190 && sensorValueMIDDLE > 190 && sensorValueRING > 190 && sensorValuePINKEY > 190) {
    str[i]='O';
    Serial.print ("O"); 

  }
  

// Check for 'P'
  else if (pitch > -10 && pitch < 40 && sensorValueTHUMB > 190 && sensorValueINDEX < 190 && sensorValueMIDDLE < 190 && sensorValueRING > 190 && sensorValuePINKEY > 190) {
    str[i]='P';
    Serial.print ("P"); 

  }
  
  
  // Check for 'Q'
  else if (pitch > -100 && pitch < -30 && sensorValueTHUMB > 190 && sensorValueINDEX > 190 && sensorValueMIDDLE < 190 && sensorValueRING < 190 && sensorValuePINKEY < 190) {
    str[i]='Q';
    Serial.print ("Q"); 

  }
 
  // Check for 'R'
  else if (sensorValueTHUMB > 190 && sensorValueINDEX < 190 && sensorValueMIDDLE < 190 && sensorValueRING > 190 && sensorValuePINKEY > 190) {
    str[i]='R';
    Serial.print ("R"); 

  }
 

  
  // Check for 'S'
  else if (pitch > -10 && pitch < 40 && sensorValueTHUMB < 190 && sensorValueINDEX > 190 && sensorValueMIDDLE > 190 && sensorValueRING > 190 && sensorValuePINKEY > 190) {
    str[i]='S';
    Serial.print ("S"); 

  }

// Check for 'T'
  else if (pitch > -100 && pitch < -30 && sensorValueTHUMB > 190 && sensorValueINDEX > 190 && sensorValueMIDDLE > 190 && sensorValueRING > 190 && sensorValuePINKEY > 190) {
    str[i]='T';
    Serial.print ("T"); 

  }
  

  // Check for 'U'
  else if (sensorValueTHUMB < 190 && sensorValueINDEX < 190 && sensorValueMIDDLE < 190 && sensorValueRING > 190 && sensorValuePINKEY < 190) {
    str[i]='U';
    Serial.print ("U"); 

  }
  

  // Check for 'V'
  else if (pitch > -10 && pitch < 40 && sensorValueTHUMB < 190 && sensorValueINDEX < 190 && sensorValueMIDDLE < 190 && sensorValueRING > 190 && sensorValuePINKEY < 190) {
    str[i]='V';
    Serial.print ("V"); 

  }
  

  // Check for 'W'
  else if (sensorValueTHUMB < 190 && sensorValueINDEX < 190 && sensorValueMIDDLE < 190 && sensorValueRING < 190 && sensorValuePINKEY > 190) {
    str[i]='W';
    Serial.print ("W"); 

  }
  

  // Check for 'X'
  else if (pitch > -100 && pitch < -30 && sensorValueTHUMB < 190 && sensorValueINDEX > 190 && sensorValueMIDDLE > 190 && sensorValueRING > 190 && sensorValuePINKEY > 190) {
    str[i]='X';
    Serial.print ("X"); 

  }
  
  // Check for 'Y'
  else if (sensorValueTHUMB < 190 && sensorValueINDEX > 190 && sensorValueMIDDLE > 190 && sensorValueRING > 190 && sensorValuePINKEY < 190) {
    str[i]='Y';
    Serial.print ("Y"); 

  }

  // Check for 'Z'
  else if (pitch > -10 && pitch < 40 && sensorValueTHUMB > 190 && sensorValueINDEX < 190 && sensorValueMIDDLE > 190 && sensorValueRING > 190 && sensorValuePINKEY > 190) {
    str[i]='Z';
    Serial.print ("Z"); 
  }
 /*
  // Check for 'mute'
  else if (sensorValueTHUMB < 190 && sensorValueINDEX < 190 && sensorValueMIDDLE > 190 && sensorValueRING > 190 && sensorValuePINKEY < 190) {
    str[i]='Z';
    Serial.print ("Z"); 
  }
*/  
  // Check for '*'
  else if(sensorValueTHUMB < 190 && sensorValueINDEX < 190 && sensorValueMIDDLE > 190 && sensorValueRING > 190 && sensorValuePINKEY < 190) {
    //Serial.print ("*");
    str[i]='\0'; 
    write();
    i=-1;
  }
 
  /// End Infinite loop. Wait one sec. then restart
  delay (1000);    
  i++;
 }
 delay (1000);
}

