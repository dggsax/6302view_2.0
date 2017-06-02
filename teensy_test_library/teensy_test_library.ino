#include <mediator.h>

//Likely User Modified Variables ******************************
unsigned int deltaT = 1000;         // Sample period in microseconds.
int angleAverages = 5;
int past_size = 3;                 // interval for delta, larger=less noise, more delay.
unsigned long transferDt = 20000; // usecs between host updates  
//float Kp, Kd, Kbemf, Ku, Ki, directV, desiredV;

// ***************************************************************

// Teensy definitions
#define angleSensorPin  A9
//#define waveGenerator  A9
#define pwmVoltagePin A0
#define motorVoltagePin  A1
#define motorOutPWM  3
#define monitorPin 2  
#define dacRes 12   // Teensy 12-bit dac resolution, max val 4095
#define dacMax 4095
#define adcRes 14
#define adcCenter 8192 //Teensy ADC set to 14 bits, 16384/2 = 8192
#define adcMax 16383
#define led 13
// Circuit Specific Definitions.
#define scaleVadc 5.0 // All signals 0->5 volts.
#define vDrive 5.0 // Driver peak voltage.
#define rShunt 0.5 // Resistor from motor to ground.
#define rMotor 0.5 // Motor series resistance.

// Variables for Conversion
#define deg2rad 0.0175    
#define twopi 6.2831853  

// Build library instance
MEDIATOR test = MEDIATOR();

//Rest of Setup:
String config_message_30_bytes = "&A~DesiredAngV~5&C&S~Direct~O~0~5.0~0.1&S~DesiredAngV~A~-1~1~0.1&T~AngleV~F4~0~2.5&T~BackEMF~F4~0~5&T~MCmd~F4~0~5&H~4&";
String config_message = config_message_30_bytes;

float rad2deg = 1.0/deg2rad;        // 180/pi

float dTsec = 1.0e-6*deltaT;       // The period in seconds. 
float scaleD = 1.0/(dTsec*past_size); // Divide deltas by interval.    

float errorVintegral;                // Variable for integrating angle errors.
float integralMax = 200;            // Maximum value of the integral

// Storage for past values.
float pastErrorV[20];  // Should be larger array than past_size.

// Variables for loop control
//
elapsedMicros loopTime; // Create elapsed time variable to ensure regular loops.
unsigned int headroom;  // Headroom is time left before loop takes too long.
boolean switchFlag;

// Initializes past values.
void setup() {
  // Set up for the mediator
  test.setup(deltaT,transferDt);
  
  // Set up inputs
  analogReadResolution(adcRes);
  pinMode(motorVoltagePin, INPUT);
  pinMode(pwmVoltagePin, INPUT);
  pinMode(angleSensorPin, INPUT);

  // Set up output
  analogWriteResolution(dacRes);
  pinMode(motorOutPWM, OUTPUT);
  pinMode(led, OUTPUT);
  analogWriteFrequency(motorOutPWM, 23437.5); // Teensy 3.0 pin 3 also changes to 23437.5 kHz
  analogWrite(motorOutPWM, LOW);
  
  // Frequency Monitor
  pinMode(monitorPin, OUTPUT);
}

void loop() {  // Main code, runs repeatedly
  
  // Reinitializes or updates from sliders on GUI.
  test.restart(config_message);
  
//  startup();

  // Make sure loop starts deltaT microsecs since last start
  unsigned int newHeadroom = max(int(deltaT) - int(loopTime), 0);
  headroom = min(headroom, newHeadroom);
  
  while (int(loopTime) < int(deltaT)) {};
  loopTime = 0;
  
  // Monitor Output should be a square wave with frequency = 1/(2*deltaT) 
  switchFlag = !switchFlag;
  digitalWrite(monitorPin, switchFlag); 

  /*************** Section of Likely User modifications.**********************/
  // Read Angle, average to reduce noise.
  int angleI = 0;
  for(int i = 0; i < angleAverages; i++) angleI += analogRead(angleSensorPin);
  float angleV = scaleVadc*float(angleI- angleAverages*adcCenter)/float(angleAverages*adcMax)+2;
  
  // Read motor voltage and pmw voltage
  float motorV = scaleVadc*float(analogRead(motorVoltagePin))/float(adcMax);
  float pwmV = scaleVadc*float(analogRead(pwmVoltagePin))/float(adcMax);
  float motor_current = (pwmV - motorV)/rShunt;
  float motor_bemf = motorV - rMotor*motor_current;  // Ignores L(di/dt).

  // Compute and write out motor command.
  float motorCmd = directV;
  float motorCmdLim = min(max(motorCmd, 0), vDrive);
  analogWrite(motorOutPWM,int((motorCmdLim/vDrive)*dacMax));

  test.pack(angleV, motor_bemf, motorCmdLim, float(headroom));
}


//// Zero past errors
//for (int i = past_size-1; i >= 0; i--) pastErrorV[i] = 0;
