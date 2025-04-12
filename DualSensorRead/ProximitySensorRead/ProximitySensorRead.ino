/*
Project: Voltair.ai Firmware/Embedded Engineer Homework (Option B)  Data Collection and transmition
Author: Ahmed Murtata Qureshi
Git: https://github.com/ForTheLoop/VoltAirHomework.git 
Demo: https://drive.google.com/drive/folders/1joZnx5whSquUPZxLYTmnA-b2azhUj22q?usp=drive_link

Summary:- This project is to demonstrate data collection from 2 types of sensors(Analog and Digital), 
analysing that data to extract useful information and then transmitting it.  The system is the sensor controller 
imaginary vehicle with sensors mounted on its left and right side. The system samples data from both sensors and 
determines what the current "proximity state" of the vehicle is.Is it too close to the left or right or is is in a 
tight spot on both side. After determining its state it will transmit a data packet on to its serial port assuming a 
transmition device which is attached to the harware serial of the controller or for that matter to another MCU.
The project uses arduino UNO R3 as main controller, A Sharp IR range sensor and a HC-SR04 Sonar.

Highlights: Mostly Non-blocking. Samples data from both analog and digital sensor, capability to generate transmission 
data package and user can set data package type: Humanreadable string or byte array.

Code Flow:
1. Initialize Serial and I/O s
2. Trigger and Get Sonar Data in a non blocking manner and covert to meaningfull data i.e. distance;
3. Get Analog data from IR sensor and covert to meaningfull data i.e. distance;
4. Based on data deterime proximity state
5. Transmit proximity state data along with data values.

Usage:- Using a preprocessor directive "HUMANREADABLESTRING"(line 32) to set what type of transmission string you want send string based or byte based.
Note: Change Baudrate to match your serial mointor/device Baud Rate (line 91)

*/


#include <Arduino.h>  // Includes basic Arduino functions

#define HUMANREADABLESTRING 1 // preprocessor to select tranmission packet type: string or byte

const int referenceVoltage = 5;// 5V ref for analog volatage calculation
const int sensorPin = A0;   // Analog pin 1 for IR sensor Input

const int sonarTriggerPin = 4;// used the generate sonar signal for a min 10us
const int sonarEchoPin = 2;// this is to listen the signal and is attached to pin that has hardware intterupt on the UNO R3.


const float proximityLimits = 10.0; // distance in cm below which vechile is too close to the wall
unsigned long lastTriggerTime = 0;

/*
TIME BETWEEN TRIGGERS
Max round trip time Time (seconds)=  2×distance/speed of sound
​max 4 meters so round trip time is 23.33ms 
with a saftey factor of 4 we should atleast give a data processing time of 93.32ms ~ 100ms

*/
const unsigned long triggerInterval = 100; // time between triggers give time to recive signal before tirggering again.

const unsigned long samplingFreq = 1000;//frequecy at which you transmit data to transmission device.
unsigned long lastSampleTime = 0;// to determine time elapsed for data tranmission

float sonarDistance=0.0;// Distance from sonar which is the sensor on the RIGHT side of the car
float IRDistance=0.0;// Distance from IR sensor which is the sensor on the left side of the car

enum SonarStates // Sonar states
{
  Idle,// sonar is not doing anything
  Triggering,// Generating trigger pulse
  WaitingForEchoStart,// Waiting for eacho pulse
  EchoStarted,//Recived rising edge of echo pulse
  EchoEnded,//echo pulse has dieded out
};

enum ProximityStates // vehicle proximity states
{
  IdleProximity,
  TooCloseToLeft,
  TooCloseToRight,
  BothTooClose
};

volatile SonarStates currentSonarState = Idle;// current proximity state

volatile unsigned long echoStartTime = 0;//the time when echo pulse was recieved Rising edge detected
volatile unsigned long echoEndTime = 0;// the time when echo pulse died out Falling edge detected
volatile bool distanceReady = false; // sonar distance has been calculated after generating pulse and listening to the echo

// Rising Edge of Echo ISR
void echoRisingISR() 
{
  echoStartTime = micros();// store time in microseconds
  detachInterrupt(digitalPinToInterrupt(sonarEchoPin));//detach current interrupt
  attachInterrupt(digitalPinToInterrupt(sonarEchoPin), echoFallingISR, FALLING);//attachintterup to pin, set ISR and tell it what feature to look for
  currentSonarState = EchoStarted;// Change Sonar state
}

//Falling edge of Echo ISR
void echoFallingISR() 
{
  echoEndTime = micros();// store time when echo pulse died in microseconds
  detachInterrupt(digitalPinToInterrupt(sonarEchoPin));//detach current interrupt to prevent it triggering again
  distanceReady = true;// "we have all the data to calulate distance now" flag
  currentSonarState = EchoEnded;// change sonar state
}


void setup() 
{
  Serial.begin(9600); // Start serial communication
  //set I/0 for sonar
  pinMode(sonarTriggerPin,OUTPUT);
  pinMode(sonarEchoPin,INPUT);
}

void loop() 
{

  SonarSensorStateMachine();// Sonar manager function

  IRDistance = GetIRRange();// IR Range sensor manager function

  ProximityStates currentProximityState = DetermineProximityState();// Determines proximity to left and right

  OutputMessagePacket(currentProximityState);// transmit usefull data 

}

// Function gets Range from IR sensor
float GetIRRange()
{
  int sensorValue = analogRead(sensorPin); // Read the analog pin (0–1023) 2^16;
  float voltage = (sensorValue / 1023.0) * referenceVoltage;// calulate actual voltage from discreate signal
  return (27.86 * pow(voltage, -1.15));     // Calculate distance in cm this is based on a reponse curve from the data sheet
}

//Function that mangaes the sonar sensor opertaion and determines distance;
void SonarSensorStateMachine()
{

  unsigned long currentMillis = millis();

  // Start a new measurement
  if (currentSonarState == Idle && (currentMillis - lastTriggerTime >= triggerInterval))// check is idle and that its not trigger faster than the determined trigger interval
  {
    currentSonarState = Triggering;// Change state
    digitalWrite(sonarTriggerPin, LOW);// clear signal
    delayMicroseconds(2);// wait 2 microseconds
    digitalWrite(sonarTriggerPin, HIGH);// generate trigger pulse
    delayMicroseconds(10); // 10us Pulse
    digitalWrite(sonarTriggerPin, LOW);//stop generating pulse

    attachInterrupt(digitalPinToInterrupt(sonarEchoPin), echoRisingISR, RISING);//attachintterup to pin, set ISR and tell it what feature to look for
    currentSonarState = WaitingForEchoStart;//change state to listen for echo now
    lastTriggerTime = currentMillis;
  }

  // If distance measurement is ready
  if (currentSonarState == EchoEnded && distanceReady) // echo pulse has died out and we have the duration for the echo pulse
  {
    sonarDistance = (echoEndTime - echoStartTime) * 0.034 / 2.0;//Duration times the speed of sound divided by 2 (round trip time)
    distanceReady = false;
    currentSonarState = Idle;// the sonar is now avilable to generate a trigger pulse
  }
}
//Function that determines the proximity state of the vehicle from the walls
ProximityStates DetermineProximityState()
{
  ProximityStates currentProximityState = IdleProximity;
  if(IRDistance >= proximityLimits && sonarDistance >= proximityLimits)// Right and left sensor both are in safe distance from the walls
  {
      currentProximityState = IdleProximity;
  }
  else if(IRDistance < proximityLimits && sonarDistance < proximityLimits)// both righ and left walls are too close the vehicle (tight space)
  {
      currentProximityState = BothTooClose;
  }
  else if(IRDistance < proximityLimits)// Left sensor is too close to the wall
  {
      currentProximityState = TooCloseToLeft;
  }
  else if(sonarDistance < proximityLimits)// right sensor is too close to the wall
  {
      currentProximityState = TooCloseToRight;
  }
  else
  {
      //within Dead band
  }
  return currentProximityState;
}

//Function transmits data packet to external device
void OutputMessagePacket(ProximityStates state )
{
  unsigned long currentMillis = millis();
  if((currentMillis- lastSampleTime) >= samplingFreq)// make sure that data is sent at user defined sampling rate
  {
    if(HUMANREADABLESTRING == 1) // if its a human readable string the user wants
    {
      int checkSum = state ^ (int)IRDistance ^ (int)sonarDistance; // XOR Checksum to make sure data is not currupted at reciving end
      //string identifier $, Proximity state, Left sensor distance, right sensor distance, checksum,string end identrifier #
      String packet = String("$")+","+String(state)+","+String(IRDistance,2)+","+String(sonarDistance,2)+","+String(checkSum)+"#";

      Serial.println(packet);


    }
    else // byte based data packet (Faster at both ends)
    {
      uint8_t packet[5];
      packet[0] = 0xA5;                        // Message Identifier
      packet[1] = state;                      // Proximity State
      packet[2] = (uint8_t)IRDistance;                        // Left Distance
      packet[3] = (uint8_t)sonarDistance;                    // Right Distance
      packet[4] = packet[1] ^ packet[2] ^ packet[3]; // Checksum

      Serial.write(packet, 5);                 // Send all bytes
    }
    lastSampleTime = currentMillis;

  }
}
