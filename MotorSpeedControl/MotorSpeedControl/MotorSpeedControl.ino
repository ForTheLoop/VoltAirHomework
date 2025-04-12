/*
Project: Voltair.ai Firmware/Embedded Engineer Homework (Option A)  DC Motor Control
Author: Ahmed Murtata Qureshi
Git: https://github.com/ForTheLoop/VoltAirHomework.git 
Demo: https://drive.google.com/drive/folders/1PDjN1kMtctyUm38Qicdr28if1JNoRU5d?usp=drive_link

Summary:- This project is to demonstrate motor speed control using PID control when a set point is set. 
The project uses arduino R4 as main controller, A L298 H Bridge driver,  74HC14D Encoder to measure RPM and 6V DC motor

Code Flow:
1. Initialize Serial, I/O and interrupts (Encoder)
2. Recieve Motor Set point via Serial - 0 to stop, 1 to 120RPM, 2 to 150RPM and 3 to 180RPM
3. Calculate RPM
4. Calculate error and PWM values using PID control 
5. PWM to motor Pin

Usage:- Using the serial port send intergers represented by enum (line 55) to trigger RPM set points.
Note: You can change the pre set values of set point (line 64)
Change Baudrate to match your serial mointor speed (line 72)

*/

const uint8_t maxRot = 20;//Total slots on encoder Disc
uint8_t targetRPM = 0;// min 120 max 180

const uint8_t interruptPin = 2;//EncoderPin
const uint8_t mtrDirPinA = 7;//Hbridge Dir+ Pin
const uint8_t mtrDirPinB = 8;//HBridge Dir- Pin
const uint8_t PWMPin = 6; //Pin at which PWM signal is generated

volatile uint8_t counter;// This is counter that is update in the ISR for the encoder
uint8_t rotations;// This is the number of complete revolution the disc has made.
float rotationPerMin;// this is the calucalted RPM

unsigned long previousMillis = 0; //used calculate time elapsed to determine when to calculate RPM
unsigned long lastMillis = 0;//used calculate delta time for PID control
unsigned long RPMTimeInterval = 600;// allow RPM calucalution every 600 ms

float errSum;// Integral Error
int previousError = 0;  //used to calculate Derrivative error

int PWMValue; //Calculated PWM value

//Ziegler-Nichols Tuning Method
float Kp = 0.18; // Proportional Gain
float Kd = 0.10575;// Derivate Gain
float Ki = 0.07659;// Integral Gain

//Manual Tuning
/*
float Kp = 0.08;
float Kd = 0.01;
float Ki = 0.001;
*/

enum PWMSteps//pre set steps to determine set points
{
  Stop,
  Slow,
  Med,
  Fast
};

const uint8_t SLOW_RPM = 120;//Slow PRM set point
const uint8_t MED_RPM = 150;//Med RPM set point
const uint8_t FAST_RPM = 180;//Fast RPM set point

void setup() 
{
  Serial.begin(9600); // Set up serial at 9600 Baud Rate

  //Set motor pins as Outputs
  pinMode(mtrDirPinA, OUTPUT); 
  pinMode(mtrDirPinB, OUTPUT);
  //set pwm pin as out put
  pinMode(PWMPin, OUTPUT);

  // set encoder pin as input
  pinMode(interruptPin, INPUT);

  //set up interrupt, its ISR and what its signal feature it should be looking for 
  attachInterrupt(digitalPinToInterrupt(interruptPin), ReadEncoderISR, RISING);

  //set motor directions
  digitalWrite(mtrDirPinA,HIGH);
  digitalWrite(mtrDirPinB,LOW);
}

void loop() 
{
  
  if (Serial.available() > 0)// Check if any data on serial buffer 
  {
    int input = Serial.parseInt();// from string to int values
    if (input > -1 && input < 4) // only 4 states 0 to 3 so making sure no out of range values are entered to avoid undesirable behaviour.
    {
        switch(input)
        {
          case Stop:
            targetRPM = 0;
          break;
          case Slow:
            targetRPM = SLOW_RPM;
          break;
          case Med:
            targetRPM = MED_RPM;
          break;
          case Fast:
            targetRPM = FAST_RPM;
          break;
        }
    }
    Serial.println(targetRPM); // Echo the RPM set by user
    while (Serial.available()) Serial.read(); // Clear buffer
  }

  RotationsPerMin();//Calculates RPM
  

  if(rotationPerMin>0)// if motor isnt moving dont print anything
  {
    Serial.print("RPM:"); 
    Serial.println(rotationPerMin);
    /* for PID tuning purposes
      float time = millis()/1000.0;
      Serial.print(rotationPerMin);
      Serial.print(",");
      Serial.print(time);
      Serial.println();
    */

  }

  unsigned long currentMillis = millis();// get current time from timer1
  unsigned long deltaT = currentMillis-lastMillis; //determine loop time
  if(deltaT >= 100)// Change Magic number store this value somewhere
  {
    float dt = deltaT / 1000.0; // delta t in seconds
    int error = targetRPM - rotationPerMin; // Calculate Proprtional Error

    float dError = (error - previousError)/dt;//calculate deritvative error
    errSum += error*dt;// calculate intergeral error
    errSum = constrain(errSum, -1000, 1000);// make sure things dont get out of hand if error is very large

    int NextStep = (int)((Kp*error)+ (Kd*dError) +(Ki*errSum));//Incremental PID has is less agressive but smoother control in noisy enviorment
    PWMValue += NextStep;

    PWMValue = constrain(PWMValue, 0, 255);// make sure PWM values does exceed 8bits


    previousError = error;

    analogWrite(PWMPin, PWMValue);// Generate PWM
    lastMillis = currentMillis;
  }
}

// Encoders ISR
void ReadEncoderISR()
{
  counter++;
}

//Calculates RPM
void RotationsPerMin()
{
  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= RPMTimeInterval) 
  {
    uint8_t pulses = counter;
    counter = 0;
    rotationPerMin = (pulses*60000)/(maxRot*RPMTimeInterval);//PRM formula
    previousMillis = currentMillis;
  }

}

