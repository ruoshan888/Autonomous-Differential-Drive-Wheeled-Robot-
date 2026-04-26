#include "SoftwareSerial.h"
SoftwareSerial BTserial(0, 1); // RX(D0), TX(D1)

const byte encoder1pinA = 2; 
const byte encoder1pinB = 11;
const byte encoder2pinA = 3;
const byte encoder2pinB = 12;
volatile byte encoder1PinALast;
volatile byte encoder2PinALast;

bool isTurning1 = false; // Flag to check if in "U-turn mode"
bool isTurning2 = false; // Flag to check if in "Left turn mode"
bool isTurning3 = false; // Flag to check if in "Right turn mode"
unsigned long turnStartTime = 0; // To store when the turn started
const int turnDuration1 = 3800; // Time in ms to complete 180 degrees 
const int turnDuration2 = 1000; // Time in ms to complete 45 degrees

float constant_W1 = 0.0;
float constant_W2 = 0.0;
int AutoManual = 0;

int setSpeed1 = 0;
int setSpeed2 = 0;
float gf_setW1 = 0.0;
float gf_setW2 = 0.0;

float gf_PIDErrorSum1 = 0.0;
float gf_PIDErrorSum2 = 0.0;
float f_w1Error_old = 0.0;
float f_w2Error_old = 0.0;
float gf_Kd = 0.5;
float gf_Ki = 1.2;
float gf_Kp = 2.0;

#define trigPin 13 // Ultrasonic sensor
#define echoPin 4 // Ultrasonic sensor
unsigned int front_distance_mm;
unsigned int left_distance_mm;
unsigned int right_distance_mm;

volatile int gn_duration1 = 0; 
volatile int gn_duration2 = 0;

float distance = 0.0; // Distance traveled

void setup()
{
  Serial.begin(9600); // Initialize the serial port
  BTserial.begin(9600); // For Bluetooth module

  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(7, OUTPUT);
  pinMode(8, OUTPUT);
  pinMode(9, OUTPUT);
  pinMode(10, OUTPUT);
  pinMode(trigPin, OUTPUT); // Ultrasonic sensor
  pinMode(echoPin, INPUT); // Ultrasonic sensor

  pinMode(encoder1pinA, INPUT_PULLUP);
  pinMode(encoder1pinB, INPUT_PULLUP);
  pinMode(encoder2pinA, INPUT_PULLUP);
  pinMode(encoder2pinB, INPUT_PULLUP);

  encoder1PinALast = digitalRead(encoder1pinA);
  encoder2PinALast = digitalRead(encoder2pinA);

  attachInterrupt(digitalPinToInterrupt(encoder1pinA), wheelSpeed1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoder2pinA), wheelSpeed2, CHANGE);
}

void loop()
{    
  float f_w1, f_w2, f_w1Error, f_w2Error;
  float f_PIDout1, f_PIDout2;
  
  right_distance_mm = unReadIRSensor1(A0); // Read the input pin
  left_distance_mm = unReadIRSensor2(A1); // Read the input pin
  front_distance_mm = ReadUltrasonicSensor(trigPin, echoPin);

  char bytRead;

  if (Serial.available() > 0)
  {
    bytRead = Serial.read();
  }
  
  if (bytRead == 'm') // Manual mode
  {
    AutoManual = 1;
    distance = 0.00;
    gf_setW1 = 0.00;
    gf_setW2 = 0.00;    
  }
  else if (bytRead == 'a') // Auto mode
  {
    AutoManual = 0;
    distance = 0.00;
    gf_setW1 = 0.00; // No movement before command 's'
    gf_setW2 = 0.00; // No movement before command 's'
    constant_W1 = 0.0; // No movement before command 's'
    constant_W2 = 0.0; // No movement before command 's'
  }

  if (AutoManual == 0) // Auto mode
  {
    if (bytRead == 's') // Start moving around
    {
      constant_W1 = 0.4; // Random constant rps
      constant_W2 = 0.4; // Random constant rps
    }

    if (isTurning1 == true) // U-Turn
    {
      if (millis() - turnStartTime >= turnDuration1) // Check if the time is up
      {
        isTurning1 = false; // Turn finished
        gf_setW1 = 0.0; // Stop
        gf_setW2 = 0.0;
      }
      else // Turning on spot
      {
        gf_setW1 = 0.3;   
        gf_setW2 = -0.3; 
      }
    }

    else if (isTurning2 == true) // Left Turn (Obstacle at right)
    {
      if (millis() - turnStartTime >= turnDuration2) // Check if the time is up
      {
        isTurning2 = false; // Turn finished
        gf_setW1 = 0.0; // Stop
        gf_setW2 = 0.0;
      }
      else // Turning on spot
      {
        gf_setW1 = 0.3;   
        gf_setW2 = -0.3; 
      }
    }

    else if (isTurning3 == true) // Right Turn (Obstacle at left)
    {
      if (millis() - turnStartTime >= turnDuration2) // Check if the time is up
      {
        isTurning3 = false; // Turn finished
        gf_setW1 = 0.0; // Stop
        gf_setW2 = 0.0;
      }
      else // Turning on spot
      {
        gf_setW1 = -0.3;   
        gf_setW2 = 0.3; 
      }
    }

    else if(isTurning1 == 0  && isTurning2 == 0 && isTurning3 == 0  && front_distance_mm <= 150) // Object detected in front
    {
      isTurning1 = true; // Start the turning sequence
      turnStartTime = millis(); // Record the current time
    }

    else if(isTurning1 == 0 && isTurning2 == 0 && isTurning3 == 0  && left_distance_mm <= 100) // Object detected at the left side
    {
      isTurning3 = true; // Start the turning sequence
      turnStartTime = millis(); // Record the current time
    }

    else if(isTurning1 == 0 && isTurning2 == 0 && isTurning3 == 0  && right_distance_mm <= 100) // Object detected at the right side
    {
      isTurning2 = true; // Start the turning sequence
      turnStartTime = millis(); // Record the current time
    }

    else // No obstacle.
    {
      gf_setW1 = constant_W1;
      gf_setW2 = constant_W2;
    }
  }

  else if (AutoManual == 1) // Manual mode
  {
    if (bytRead == '6') // Turn right
      gf_setW2 += 0.15; 
    else if (bytRead == '2') // Slow down or move backward
    {
      gf_setW1 -= 0.15;
      gf_setW2 -= 0.15;
    }
    else if (bytRead == '4') // Turn left
      gf_setW1 += 0.15;
    else if (bytRead == '5') // Stop
    {
      gf_setW1 = 0.00;
      gf_setW2 = 0.00;
    }
    else if (bytRead == '8') // Move forward
    {
      gf_setW1 += 0.15;
      gf_setW2 += 0.15;
    }
    if (front_distance_mm <= 100 || left_distance_mm <= 100 || right_distance_mm <= 100)
    {
      gf_setW1 = 0.0;
      gf_setW2 = 0.0;
    }    
  }

  noInterrupts();
  // Pulse1
  f_w1 = (gn_duration1/960.0)*10.0;
  gn_duration1 = 0;

  // Pulse2
  f_w2 = (gn_duration2/960.0)*10.0;
  gn_duration2 = 0;
  interrupts();

  f_w1Error = gf_setW1 - f_w1;
  f_w2Error = gf_setW2 - f_w2;

  gf_PIDErrorSum1 = gf_Ki*f_w1Error + gf_PIDErrorSum1;
  gf_PIDErrorSum2 = gf_Ki*f_w2Error + gf_PIDErrorSum2;
  f_PIDout1 = gf_Kp*f_w1Error + gf_Kd*(f_w1Error - f_w1Error_old) + gf_PIDErrorSum1;
  f_PIDout2 = gf_Kp*f_w2Error + gf_Kd*(f_w2Error - f_w2Error_old) + gf_PIDErrorSum2;
  setSpeed1 = f_PIDout1*960.0/10.0;
  setSpeed2 = f_PIDout2*960.0/10.0;

  f_w1Error_old = f_w1Error;
  f_w2Error_old = f_w2Error;

  SetMotor1(setSpeed1);
  SetMotor2(setSpeed2);

  if (AutoManual == 0)
  {
    Serial.print(left_distance_mm); // Obstacle distance
    Serial.print("; ");

    Serial.print(right_distance_mm); // Obstacle distance
    Serial.print("; ");

    Serial.print(front_distance_mm); // Obstacle distance
    Serial.print("; ");    
  }

  /* Serial.print(f_w1);
  Serial.print(" ");
  Serial.println(f_w2); */

  float d1 = fabs(f_w1) * 0.1 * 2.0 * PI * 0.03;
  float d2 = fabs(f_w2) * 0.1 * 2.0 * PI * 0.03;

  float d = (d1 + d2) / 2.0;  // Robot traveled distance
  distance += d;
  Serial.print(distance);
  Serial.println("m"); 
  
  delay(100);
}

void wheelSpeed1()
{
  byte Lstate = digitalRead(encoder1pinA);
  if((encoder1PinALast == LOW) && Lstate==HIGH)
  {
    byte Bstate = digitalRead(encoder1pinB);
    if(Bstate == LOW)
    {
      gn_duration1--; // Forward
    }
    else
    {
      gn_duration1++; // Reverse
    }
  }
  encoder1PinALast = Lstate;
}

void wheelSpeed2()
{
  byte Lstate = digitalRead(encoder2pinA);
  if((encoder2PinALast == LOW) && Lstate==HIGH)
  {
    byte Bstate = digitalRead(encoder2pinB);
    if(Bstate == LOW)
    {
      gn_duration2--; // Forward
    }
    else
    {
      gn_duration2++; // Reverse
    }
  }
  encoder2PinALast = Lstate;
}

void SetMotor1(int nSpeed)
{
  if (nSpeed >= 0)
  {
    analogWrite(5, nSpeed);
    digitalWrite(7, LOW);
    digitalWrite(8, HIGH);
  }
  else // nSpeed is negative.
  {
    analogWrite(5, -nSpeed);
    digitalWrite(7, HIGH);
    digitalWrite(8, LOW);
  }
}

void SetMotor2(int nSpeed)
{
  if (nSpeed >= 0)
  {
    analogWrite(6, nSpeed);
    digitalWrite(9, LOW);
    digitalWrite(10, HIGH);
  }
  else // nSpeed is negative.
  {
    analogWrite(6, -nSpeed);
    digitalWrite(9, HIGH);
    digitalWrite(10, LOW);
  }
}

unsigned int unReadIRSensor1(unsigned int unPin) // 10cm - 80cm
{
  if (unPin < A0 || unPin > A5) {
    return 0; // Return 0 to indicate invalid pin
  }

  int value = analogRead(unPin); // Read Sensor
  float volts = value * 0.0048828125; // Convert to Voltage
  if (volts < 0.1) 
    volts = 0.1;  // Prevent math explosion

  unsigned int distance_mm = 262.22 * pow(volts, -1.299);

  // Limit the range (100mm - 8000mm)
  if (distance_mm < 100) {
    distance_mm = 100;
  } 
  else if (distance_mm > 8000) {
    distance_mm = 8000;
  }

  return distance_mm;
}

unsigned int unReadIRSensor2(unsigned int unPin) // 4cm - 30cm
{
  if (unPin < A0 || unPin > A5) {
    return 0; // Return 0 to indicate invalid pin
  }

  int value = analogRead(unPin); // Read Sensor
  float volts = value * 0.0048828125; // Convert to Voltage
  if (volts < 0.1) 
    volts = 0.1;  // Prevent math explosion

  unsigned int distance_mm = 123.99 * pow(volts, -1.061);

  // Limit the range (40mm - 300mm)
  if (distance_mm < 40) {
    distance_mm = 40;
  } 
  else if (distance_mm > 300) {
    distance_mm = 300;
  }

  return distance_mm;
}

unsigned int ReadUltrasonicSensor(byte trig, byte echo)
{
  unsigned long duration, distance;

  digitalWrite(trig, LOW);
  delayMicroseconds(2);
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);  
  digitalWrite(trig, LOW);

  duration = pulseIn(echo, HIGH, 25000);
  if (duration == 0)
    duration = 8000; // Assume no obstacle

  distance = duration*0.1718;

  return distance;
}