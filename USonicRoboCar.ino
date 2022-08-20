#include <Servo.h>
#include "TimerOne.h"

Servo servo;

// Ultrasonic Module pins
const int trigPin = A0; // 10 microsecond high pulse causes chirp , wait 50 us
const int echoPin = A1; // Width of high pulse indicates distance

// Servo motor that aims ultrasonic sensor .
const int servoPin = 10; // PWM output for servo

// Constants for Interrupt Pins speed sensors
// Change values if not using Arduino Uno
const byte Encoder_L = 3;  // Left Motor motor Interrupt Pin
const byte Encoder_R = 2;  // Right Motor motor Interrupt Pin

// Motor control pins : L293 H bridge
const int en1Pin = 6; // L_Motor motor PWM speed control
const int en2Pin = 5; // R_Motor motor PWM speed control
const int in1Pin = 8; // L_Motor motor Direction 1 (inPin1 and inPin2 are swapped cause my motor is inverted)
const int in2Pin = 9; // L_Motor motor Direction 2
const int in3Pin = 7; // R_Motor motor Direction 1
const int in4Pin = 4; // R_Motor motor Direction 2


// Constant for steps in disk
const float stepcount = 20.00;  // 20 Slots in disk, change if different

// Integers for pulse counters
volatile int counter_L = 0;
volatile int counter_R = 0;

int rpmL;
int rpmR;

enum Motor{L_Motor, R_Motor};

// Left motor pulse count ISR
void ISR_countL()  
{
  counter_L++;  // increment Left motor counter value
} 
 
// Right motor pulse count ISR
void ISR_countR()  
{
  counter_R++;  // increment Right motor counter value
}

void encodersISR()
{
  static uint32_t previousMillis;

  if (millis() - previousMillis >= 1000) {
    rpmL = (counter_L/stepcount)*60;
    rpmR = (counter_R/stepcount)*60;

    counter_L = 0;
    counter_R = 0;
    
    previousMillis += 1000;
  }

  Serial.print("Left motor speed: ");
  Serial.print(rpmL);
  Serial.println(" rpm");

  Serial.print("Right motor speed: ");
  Serial.print(rpmR);
  Serial.println(" rpm");
}

// Set motor speed: 255 full ahead, -255 full reverse , 0 stop
void go(enum Motor m, int speed)
{
  digitalWrite((m == L_Motor) ? in1Pin: in3Pin, (speed > 0) ? HIGH: LOW); 
  digitalWrite((m == L_Motor) ? in2Pin: in4Pin, (speed <= 0) ? HIGH: LOW);
  analogWrite((m == L_Motor) ? en1Pin: en2Pin, (speed < 0) ? -speed: speed);
}

// Initial motor test :
// Left Motor motor forward then back
// Right Motor motor forward then back
void testMotors ()
{
  static int speed[8] = {128, 255, 128, 0, -128, -255, -128, 0};
  go(R_Motor, 0);

  for (unsigned char i = 0; i < 8; i++)
    go(L_Motor, speed[i]), delay (200);
  for (unsigned char i = 0; i < 8; i++)
    go(R_Motor, speed[i]), delay (200);
}

// Read distance from the ultrasonic sensor , return distance in mm
//
// Speed of sound in dry air , 20C is 343 m/s
// pulseIn returns time in microseconds (10ˆ−6)
// 2d = p * 10ˆ−6 s * 343 m/s = p * 0.00343 m = p * 0.343 mm/us
unsigned int readDistance()
{
  digitalWrite( trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  unsigned long period = pulseIn(echoPin, HIGH);

  return period * 343 / 2000;
}

#define NUM_ANGLES 7
unsigned char sensorAngle[NUM_ANGLES] = {60, 70, 80, 90, 100, 110, 120};
unsigned int distance[NUM_ANGLES];

// Scan the area ahead by sweeping the ultrasonic sensor L_Motor and R_Motor
// and recording the distance observed. This takes a reading , then
// sends the servo to the next angle. Call repeatedly once every 50 ms or so.
void readNextDistance()
{
  static unsigned char angleIndex = 0;
  static signed char step = 1;

  distance[angleIndex] = readDistance();
  angleIndex += step;

  if (angleIndex == NUM_ANGLES - 1) step = -1;
  else if (angleIndex == 0) step = 1;
  
  servo.write(sensorAngle[angleIndex]);
}

// Initial configuration
//
// Configure the input and output pins
// Center the servo
// Turn off the motors
// Test the motors
// Scan the surroundings once
//
void setup() {
  Serial.begin(9600);
  
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  digitalWrite(trigPin, LOW);

  pinMode(en1Pin, OUTPUT);
  pinMode(in1Pin, OUTPUT);
  pinMode(in2Pin, OUTPUT);
  pinMode(in3Pin, OUTPUT);
  pinMode(in4Pin, OUTPUT);
  pinMode(en2Pin, OUTPUT);

  servo.attach(servoPin);
  servo.write(90);
  go(L_Motor, 0);
  go(R_Motor, 0);
  testMotors();

  // Scan the surroundings before starting
  servo.write(sensorAngle[0]);
  delay(200);
  for (unsigned char i = 0; i < NUM_ANGLES; i++) {
    readNextDistance (), delay(200);
  }

  // Attach the Interrupts to their ISR's
  attachInterrupt(digitalPinToInterrupt (Encoder_L), ISR_countL, RISING);  // Increase counter L when speed sensor pin goes High
  attachInterrupt(digitalPinToInterrupt (Encoder_R), ISR_countR, RISING);  // Increase counter R when speed sensor pin goes High
}

// Main loop:
//
// Get the next sensor reading
// If anything appears to be too close , back up
// Otherwise, go forward
//
void loop() {
  readNextDistance();
  // See if something is too close at any angle
  unsigned char tooClose = 0;
  encodersISR();

  for (unsigned char i = 0; i < NUM_ANGLES; i++) {
    if (distance[i] < 300) tooClose = 1;
  }    
  if (tooClose) {
    // Something's nearby: back up left motor
    go(L_Motor, -180);
    go(R_Motor, 180);
  } else {
    // Nothing in our way: go forward
    go(L_Motor, 255);
    go(R_Motor, 255);
  }

  // Check the next direction in 50 ms
  delay (50);
}