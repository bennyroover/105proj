/*
* Firmware for the ”2WD Ultrasonic Motor Robot Car Kit”
*
* Stephen A. Edwards
*
* Hardware configuration :
* A pair of DC motors driven by an L298N H bridge motor driver
* An HC-SR04 ultrasonic range sensor mounted atop a small hobby servo
*/
#include <Servo.h>
Servo servo;

// Ultrasonic Module pins
const int trigPin = 13; // 10 microsecond high pulse causes chirp , wait 50 us
const int echoPin = 12; // Width of high pulse indicates distance

// Servo motor that aims ultrasonic sensor .
const int servoPin = 11; // PWM output for hobby servo

// Motor control pins : L298N H bridge
const int enAPin = 6; // Left motor PWM speed control
const int in1Pin = 7; // Left motor Direction 1
const int in2Pin = 5; // Left motor Direction 2
const int in3Pin = 4; // Right motor Direction 1
const int in4Pin = 8; // Right motor Direction 2
const int enBPin = 3; // Right motor PWM speed control

enum Motor {LEFT, RIGHT};
#define NUM_ANGLES 7
unsigned char sensorAngle[NUM_ANGLES] = { 60, 70, 80, 90, 100, 110, 120 };
unsigned int distance[NUM_ANGLES];

// Set motor speed: 255 full ahead, -255 full reverse , 0 stop
void go( enum Motor m, int speed)
{
  digitalWrite(m == LEFT ? in1Pin : in3Pin , speed > 0 ? HIGH : LOW);
  digitalWrite(m == LEFT ? in2Pin : in4Pin , speed <= 0 ? HIGH : LOW);
  analogWrite(m == LEFT ? enAPin : enBPin, speed < 0 ? -speed : speed);
}

bool sinDrive(double freq, int duration) { // duration in seconds
  static int tstart;
  static bool init = true;

  if (init) {
     tstart = millis();
     init = false;
  }
  
  double t = millis();
  int speed;

//  Serial.print("tstart is ");
//  Serial.print(tstart);
//  Serial.print(" t is ");
//  Serial.println(t);


  if (t < tstart + duration*1000) {
    speed = 255*sin(2*PI*freq*t/1000);
    go(LEFT, speed);
    go(RIGHT, -1*speed);
    return true;
  }

  go(LEFT, 0);
  go(RIGHT, 0);
  init = true;
  return false;
  
}

// Initial motor test :
// left motor forward then back
// right motor forward then back
void testMotors ()
{
  static int speed[8] = { 128, 255, 128, 0 ,
  -128, -255, -128, 0};
  go(RIGHT, 0);
  for (unsigned char i = 0 ; i < 8 ; i++)
    go(LEFT, speed[i]*0.85), delay (200);
  for (unsigned char i = 0 ; i < 8 ; i++)
    go(RIGHT, speed[i]), delay (200);
}

// Read distance from the ultrasonic sensor , return distance in mm
//
// Speed of sound in dry air , 20C is 343 m/s
// pulseIn returns time in microseconds (10ˆ-6)
// 2d = p * 10ˆ-6 s * 343 m/s = p * 0.00343 m = p * 0.343 mm/us
unsigned int readDistance ()
{
  digitalWrite ( trigPin , HIGH );
  delayMicroseconds (10);
  digitalWrite ( trigPin , LOW );
  unsigned long period = pulseIn ( echoPin, HIGH );
  return period * 343 / 2000;
}

// Scan the area ahead by sweeping the ultrasonic sensor left and right
// and recording the distance observed. This takes a reading , then
// sends the servo to the next angle. Call repeatedly once every 50 ms or so.
void readNextDistance ()
{
  static unsigned char angleIndex = 0;
  static signed char step = 1;
  distance[angleIndex] = readDistance();
  angleIndex += step ;
  if (angleIndex == NUM_ANGLES - 1) step = -1;
  else if (angleIndex == 0) step = 1;
  servo.write(sensorAngle[angleIndex]);
}

void setup() {
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  digitalWrite (trigPin, LOW);
  pinMode(enAPin, OUTPUT);
  pinMode(in1Pin, OUTPUT);
  pinMode(in2Pin, OUTPUT);
  pinMode(in3Pin, OUTPUT);
  pinMode(in4Pin, OUTPUT);
  pinMode(enBPin, OUTPUT);
  servo.attach(servoPin);
  servo.write(90);
//  go(LEFT, 0);
//  go(RIGHT, 0);
//  testMotors();
//  // Scan the surroundings before starting
//  servo.write (sensorAngle[0]);
//  delay (200);
//  for (unsigned char i = 0; i < NUM_ANGLES; i++)
//    readNextDistance(), delay (200);
  go(LEFT, 0);
  go(RIGHT, 0); 

  Serial.begin(9600);
}



void loop() {
//  readNextDistance();
//  // See if something is too close at any angle
//  unsigned char tooClose = 0;
//  for (unsigned char i = 0; i<NUM_ANGLES; i++)
//    if (distance[i] < 300)
//      tooClose = 1;
//      
//  if (tooClose) {
//  // Something's nearby: back up left
//    go(LEFT, -180);
//    go(RIGHT, -80);
//  } else {
//    // Nothing in our way: go forward
//    go(LEFT, 255);
//    go(RIGHT, 255);
//  }
//  // Check the next direction in 50 ms
//  delay(50);

  double duration = 3;
  const int NFREQ = 12;

  int freq[NFREQ] = {0.5000, 0.7600, 1.1551, 1.7556, 2.6683, 4.0557, 6.1642, 9.3691, 14.2402, 21.6438, 32.8967, 50.0000};
  
  for (int i = 0; i < NFREQ; i++) {
    while (sinDrive(freq[i], duration)) {
      
    }
  }

}
