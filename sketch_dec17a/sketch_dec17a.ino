const int lightSensor = A0;
int reading = 0;

// Motor control pins : L298N H bridge
const int enAPin = 6; // Left motor PWM speed control
const int in1Pin = 7; // Left motor Direction 1
const int in2Pin = 5; // Left motor Direction 2
const int in3Pin = 4; // Right motor Direction 1
const int in4Pin = 8; // Right motor Direction 2
const int enBPin = 3; // Right motor PWM speed control

enum Motor {LEFT, RIGHT};

// Set motor speed: 255 full ahead, -255 full reverse , 0 stop
void go( enum Motor m, int speed)
{
  digitalWrite(m == LEFT ? in1Pin : in3Pin , speed > 0 ? HIGH : LOW);
  digitalWrite(m == LEFT ? in2Pin : in4Pin , speed <= 0 ? HIGH : LOW);
  analogWrite(m == LEFT ? enAPin : enBPin, speed < 0 ? -speed : speed);
}



void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
}


// max error = 50 (go left)
// min error = -50 (go right)
bool line_pid(int reference, int duration) {
  const int speed_baseline = 175;
  
  static int tstart = millis();
  static float e_prev = 0;
  static float e_accum = 0;
  float e = analogRead(lightSensor) - reference;
  int output;

  const int kp = 5;
  const int kd = 10;
  const int ki = 0;
  int left_ctrl, right_ctrl;

  output = kp*e + kd*(e - e_prev) + ki*e_accum;
  if (output > 100) output = 100;
  if (output < -100) output = -100;  //saturate output
  
//right motor control
  if (output > 0){
    right_ctrl = speed_baseline + output*(255-speed_baseline)/100;
  }
  else{
    right_ctrl = speed_baseline+output*speed_baseline/100;
  }

//left motor control
   if (output > 0){
    left_ctrl = speed_baseline + output*-speed_baseline/100;
  }
  else{
    left_ctrl = speed_baseline + output*(speed_baseline-255)/100;
  }
  
  go(LEFT, left_ctrl*0.85);
  go(RIGHT, right_ctrl);

  e_prev = e;
  e_accum = e_accum + e;

  if (millis() % 500 == 0) {
    Serial.print("Output is:\t");
    Serial.println(output);
    Serial.print("left_ctrl is\t");
    Serial.println(left_ctrl);
    Serial.print("right_ctrl is\t");
    Serial.println(right_ctrl);
    Serial.println();
  }
  

  if (millis() < tstart + duration*1000)
    return true;
  else
    return false;
}

void loop() {
  // put your main code here, to run repeatedly:
  int reference = 70;

//  Serial.println('Move bot over reference and press a key to confirm');
//  while(!Serial.available()) {
//    reference = analogRead(lightSensor);
//    if (millis() % 250 == 0) {
//      Serial.println(reference);
//    }
//  }
//
//  Serial.print("Reference is ");
//  Serial.println(reference);
  
  while (line_pid(reference, 100));
  while(1);
}
