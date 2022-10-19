#include <Servo.h> //define the servo library
#define trigPin1 6  //Trigger Pin of Front ultrasonic sensor
#define echoPin1 7  //Echo Pin of Front ultrasonic sensor
#define trigPin2 4  //Trigger Pin of Left ultrasonic sensor
#define echoPin2 5  //Echo Pin of Left ultrasonic sensor
#define trigPin3 2  //Trigger Pin of Right ultrasonic sensor
#define echoPin3 3  //Echo Pin of Right ultrasonic sensor


// Servo instantiation
Servo ssm; //create an instance of the servo object called ssm
Servo esc; //create an instance of the servo object called esc

// Steering and Throttle Values
float steering;                   // Control input for steering (between 0 to 180)
float throttle;                   // Control input for throttle (between 70 to 110)
int maxThrottle = 100;            // Maximum throttle input
int minThrottle = 82;             // Minimum throttle input
int maxSteering = 125;            // Maximum steering input
int minSteering = 55;             // Minimum steering input
float pwm = 0.4;                  // Pulse Width Modulation percentage
int pwmSlot = 1000;                // Total time delay between distance measurements

// General Distances
int sensorToBumper = 12;                          // Distance between front sensor and bumper (in cm)
float frontDistance, prevFrontDistance;           // Distance of front bumper from obstacle in front (in cm)
float leftDistance, prevLeftDistance;             // Distance of left sensor from obstacle in left (in cm)
float rightDistance, prevRightDistance;           // Distance of right sensor from obstacle in right (in cm)
int maxDistance = 800;                            // Maximum distance that ultrasonic sensor can measure in front (in cm)
int maxSideDistance = 70;                         // Maximum distance that ultrasonic sensor can measure on the sides (in cm)

// Throttle Parameters
float e_thr = 0, e1_thr =0, e2_thr = 0;                         // Errors at time t, t-1, and t-2
float kp_thr = 0.38 , ki_thr = 0.0003, kd_thr = 0.00007;        // PID parameters for throttle control
float k1_thr = kp_thr + ki_thr + kd_thr;
float k2_thr = -(kp_thr + 2*kd_thr);
float k3_thr = kd_thr;

// Steering Parameters
float e_str = 0, e1_str =0, e2_str = 0;                         // Errors at time t, t-1, and t-2
float kp_str = 0.7 , ki_str = 0, kd_str = 0;                    // PID parameters for steering control
float k1_str = kp_str + ki_str + kd_str;
float k2_str = -(kp_str + 2*kd_str);
float k3_str = kd_str;

void setup() {
  Serial.begin(9600);                          //start serial connection. Uncomment for PC
  ssm.attach(10);                                 //define that ssm is connected at pin 10
  esc.attach(11);                                 //define that esc is connected at pin 11
  pinMode(trigPin1, OUTPUT);
  pinMode(echoPin1, INPUT);
  pinMode(trigPin2, OUTPUT);
  pinMode(echoPin2, INPUT);
  pinMode(trigPin3, OUTPUT);
  pinMode(echoPin3, INPUT);

  // Initialzing steering and throttle
  steering = 90;
  throttle = 90;
  setVehicle(steering, throttle);
  delay(500);
  
  // Initializing front distance
  triggerSensor1();
  float sensor1Read = pulseIn(echoPin1, HIGH);                                     // Time in microseconds
  frontDistance = min(maxDistance, distancefunc(sensor1Read)) - sensorToBumper;    // Distance in cm

  // Initializing left distance
  triggerSensor2();
  float sensor2Read = pulseIn(echoPin2, HIGH);                        // Time in microseconds
  leftDistance = min(maxSideDistance, distancefunc(sensor2Read));     // Distance in cm

  // Initializing right distance
  triggerSensor3();
  float sensor3Read = pulseIn(echoPin3, HIGH);                        // Time in microseconds
  rightDistance = min(maxSideDistance, distancefunc(sensor3Read));    // Distance in cm
  
  delay(100);
}

void loop() {
  // Measure distance from front object
  prevFrontDistance = frontDistance;
  triggerSensor1();
  float sensor1Read = pulseIn(echoPin1, HIGH);                    // Time in microseconds
  frontDistance = distancefunc(sensor1Read) - sensorToBumper;     // Distance in cm
  
  // Throttle PID Control
  e_thr = 0;
  e2_thr = e1_thr;
  e1_thr = e_thr;
  e_thr = -(30 - frontDistance);           // Compute current control error
  if(frontDistance < 27){
    throttle = 86;
    throttle = throttle + (k1_thr*e_thr) + (k2_thr*e1_thr) + (k3_thr*e2_thr);
  }
  else if(frontDistance >= 27 && frontDistance <= 33){
    throttle = 90;
  }
  else{
    throttle = 94;
    throttle = throttle + (k1_thr*e_thr) + (k2_thr*e1_thr) + (k3_thr*e2_thr);
  }
  throttle = throttle + (k1_thr*e_thr) + (k2_thr*e1_thr) + (k3_thr*e2_thr);
  throttle = max(min(maxThrottle, throttle), minThrottle);
//  Serial.print("Front Distance = ");
//  Serial.print(frontDistance);
//  Serial.print(", e = ");
//  Serial.print(e_thr);
//  Serial.print(" ");
//  Serial.print(", e1 = ");
//  Serial.print(e1_thr);
//  Serial.print(", k1 = ");
//  Serial.print(k1_thr);
//  Serial.print(", k2 = ");
//  Serial.print(k2_thr);
//  Serial.print(", Throttle = ");
//  Serial.println(throttle);

  // Measure distance from left object
  prevLeftDistance = leftDistance;
  triggerSensor2();
  float sensor2Read = pulseIn(echoPin2, HIGH);        // Time in microseconds
  leftDistance = distancefunc(sensor2Read);           // Distance in cm
  if(leftDistance > maxSideDistance){
    leftDistance = prevLeftDistance;
  }
//  else if(abs(leftDistance - prevLeftDistance) > 20){
//    leftDistance = prevLeftDistance;
//  }

  // Measure distance from right object
  prevRightDistance = rightDistance;
  triggerSensor3();
  float sensor3Read = pulseIn(echoPin3, HIGH);        // Time in microseconds
  rightDistance = distancefunc(sensor3Read);          // Distance in cm
  if(rightDistance > maxSideDistance){
    rightDistance = prevRightDistance;
  }
//  else if(abs(rightDistance - prevRightDistance) > 20){
//    rightDistance = prevRightDistance;
//  }

  // Steering PID Control
  e2_str = e1_str;
  e1_str = e_str;
  e_str = -(leftDistance - rightDistance);            // Compute current control error
  steering = steering + (k1_str*e_str) + (k2_str*e1_str) + (k3_str*e2_str);
  if(abs(e_str) < 2){
    steering = 90;
  }
  steering = max(minSteering, min(maxSteering, steering));
  Serial.print("Left Distance = ");
  Serial.print(leftDistance);
  Serial.print(", Right Distance = ");
  Serial.print(rightDistance);
  Serial.print(", Steering = ");
  Serial.println(steering);

  
  // Call the setVehicle function to set the vehicle steering and velocity(speed) values 
  setVehicle(steering, throttle);
  delay(pwmSlot * pwm);
  setVehicle(steering, 90);
  delay(pwmSlot * (1-pwm));  
}

//********************** Vehicle Control **********************//
//***************** Do not change below part *****************//
void setVehicle(int s, int v) 
{
  s=min(max(0,s),180);  //saturate steering command
  v=min(max(70,v),105); //saturate velocity command
  ssm.write(s); //write steering value to steering servo
  esc.write(v); //write velocity value to the ESC unit
}
//***************** Do not change above part *****************//

float distancefunc(float duration){
  return (duration/2)*0.03435 ; // Distance in cm
}

void triggerSensor1(){
  digitalWrite(trigPin1, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin1, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin1, LOW);
}

void triggerSensor2(){
  digitalWrite(trigPin2, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin2, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin2, LOW);
}

void triggerSensor3(){
  digitalWrite(trigPin3, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin3, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin3, LOW);
}
