// Group 13
// Connect Sensor 1 Trigger pin to digital I/O 13 and Echo pin to digital I/O 12
// Connect Sensor 2 Trigger pin to digital I/O 11 and Echo pin to digital I/O 10
#define trigPin1 13
#define echoPin1 12
#define trigPin2 11
#define echoPin2 10
#define ledPin 6
#define buzzerPin 5

// Declaring variance and Kalman filter variables
float pred_variance;                 // Predicted covariance
float measure_variance1;             // Measured data covariance of sensor 1
float measure_variance2;             // Measured data covariance of sensor 2
float est_variance1;                 // Estimated covariance after 1st Correction
float est_variance2;                 // Estimated covariance after 2nd Correction
float yPred;                         // Predicted state
float yMeas1;                        // Measured state from sensor 1
float yMeas2;                        // Measured state from sensor 2
float yEst1;                         // Estimated state after 1st Correction
float yEst2;                         // Estimated state after 2nd Correction

// Calculated distance-based R values for 10cm to 150 cm in intervals of 10cm
float rValues1[15] = [0.25, 0.19, 0.80, 0.29, 0.22, 0.69, 0.27, 0.42, 0.22, 0.33, 0.26, 0.29, 0.38, 0.51, 0.32];  // For sensor 1
float rValues2[15] = [0.25, 0.19, 0.80, 0.29, 0.22, 0.69, 0.27, 0.42, 0.22, 0.33, 0.26, 0.29, 0.38, 0.51, 0.32];  // For sensor 2
float varThreshold = 0.02;           // Variance threshold at which to stop filtering process

// Other variables
bool hasPrinted = 0;                // Whether results have been printed or not (after Kalman filtering is finished)
float Time1, Time2;                 // Time values before and after Kalman filter process

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(trigPin1, OUTPUT);
  pinMode(echoPin1, INPUT);
  pinMode(trigPin2, OUTPUT);
  pinMode(echoPin2, INPUT);
  pinMode(ledPin, OUTPUT);
  pinMode(buzzerPin, OUTPUT);

  // Switching off led and buzzer
  digitalWrite(ledPin, LOW);
  digitalWrite(buzzerPin, LOW);

  // Beginning Kalman convergence time measurement
  Time1 = millis();
  Serial.println("START");

  // Choosing appropriate R (measurement covariance) using first measurement
  triggerSensor1();
  yEst1 = calibrationFunc1(pulseIn(echoPin1, HIGH));
  int indexForR1 = (calibrationFunc1(yEst1) / 10) / 10;  // Calculating required index in rValues1
  delay(10);
  triggerSensor2();
  yEst2 = calibrationFunc2(pulseIn(echoPin2, HIGH));     // First estimated value for Kalman filter
  int indexForR2 = (calibrationFunc2(yEst2) / 10) / 10;  // Calculating required index in rValues2
  delay(10);
  
  measure_variance1 = rValues1[indexForR1];              // R value for sensor 1
  measure_variance2 = rValues2[indexForR2];              // R value for sensor 2
  est_variance2 = measure_variance2;                     // Initializing estimated variance at R2
}

void loop() {
  while(est_variance2 > varThreshold){
    // Prediction Stage
    yPred = yEst2;
    pred_variance = est_variance2;
  
    // Correction Stage 1
    triggerSensor1();
    yMeas1 = calibrationFunc1(pulseIn(echoPin1, HIGH));  // Distance in mm
    float kalman_gain1 = pred_variance /(pred_variance + measure_variance1);
    yEst1 = yPred + kalman_gain1*(yMeas1 - yPred);
    est_variance1 =  (1 - kalman_gain1)*pred_variance;
    delay(10);
    
    // Correction Stage 2
    triggerSensor2();
    yMeas2 = calibrationFunc2(pulseIn(echoPin2, HIGH));  // Distance in mm
    float kalman_gain2 = est_variance1 /(est_variance1 + measure_variance2);
    yEst2 = yEst1 + kalman_gain2*(yMeas2 - yEst1);
    est_variance2 =  (1 - kalman_gain2)*est_variance1;
    delay(10);
    
    Serial.print("Filtered = ");
    Serial.print(yEst2);
    Serial.print(" mm | Raw = (");
    Serial.print(yMeas1);
    Serial.print(", ");
    Serial.print(yMeas2);
    Serial.print(") | Variance = ");
    Serial.println(est_variance2);
  }
  if(hasPrinted == 0){
    Time2 = millis();
    
    Serial.print(yEst2);
    Serial.print(" mm | Variance = ");
    Serial.print(est_variance);
    Serial.print(" | Total Time = ");
    Serial.print(Time2 - Time1);
    Serial.println(" milliseconds");

    // Flashing led and sounding buzzer
    digitalWrite(ledPin, HIGH);
    digitalWrite(buzzerPin, HIGH);
    delay(3000);
    digitalWrite(buzzerPin, LOW);
  
    hasPrinted = 1;
  }
}

// Trigger Function for sensor 1
void triggerSensor1() {
  digitalWrite(trigPin1, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin1, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin1, LOW);
}

// Trigger Function for sensor 2
void triggerSensor2() {
  digitalWrite(trigPin2, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin2, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin2, LOW);
}

// Calibration function for sensor 1
float calibrationFunc1(float duration) {
  return duration / 2 * 0.3435;       // returns distance values in mm
}

// Calibration function for sensor 2
float calibrationFunc2(float duration) {
  return duration / 2 * 0.3435;       // returns distance values in mm
}
