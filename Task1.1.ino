// Group 13
// Connect Trigger pin to digital I/O 13 and Echo pin to digital I/O 12
#define trigPin 13
#define echoPin 12
#define ledPin 6
#define buzzerPin 5

// Declaring variance and Kalman filter variables
float measure_variance;             // Measured data covariance
float pred_variance;                // Predicted covariance
float est_variance;                 // Estimated covariance
float yPred;                        // Predicted state
float yMeas;                        // Measured state
float yEst;                         // Estimated state

// Calculated R values for 10cm to 150 cm in intervals of 10cm
float rValues[15] = [0.25, 0.19, 0.80, 0.29, 0.22, 0.69, 0.27, 0.42, 0.22, 0.33, 0.26, 0.29, 0.38, 0.51, 0.32];
float varThreshold = 0.02;          // Variance threshold at which to stop filtering process

// Other variables
bool hasPrinted = 0;                // Whether results have been printed or not (after Kalman filtering is finished)
float Time1, Time2;                 // Time values before and after Kalman filter process

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(ledPin, OUTPUT);
  pinMode(buzzerPin, OUTPUT);

  // Switching off led and buzzer
  digitalWrite(ledPin, LOW);
  digitalWrite(buzzerPin, LOW);

  // Beginning Kalman convergence time measurement
  Time1 = millis();
  Serial.println("START");

  // Choosing appropriate R (measurement covariance) using first measurement
  triggerSensor();
  yEst = pulseIn(echoPin, HIGH);                          // First estimated value for Kalman filter
  int indexForR = (calibrationDistance(yEst) / 10) / 10;  // Calculating required index in rValues
  measure_variance = rValues[indexForR];                  // R value
  est_variance = measure_variance;                        // Initializing estimated variance at R
}

void loop() {
  while(est_variance > varThreshold){
    // Prediction Stage
    yPred = yEst;
    pred_variance = est_variance;
  
    triggerSensor();
    yMeas = pulseIn(echoPin, HIGH);  // Duration in microseconds
    
    // Correction Stage
    float kalman_gain = pred_variance /(pred_variance + measure_variance);
    yEst = yPred + kalman_gain*(yMeas - yPred);
    est_variance =  (1 - kalman_gain)*pred_variance;
    
    Serial.print(yEst);
    Serial.print(" microseconds | Variance = ");
    Serial.println(est_variance);
  
    delay(50);
  }
  if(hasPrinted == 0){
    Time2 = millis();
    
    float distance = calibrationDistance(yEst);
    Serial.print(yEst);
    Serial.print(" microseconds | Variance = ");
    Serial.print(est_variance);
    Serial.print(" | Distance = ");
    Serial.print(distance);
    Serial.print(" mm | Total Time = ");
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

void triggerSensor() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
}

float calibrationDistance(float duration) {
  return duration / 2 * 0.3435;       // returns distance values in mm
}
