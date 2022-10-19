// Group 13
// Connect Trigger pin to digital I/O 13 and Echo pin to digital I/O 12
#define trigPin 13
#define echoPin 12

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  // Calculating R (measurement covariance) by taking sample measurements
  numSamples = 100;                           // Number of samples taken to calculate R
  float duration[numSamples];                 // Array to store time duration values from samples
  float durSum = 0;                           // Sum of time durations (to calculate mean)
  for(int i=0; i<numSamples; i++){
    triggerSensor();
    duration[i] = pulseIn(echoPin, HIGH);
    durSum += duration[i];
    Serial.println(duration[i], 2);
    delay(10);
  }
  float durMean = durSum / numSamples;        // Calculating mean of time durations (to calculate variance)
  float durVar = 0;                           // Helper variable to calculate variance
  for(int i=0; i<numSamples; i++){
    durVar += (duration[i] - durMean)*(duration[i] - durMean);
  }
  measure_variance = durVar / numSamples;     // Calculated duration variance
  Serial.print("R = ");
  Serial.println(measure_variance);
}

void loop() {
  // Empty
}

void triggerSensor() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
}
