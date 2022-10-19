# Adaptive Cruise Control with Lane Keep Assist on RC Vehicle

Dependencies:
- RC vehicle
- 1 Arduino Uno board
- 4 ultrasonic sensors
- Connecting wires and breadboard
- Batteries
- Arduino IDE


This project implements Adaptive Cruise Control with Lane Keep Assist on a RC Vehicle using Ultrasonic distance sensors. The sensor data is pre-processed using a Kalman filter to mitigate dynamic and measurement noise. The project is based on the Arduino platform and a board will be required to run the files.

The project can be repeated as follows:

![image](https://user-images.githubusercontent.com/23109324/196822693-2b759857-aa06-46b1-ab60-983d6af010d4.png)

1. Build the above circuit.

2. Run "Calculate_measurement_noise.ino" to get an idea about the ultrasonic sensor's measurement noise. This information will be essential to construct the "R" matrix of the Kalman filter.

3. Run "Kalman_filter_single_sensor.ino" to experience Kalman filtering on incoming distance data from a single ultrasonic sensor. This will finally be relevant for Lane Keep Assist as each side of the vehicle will have one ultrasonic sensor whose data will need to be processed through a Kalman filter.

4. Run "Kalman_filter_sensor_fusion.ino" to experience sensor fusion using a Kalman filter. This will be relevant for Adaptive Cruise Control as the vehicle uses two ultrasonic sensors on the front to estimate distance from an obstacle. The data from these two ultrasonic sensors is fused together to improve the distance accuracy.

![image](https://user-images.githubusercontent.com/23109324/196823457-854916fd-f726-41d1-9807-974728f52462.png)

To proceed further, a setup similar to above will be required. Essentially, an RC car with front steering and rear traction motors is used. This vehicle is attached with an Arduino board and 4 ultrasonic sensors - two on the front and one on each side.

5. Run "ACC_with_Lane_Keep.ino" to perform adaptive cruise control manoeuvres with lane keep assist on the RC vehicle. 
