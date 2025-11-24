# GyroBalance Motorcycle : Code Explanation
BY ASSUMPTION COLLEGE SAMUTPRAKAN GRADE 12th STUDENTS
This document breaks down the code for the `GyroBalance Motorcycle` project line by line, explaining the programming, the physics, and the control theory in an easy-to-understand way.
MAIN CODE : PA.ino
## 1. Header & Includes

```cpp
/*
   ESP32 MOTOCYCLE REACTION WHEEL BALANCER (ANALOGWRITE VERSION)
   --------------------------------------------------------------
   Features:
   - Auto axis detection (tilt L<->R for 5 seconds after boot)
   - Complementary filter (gyro + accel fusion)
   - PD control using gyro as derivative
   - Optional flywheel compensation term Kf * omega
   - Web UI + WebSocket + UDP control
   - analogWrite PWM so it compiles on your ESP32 core
*/

#include <WiFi.h>
#include <WebServer.h>
#include <WebSocketsServer.h>
#include <WiFiUdp.h>
#include <Wire.h>
#include "MPU6050.h"
```
- **Comment Block**: This provides a high-level summary of the project's features. The goal is to balance a single-axis vehicle (like a toy motorcycle) using a "reaction wheel".
- **Includes**: These lines import necessary libraries:
    - `WiFi.h`, `WebServer.h`, `WebSocketsServer.h`, `WiFiUdp.h`: Used for creating a Wi-Fi Access Point and hosting a web page for remote control and monitoring.
    - `Wire.h`: The standard library for I2C communication, which is how the ESP32 talks to the MPU6050 sensor.
    - `MPU6050.h`: A specific library to simplify reading data from the MPU6050 Inertial Measurement Unit (IMU).

---
## 2. Global Objects & Pin Definitions

```cpp
MPU6050 mpu;

// ===== PIN DEFINITIONS =====
#define ENA_PIN 33
#define IN1_PIN 26
#define IN2_PIN 25

#define SDA_PIN 21
#define SCL_PIN 22
```
- `MPU6050 mpu;`: Creates an `mpu` object, which is our interface to the MPU6050 sensor.
- **PIN DEFINITIONS**: These lines assign names to the GPIO pins used on the ESP32.
    - `ENA_PIN`, `IN1_PIN`, `IN2_PIN`: These are for the motor driver. `ENA` (Enable) controls the motor's speed using PWM, while `IN1` and `IN2` control its direction.
    - `SDA_PIN`, `SCL_PIN`: These are the I2C communication pins for the MPU6050. `SDA` is for data, and `SCL` is for the clock signal.

---
## 3. Wi-Fi & Network Configuration

```cpp
// ===== WIFI AP MODE =====
const char* ap_ssid = "ESP32-BIKE";
const char* ap_pass = "12345678";

WebServer server(80);
WebSocketsServer webSocket = WebSocketsServer(81);
WiFiUDP Udp;
const uint16_t UDP_PORT = 4210;
```
- The ESP32 will create its own Wi-Fi network (Access Point) with the given SSID and password.
- `WebServer server(80)`: Creates a web server on port 80 (the standard HTTP port).
- `WebSocketsServer webSocket = WebSocketsServer(81)`: Creates a WebSocket server on port 81. WebSockets provide a fast, two-way communication channel between the web browser and the ESP32, perfect for real-time control and data streaming.
- `WiFiUDP Udp;` & `UDP_PORT`: Sets up the ability to receive commands over UDP on port 4210, another method for real-time control.

---
## 4. Control System Parameters

```cpp
// ===== CONTROL PARAMETERS =====
float Kp = 7.0;      // proportional (for 960RPM, 6–10)
float Kd = 0.55;     // derivative (gyro based)
float Kf = 0.10;     // flywheel compensation

float targetAngle = 0.0;
bool motorsEnabled = false;
```
This is the core of the **PD (Proportional-Derivative) controller**.

- `targetAngle = 0.0;`: The goal. We want the robot to be perfectly vertical, which is an angle of 0 degrees.
- `Kp`: **Proportional Gain**. This determines *how much* to react based on the current error.
    - **Physics**: Think of this like a spring. The further you pull it from its resting position, the harder it pulls back. If the robot is tilted by `error` degrees, the motor output will be proportional to `Kp * error`. A higher `Kp` gives a stronger, faster response, but can lead to overshoot and oscillations.
- `Kd`: **Derivative Gain**. This determines how much to react based on the *rate of change* of the error.
    - **Physics**: Think of this as a damper or shock absorber. It resists fast motion. In our case, it uses the gyroscope's rotation rate (`gyroRate`) to predict where the robot is going. If the robot is falling fast, the `Kd` term applies a strong counter-force to "dampen" the fall. This is crucial for preventing the oscillations caused by an aggressive `Kp`.
- `Kf`: **Flywheel Compensation**. A more advanced term to compensate for the flywheel's own momentum.

---
## 5. IMU & Sensor Fusion Variables

```cpp
// ===== IMU FUSION VARIABLES =====
float angle = 0;        // fused angle
float gyroRate = 0;     // deg/sec
unsigned long lastMicros = 0;
float alpha = 0.98;     // complementary filter constant
```
- **The Problem**: The IMU has an accelerometer and a gyroscope, but both have flaws.
    - **Accelerometer**: Measures acceleration, including the constant acceleration of gravity. It can give you a good idea of the tilt angle, but it's very "noisy" and easily affected by vibrations or movement.
    - **Gyroscope**: Measures the rate of rotation. It's very precise for fast changes, but it has a "bias" that causes its reading to "drift" over time. If you integrate its rate to get an angle, the error will build up continuously.
- **The Solution (Complementary Filter)**: The code combines the best of both sensors.
    - `alpha = 0.98`: This is the filter's tuning constant. It means the new angle will be **98%** of the old angle (trusted gyroscope data) and **2%** of the new accelerometer data.
    - **Physics/Analogy**: Imagine you are walking to a destination. The gyroscope is like your sense of direction for each step you take (very good short-term). The accelerometer is like occasionally looking up at the North Star (gravity) to get your absolute bearing (very good long-term). The filter trusts your steps most of the time but uses the North Star to correct any drift in your path.

---
## 6. Auto-Axis Detection & Safety

```cpp
bool useAngleX = true;
bool axisDetected = false;
const float ANGLE_CUTOFF = 40.0;
```
- `useAngleX`, `axisDetected`: Variables for the automatic axis detection feature, which figures out the correct orientation of the IMU at startup.
- `ANGLE_CUTOFF`: A safety feature. If the robot tilts more than 40 degrees, it's considered to have fallen over. The motors will shut off to prevent the flywheel from spinning uncontrollably on the ground.

---
## 7. Motor Control Logic

```cpp
void motorDrive(int pwm) {
  pwm = constrain(pwm, -255, 255);
  //... (flywheel speed estimation) ...
  int duty = abs(pwm);

  if (pwm > 0) { // Spin one way
    digitalWrite(IN1_PIN, HIGH);
    digitalWrite(IN2_PIN, LOW);
  } else if (pwm < 0) { // Spin the other way
    digitalWrite(IN1_PIN, LOW);
    digitalWrite(IN2_PIN, HIGH);
  } else { // Stop
    digitalWrite(IN1_PIN, LOW);
    digitalWrite(IN2_PIN, LOW);
  }
  analogWrite(ENA_PIN, duty);
}
```
This function translates a desired power value (`pwm`) into signals for the motor driver.
- A positive `pwm` value sets `IN1` HIGH and `IN2` LOW to spin the motor in one direction.
- A negative `pwm` value reverses the polarity to spin it the other way.
- The absolute value of `pwm` is used with `analogWrite` to set the motor's speed (duty cycle).
- `constrain()` ensures the PWM value doesn't exceed the maximum possible range (-255 to 255).

---
## 8. Angle Calculation from Accelerometer

```cpp
float calcAngleX() {
  return atan2((float)ay, (float)az) * 57.2958;
}
float calcAngleY() {
  return atan2((float)ax, (float)az) * 57.2958;
}
```
- **Physics**: When the IMU is perfectly level, the force of gravity (1g) is detected entirely on the Z-axis. When it tilts, that gravity vector is split between the Z-axis and one of the horizontal axes (X or Y).
- **Math**: `atan2` is the "2-argument arctangent" function. It takes the accelerometer readings from two axes (`ay` and `az` for the X-angle) and computes the angle of the gravity vector. This gives us the angle of tilt.
- `* 57.2958`: This converts the angle from radians (which `atan2` returns) to degrees.

---
## 9. Web Interface & Communication

The code includes functions (`webSocketEvent`, `handleUDP`) to process incoming commands (`START`, `STOP`, `ZERO`, and tuning `Kp`, `Kd`, `Kf`) and a large `webpage` string that contains the HTML, CSS, and JavaScript for the control panel you access from a browser.

---
## 10. `setup()` Function

```cpp
void setup() {
  // ... (Serial, Pin Modes) ...
  
  // MPU
  Wire.begin(SDA_PIN, SCL_PIN);
  mpu.initialize();

  // Axis detection
  detectAxis();

  // ... (WiFi, Web Server, WebSocket, UDP startup) ...
  
  lastMicros = micros();
}
```
This function runs once at startup to initialize everything:
1.  Starts Serial communication for debugging.
2.  Sets the motor and I2C pins.
3.  Initializes the MPU6050 sensor.
4.  Runs the `detectAxis()` routine to figure out IMU orientation.
5.  Starts the Wi-Fi network and web services.
6.  Records the initial time for the main loop.

---
## 11. `loop()` Function - The Balancing Core

This is the main function that runs over and over, thousands of times per second.

```cpp
void loop() {
  // ... (Handle web/UDP clients) ...

  // Calculate delta-time (dt) for physics calculations
  unsigned long now = micros();
  float dt = (now - lastMicros) / 1000000.0;
  // ... (dt validation) ...
  lastMicros = now;


  // 1. READ SENSORS
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // 2. FUSE SENSOR DATA (Complementary Filter)
  float accAngle = useAngleX ? calcAngleX() : calcAngleY();
  gyroRate = (float)gy / 131.0; // Convert raw gyro to deg/sec
  angle = alpha * (angle + gyroRate * dt) + (1.0 - alpha) * accAngle;

  // 3. APPLY CONTROL LAW (PD Controller)
  float error = angle - targetAngle;
  float out = Kp * error + Kd * (-gyroRate) + Kf * flywheel_speed;

  // 4. DRIVE THE MOTOR
  if (!motorsEnabled || abs(angle) > ANGLE_CUTOFF) {
    motorDrive(0); // Safety cutoff
  } else {
    motorDrive((int)out);
  }

  // ... (Send status updates over WebSocket) ...
}
```

1.  **Read Sensors**: Gets the latest acceleration and rotation data from the IMU.
2.  **Sensor Fusion**:
    - It calculates the `accAngle` from the accelerometer.
    - It converts the raw gyroscope reading (`gy`) into degrees per second (`gyroRate`).
    - It applies the **complementary filter** to combine the two into one stable `angle` measurement. `gyroRate * dt` calculates how much the angle has changed since the last loop according to the gyroscope.
3.  **Control Law**:
    - `error = angle - targetAngle;`: Calculates how far the robot is tilted from the vertical goal.
    - `out = Kp*error + Kd*(-gyroRate) ...`: This is the PD controller in action. It calculates the required motor power. The `gyroRate` is negative because we want to create a torque that *opposes* the direction of falling.
4.  **Drive Motor**:
    - **Physics (Reaction Wheel)**: The output of the PD controller is sent to the motor. As the motor spins the flywheel, the flywheel gains **angular momentum**. Due to the **conservation of angular momentum**, an equal and opposite torque is applied to the body of the robot. This counter-torque is what pushes the robot back towards vertical.
    - **Example**: If the robot is falling to the right, the controller will spin the flywheel in one direction. This creates a reaction torque that pushes the robot's body to the left, correcting its fall.

This loop of **Measure -> Calculate -> Act** is the fundamental principle of a closed-loop control system and is what allows the robot to balance.
