# ESP32 Motorcycle Balancer (V2): Code & Physics Explanation

This document provides a detailed analysis of the `PA2.ino` code, an advanced version of the self-balancing motorcycle project. It now includes a rear-drive motor and steering, turning it into a fully functional vehicle.

We will cover the new code features and then take a deep dive into the laws of physics that make this project possible.

---

## Part 1: Code Analysis (`PA2.ino`)

This version builds upon the first by adding components and logic for movement and steering, making the web interface more comprehensive.

### 1. New Features Overview
- **Rear-Drive Motor**: The motorcycle can now move forward and backward.
- **Steering Servo**: The motorcycle can now be steered left and right.
- **Expanded Web UI**: The web page now includes buttons for driving and steering in addition to the balancing controls.

### 2. Code Breakdown

#### Includes and Pin Map
```cpp
#include <ESP32Servo.h>

// Rear motor (single)
#define DRV_IN1 19
#define DRV_IN2 18

// Steering servo
#define SERVO_PIN 14
```
- A new library, `ESP32Servo.h`, is included to control the steering servo motor.
- New pins are defined for the **rear drive motor** (`DRV_IN1`, `DRV_IN2`) and the **steering servo** (`SERVO_PIN`).

#### Servo Control
```cpp
Servo steeringServo;
int SERVO_CENTER = 90;
int SERVO_LEFT = 50;
int SERVO_RIGHT = 130;
```
- A `Servo` object is created.
- Integers are defined to represent the servo positions for center, left, and right, making the code more readable. A standard servo responds to angles between 0 and 180 degrees.

#### Web Page (`page[]`)
The HTML content now includes a more complex control panel:
- **Balance Control**: `START` and `STOP` buttons for the flywheel.
- **Drive Control**: `FORWARD` and `BACK` buttons for the rear wheel.
- **Steering Control**: `LEFT`, `RIGHT`, and `STRAIGHT` buttons.
- **Manual Flywheel Control**: An input field and button to set a manual PWM value for the flywheel, useful for testing.

#### New Functions for Movement
```cpp
// --- MOTOR FUNCTIONS ---
void rearStop(){ ... }
void rearForward(){ ... }
void rearBackward(){ ... }

// --- STEERING ---
void steerLeft(){ steeringServo.write(SERVO_LEFT); }
void steerRight(){ steeringServo.write(SERVO_RIGHT); }
void steerCenter(){ steeringServo.write(SERVO_CENTER); }
```
- **Rear Motor**: Simple functions are created to control the rear drive motor by setting its driver pins HIGH or LOW.
- **Steering**: Functions now exist to command the servo to the predefined left, right, or center positions.

#### WebSocket Event Handler (`wsEvent`)
The WebSocket handler has been expanded to recognize and act on the new commands from the web UI, such as `"FWD"`, `"LEFT"`, `"STRAIGHT"`, etc., calling the appropriate motor or servo functions.

#### `setup()`
The `setup()` function is updated to:
1.  Initialize the pin modes for the new rear motor driver.
2.  "Attach" the servo object to its pin (`steeringServo.attach(SERVO_PIN)`).
3.  Set the steering to the center position on startup.

#### `loop()`
The main `loop()` function's balancing logic remains identical to the previous version. It continuously:
1.  Calculates the time delta (`dt`).
2.  Reads the IMU.
3.  Fuses the sensor data using the complementary filter.
4.  Calculates the required correction using the PD control law.
5.  Drives the flywheel motor if balancing is enabled and the bike is within the safe angle cutoff.

The new drive and steering commands are handled independently by the WebSocket event handler and do not interfere with the primary balancing loop.

---

## Part 2: The Physics of Balancing

The ability of this motorcycle to self-balance relies on several key physics principles, implemented through sensor fusion and control theory.

### 1. The Reaction Wheel: Conservation of Angular Momentum

This is the core principle that allows the robot to balance.

- **The Law**: The **Law of Conservation of Angular Momentum** states that for an isolated system, the total angular momentum remains constant. Angular momentum (`L`) is the rotational equivalent of linear momentum and is calculated as:
  
  `L = I * ω`
  
  Where:
  - `I` is the **Moment of Inertia**, a measure of an object's resistance to rotational acceleration (dependent on mass and its distribution around the axis of rotation). A heavy, large-diameter flywheel has a high moment of inertia.
  - `ω` (omega) is the **Angular Velocity** (how fast it's spinning).

- **How it's Used**: The robot and its flywheel form a (mostly) isolated system. When the robot starts to fall, the motor applies a **torque** (`τ`) to the flywheel, causing it to accelerate. According to Newton's Third Law for rotation, the flywheel exerts an equal and opposite torque on the robot's body.

  `τ_body = -τ_flywheel`

- **Example**:
  1. The motorcycle starts tilting to the **right**.
  2. The control system detects this tilt and commands the motor to accelerate the flywheel in a specific direction (e.g., clockwise).
  3. This action creates a **counter-clockwise reaction torque** on the motorcycle's frame.
  4. This reaction torque pushes the motorcycle's body back to the **left**, counteracting the fall and bringing it back to the vertical `targetAngle`.

### 2. Sensor Fusion: The Complementary Filter

The system needs a precise and stable measurement of its tilt angle. It gets this by "fusing" data from two different sensors.

- **The Problem**:
    - The **Accelerometer** can determine the angle of tilt by measuring the direction of the constant force of gravity. It's accurate for long-term, static angles but is very noisy and easily disturbed by vibration or movement.
    - The **Gyroscope** measures the rate of rotation (`ω`). It's very accurate for fast, short-term changes. However, it suffers from "drift," a small error that accumulates over time, making its long-term angle calculation unreliable.

- **The Solution**: The complementary filter combines the two signals, taking the best from both.

  **Formula**:
  `angle_new = α * (angle_prev + ω_gyro * dt) + (1 - α) * θ_accel`

  Where:
  - `angle_new`: The new, fused angle estimate.
  - `angle_prev`: The angle from the previous loop.
  - `ω_gyro * dt`: The change in angle since the last loop, as measured by the gyroscope. This is basic integration: `angle = velocity * time`. This part of the formula is a **high-pass filter**; it responds well to fast changes.
  - `θ_accel`: The angle calculated directly from the accelerometer. This part is a **low-pass filter**; it responds to slow, steady states.
  - `α`: The filter constant (e.g., `0.98`).

- **Example**: With `α = 0.98`:
  - **98%** of the new angle comes from the gyroscope's reading (`angle_prev + ω_gyro * dt`). This trusts the gyro for smooth, fast motion.
  - **2%** of the new angle comes from the accelerometer's reading (`θ_accel`). This small, continuous correction is enough to cancel out the gyroscope's drift over the long term, "pulling" the estimate back to the true angle provided by gravity.

### 3. Control Theory: The PD Controller

The PD (Proportional-Derivative) controller is the "brain" that decides how much power to send to the flywheel motor.

- **The Goal**: To reduce the `error` (the difference between the current `angle` and the `targetAngle`) to zero, quickly and without oscillating.

  **Formula**:
  `Output = (Kp * e) + (Kd * de/dt)`

  Where:
  - `e`: The current error (`angle - targetAngle`).
  - `de/dt`: The rate of change of the error (how fast the error is increasing or decreasing). In our code, this is `-gyroRate`.

- **Proportional (P) Term (`Kp * e`)**:
  - This provides a response that is proportional to the size of the error. A bigger tilt results in a bigger corrective force.
  - **Analogy**: This is like a spring. The further you stretch it (`error`), the harder it pulls back.
  - **Example**: If `Kp = 7.0` and the bike is tilted by 5 degrees, the P-term contributes `7.0 * 5 = 35` to the motor output. If it's tilted by 10 degrees, the P-term contributes `70`.

- **Derivative (D) Term (`Kd * de/dt`)**:
  - This provides a response based on how fast the bike is falling. It acts as a damper, resisting change and preventing the P-term from overshooting the target and oscillating.
  - **Analogy**: This is like a shock absorber in a car. It slows down the bouncing of the springs.
  - **Example**:
    1.  **Falling Fast**: The bike is falling to the right, so `gyroRate` is high and positive. The D-term (`Kd * -gyroRate`) becomes a large negative value, applying a strong force to slow the fall.
    2.  **Returning to Center**: The bike is now moving quickly back to vertical, so `gyroRate` is negative. The D-term (`Kd * -gyroRate`) is now *positive*, applying a force in the opposite direction to "brake" the bike as it approaches `targetAngle`, preventing it from overshooting and falling over on the other side.
