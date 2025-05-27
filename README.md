# Bi-Copter Pitch Stabilization using Arduino and PID

This project demonstrates a simplified drone-inspired **bi-copter balancing system**, designed as a practical application of concepts from a Control Systems course. It uses **two brushless motors** (with ESCs), a **GY-87 IMU** (MPU6050), and an **Arduino UNO** to achieve real-time pitch stabilization via a **PID controller**.

---

## 🔧 Features

- Real-time pitch angle reading from GY-87 IMU (MPU6050 DMP).
- PID control loop for dynamic balance using differential thrust.
- Serial interface to:
  - Start/stop the balancing system
  - Shift pitch setpoint by ±10° (left/right)
  - Reset the setpoint to vertical (0°)
  - Activate ESC safety stop (PWM = 1000 μs)
- ESC PWM output via pins 9 (left) and 10 (right)

---

## 📷 System Overview

- **IMU Sensor**: GY-87 (MPU6050)
- **Controller**: Arduino UNO
- **Actuators**: 2 Brushless Motors with ESCs
- **Control Algorithm**: PID (Proportional-Integral-Derivative)

---


