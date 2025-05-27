# Real-Time Pitch Control on Dual-Motor UAV (Bi-Copter)

This project is a practical implementation of real-time pitch stabilization for a dual-motor drone-inspired system (bi-copter), developed as part of my Control Systems course.

It uses an Arduino UNO, a GY-87 IMU (MPU6050), and two brushless motors with ESCs to create a simple but effective platform for learning feedback control and UAV balance.

---

## 🚀 Features

- Real-time pitch angle measurement using MPU6050's DMP (Digital Motion Processor)
- PID controller to maintain pitch balance by adjusting motor thrust
- Serial commands to:
  - Start/Stop the balancing process
  - Adjust the setpoint by ±10° (simulate forward/backward tilt)
  - Reset the setpoint to 0°
  - Safely disable ESCs (PWM = 1000 μs)

---

## 🛠️ Hardware Components

| Component           | Description                        |
|---------------------|------------------------------------|
| Microcontroller     | Arduino UNO                        |
| IMU Sensor          | GY-87 (with MPU6050)               |
| Motors              | 2 Brushless Motors with ESCs       |
| Output Pins         | ESCs connected to pins 9 and 10    |
| Input               | Serial Monitor for user commands   |

---

## 🔁 Control Logic

1. Read pitch angle from the IMU (MPU6050 via DMP).
2. Apply PID to compute correction based on the angle error.
3. Use differential PWM signals to adjust motor thrust accordingly.
4. Use serial commands to tune and control behavior in real time.

---

## 🖥️ Serial Commands

| Command | Function                                  |
|---------|-------------------------------------------|
| `b`     | Begin PID balancing                       |
| `s`     | Reset pitch setpoint to 0°                |
| `r`     | Increase setpoint by +10° (tilt forward)  |
| `l`     | Decrease setpoint by -10° (tilt backward) |
| `x`     | ESC safety stop (PWM output = 1000 μs)    |

---

##  Demonstration

![overview](https://github.com/user-attachments/assets/97da021b-ad3a-4481-a2ef-632545a71af7)

---

## Learning Outcomes

- Practical application of PID control
- Real-time sensor-actuator interfacing
- UAV balancing concepts using differential thrust
- Embedded systems debugging and tuning

---

##  Future Work

- Add full 6DOF control (roll, yaw, altitude)
- Integration with full drone flight controller logic

---

## 👨‍💻 Author

**Mohamed Abdelsabour Radwan**  
Mechatronics Engineering Student @ E-JUST  
GitHub: [@mohammedabdelsaboor](https://github.com/mohammedabdelsaboor)  
LinkedIn: [LinkedIn Profile](https://www.linkedin.com/in/mohammed-a-1b519321a/)

---

## 📄 License

This project is open-source and intended for educational and experimental use.
