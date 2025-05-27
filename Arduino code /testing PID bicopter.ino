#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <Servo.h>

MPU6050 mpu;
Servo motorLeft;
Servo motorRight;

bool dmpReady = false;
uint8_t fifoBuffer[64];
Quaternion q;
VectorFloat gravity;
float ypr[3];  

// PID Variables
float Kp = 3.0, Ki = 0.0, Kd = 1.5;
float previous_error = 0;
float integral = 0;

float setpoint = 0;
float offsetAngle = 0;
bool balanceActive = false;

// Time
unsigned long lastTime = 0;

// ESC signal values
int baseThrottle = 1100;  // neutral throttle (ESC idle)
int maxThrottle = 1700;

void setup() {
  Serial.begin(115200);
  Wire.begin();

  mpu.initialize();
  if (mpu.dmpInitialize() == 0) {
    mpu.setDMPEnabled(true);
    dmpReady = true;
    Serial.println("MPU6050 ready.");
  } else {
    Serial.println("MPU6050 init failed.");
  }

  motorLeft.attach(9);
  motorRight.attach(10);

  
  motorLeft.writeMicroseconds(baseThrottle);
  motorRight.writeMicroseconds(baseThrottle);
  delay(3000);  
  Serial.println("ESCs armed. Waiting for 'b' to start balancing.");
}

void loop() {
  if (Serial.available()) {
    char cmd = Serial.read();

    if (cmd == 'b') {
      balanceActive = true;
      setpoint = offsetAngle;
      Serial.println("Balance started.");
    } else if (cmd == 'x') {
      balanceActive = false;
      Serial.println("Balance stopped.");
      motorLeft.writeMicroseconds(1000);   
      motorRight.writeMicroseconds(1000);
    } else if (cmd == 'r') {
      offsetAngle += 10;
      setpoint = offsetAngle;
      Serial.print("Setpoint increased: ");
      Serial.println(setpoint);
    } else if (cmd == 'l') {
      offsetAngle -= 10;
      setpoint = offsetAngle;
      Serial.print("Setpoint decreased: ");
      Serial.println(setpoint);
    } else if (cmd == 's') {
      offsetAngle = 0;
      setpoint = 0;
      Serial.println("Setpoint reset to 0");
    } else if (cmd == 'z') {
      offsetAngle = 0;
      setpoint = 0;
      balanceActive = false;
      motorLeft.writeMicroseconds(1000);   
      motorRight.writeMicroseconds(1000);
      Serial.println("Safety Reset: All values to 0, balance OFF");
    }
  }

  if (dmpReady && mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

    float pitch = ypr[1] * 180 / PI;

    if (balanceActive) {
      float error = setpoint - pitch;
      unsigned long now = millis();
      float dt = (now - lastTime) / 1000.0;
      lastTime = now;

      integral += error * dt;
      float derivative = (error - previous_error) / dt;
      previous_error = error;

      float output = Kp * error + Ki * integral + Kd * derivative;

      int leftSpeed = constrain(baseThrottle + output, 1000, maxThrottle);
      int rightSpeed = constrain(baseThrottle - output, 1000, maxThrottle);

      motorLeft.writeMicroseconds(leftSpeed);
      motorRight.writeMicroseconds(rightSpeed);

      Serial.print("Pitch: ");
      Serial.print(pitch);
      Serial.print(" | Setpoint: ");
      Serial.print(setpoint);
      Serial.print(" | PID: ");
      Serial.print(output);
      Serial.print(" | L: ");
      Serial.print(leftSpeed);
      Serial.print(" | R: ");
      Serial.println(rightSpeed);
    }
  }
}
