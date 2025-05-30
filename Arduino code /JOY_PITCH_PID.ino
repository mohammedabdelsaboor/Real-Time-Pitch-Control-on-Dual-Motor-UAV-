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

float Kp = 3.0, Ki = 0.0, Kd = 2;
float previous_error = 0;
float integral = 0;

float setpoint = 0;
float offsetAngle = 0;
bool balanceActive = false;

unsigned long lastTime = 0;

int baseThrottle = 1120;
int maxThrottle = 1700;

const int VRxPin = A1;
const int SWPin = 4;
int joystickCenter = 512;
int joystickDeadzone = 30;
float maxJoystickAngle = 50.0;

bool lastSWState = HIGH;

void setup() {
  Serial.begin(115200);
  Wire.begin();

  pinMode(SWPin, INPUT_PULLUP);

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

  Serial.println("ESCs armed. Press joystick button (SW) to toggle balance.");
}

void loop() {
  int currentSWState = digitalRead(SWPin);

  if (lastSWState == HIGH && currentSWState == LOW) {
    balanceActive = !balanceActive;
    if (balanceActive) {
      Serial.println("Balance ON");
      offsetAngle = 0;    
      integral = 0;       
      previous_error = 0;
    } else {
      Serial.println("Balance OFF");
      motorLeft.writeMicroseconds(1000);
      motorRight.writeMicroseconds(1000);
    }
    delay(200); 
  }
  lastSWState = currentSWState;

  if (dmpReady && mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

    float pitch = ypr[1] * 180 / PI;

    int joyX = analogRead(VRxPin);
    if (abs(joyX - joystickCenter) > joystickDeadzone) {
      offsetAngle = ((float)joyX - joystickCenter) * (maxJoystickAngle / joystickCenter);
    } else {
      offsetAngle = 0;
    }

    setpoint = offsetAngle;

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
