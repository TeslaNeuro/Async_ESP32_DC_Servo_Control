#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define SERVOMIN  125  // Minimum pulse length count
#define SERVOMAX  575  // Maximum pulse length count
#define NUM_SERVOS  6  // Number of servos

struct ServoData {
    int currentAngle;
    int targetAngle;
    int stepSize;  // Speed control: smaller = slower, larger = faster
    unsigned long lastUpdate;
    unsigned long updateInterval;  // Time interval per step
};

ServoData servos[NUM_SERVOS] = {
    {0, 90, 2, 0, 20},
    {0, 45, 3, 0, 15},
    {0, 60, 1, 0, 25},
    {0, 120, 4, 0, 10},
    {0, 30, 2, 0, 20},
    {0, 150, 3, 0, 15}
};

void setup() {
    Serial.begin(115200);
    pwm.begin();
    pwm.setPWMFreq(60);
}

void loop() {
    handleSerialInput();  // Check for user input

    unsigned long currentMillis = millis();
    
    for (int i = 0; i < NUM_SERVOS; i++) {
        if (currentMillis - servos[i].lastUpdate >= servos[i].updateInterval) {
            servos[i].lastUpdate = currentMillis;
            updateServo(i);
        }
    }
}

void updateServo(int servoIndex) {
    ServoData &servo = servos[servoIndex];

    if (servo.currentAngle != servo.targetAngle) {
        if (servo.currentAngle < servo.targetAngle) {
            servo.currentAngle += servo.stepSize;
            if (servo.currentAngle > servo.targetAngle) servo.currentAngle = servo.targetAngle;
        } else {
            servo.currentAngle -= servo.stepSize;
            if (servo.currentAngle < servo.targetAngle) servo.currentAngle = servo.targetAngle;
        }

        pwm.setPWM(servoIndex, 0, angleToPulse(servo.currentAngle));
        Serial.print("Servo "); Serial.print(servoIndex);
        Serial.print(" -> Angle: "); Serial.print(servo.currentAngle);
        Serial.print(" | Speed: "); Serial.println(servo.stepSize);
    }
}

int angleToPulse(int ang) {
    return map(ang, 0, 180, SERVOMIN, SERVOMAX);
}

void handleSerialInput() {
    if (Serial.available()) {
        String input = Serial.readStringUntil('\n');  // Read full line
        input.trim();  // Remove unwanted spaces/newlines

        int servoIndex = -1, angle = -1, speed = -1;

        if (sscanf(input.c_str(), "S%d A%d V%d", &servoIndex, &angle, &speed) == 3) {
            if (servoIndex >= 0 && servoIndex < NUM_SERVOS && angle >= 0 && angle <= 180 && speed > 0) {
                servos[servoIndex].targetAngle = angle;
                servos[servoIndex].stepSize = speed;
                Serial.print("Command Received -> Servo: "); Serial.print(servoIndex);
                Serial.print(" | Angle: "); Serial.print(angle);
                Serial.print(" | Speed: "); Serial.println(speed);
            } else {
                Serial.println("Invalid values! Format: S<servo> A<angle> V<speed>");
            }
        } else {
            Serial.println("Invalid command! Use format: S<servo> A<angle> V<speed>");
        }
    }
}
