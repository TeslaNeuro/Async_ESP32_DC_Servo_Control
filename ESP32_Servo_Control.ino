#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define SERVOMIN  125 // Minimum pulse length count
#define SERVOMAX  575 // Maximum pulse length count

uint8_t servonum = 0;
int angle = 0;
bool increasing = true;
unsigned long previousMillis = 0;
const long interval = 500; // Interval in milliseconds

void moveServo() {
    pwm.setPWM(0, 0, angleToPulse(angle));
    pwm.setPWM(3, 0, angleToPulse(angle));
    Serial.print("Moving to angle: "); Serial.println(angle);
    
    if (increasing) {
        angle += 20;
        if (angle >= 180) {
            increasing = false;
        }
    } else {
        angle -= 20;
        if (angle <= 0) {
            increasing = true;
        }
    }
}

void setup() {
    Serial.begin(115200);
    Serial.println("16 channel Servo test!");

    pwm.begin();
    pwm.setPWMFreq(60);  // Analog servos run at ~60 Hz updates
}

void loop() {
    unsigned long currentMillis = millis();
    if (currentMillis - previousMillis >= interval) {
        previousMillis = currentMillis;
        moveServo();
    }
}

int angleToPulse(int ang) {
    int pulse = map(ang, 0, 180, SERVOMIN, SERVOMAX);
    Serial.print("Angle: "); Serial.print(ang);
    Serial.print(" pulse: "); Serial.println(pulse);
    return pulse;
}