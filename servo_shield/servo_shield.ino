#include <Adafruit_PWMServoDriver.h>

#define SERVO1   0
#define SERVO2   4
#define SERVO3   8

static const int SERVO_MAX = 500; // CCW
static const int SERVO_MIN = 100; // CW
static const int SERVO_MID = (SERVO_MAX + SERVO_MIN) / 2;
static const int PWM_FREQ = 50;
int x = 0;

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();


void setup() {
    Serial.begin(9600);

    pwm.begin();
    pwm.setPWMFreq(PWM_FREQ);

    // move all servos to middle position
    pwm.setPWM(SERVO1, 0, SERVO_MID);
    pwm.setPWM(SERVO2, 0, SERVO_MID);
    pwm.setPWM(SERVO3, 0, SERVO_MID);
    delay(1000);
}

void loop() {
    // move all servos once
    pwm.setPWM(SERVO1, 0, SERVO_MID);
    pwm.setPWM(SERVO2, 0, SERVO_MID);
    pwm.setPWM(SERVO3, 0, SERVO_MID + x);
}
