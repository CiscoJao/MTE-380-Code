#include <Adafruit_PWMServoDriver.h>

#define SERVO1   0
#define SERVO2   4
#define SERVO3   8

static const int ARM_MAX = 200; // physical upper limit of the arms
static const int SERVO_MAX = 500; // CCW 180 deg
static const int SERVO_MIN = 100; // CW 0 deg
static const int SERVO_MID = (ARM_MAX + SERVO_MIN) / 2;
static const int PWM_FREQ = 50;
bool newCommand = false;
byte receivedAngles[3];

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

void setup() {
    Serial.begin(9600);

    pwm.begin();
    pwm.setPWMFreq(PWM_FREQ);

    // move all servos to zero position
    // pwm.setPWM(SERVO1, 0, SERVO_MID);
    // pwm.setPWM(SERVO2, 0, SERVO_MID);
    // pwm.setPWM(SERVO3, 0, SERVO_MID);
    // delay(1000);
}

void loop() {

    // read target angles from serial
    if (Serial.available() > 3) {
        Serial.readBytes(receivedAngles, 3);
        newCommand = true;
    }

    // Serial.println(receivedAngles);

    // // update target angles for the servos
    if (newCommand) {
        pwm.setPWM(SERVO1, 0, (int)receivedAngles[0]);
        pwm.setPWM(SERVO2, 0, (int)receivedAngles[1]);
        pwm.setPWM(SERVO3, 0, (int)receivedAngles[2]);
        newCommand = false;
    }

    // delay(10);
}