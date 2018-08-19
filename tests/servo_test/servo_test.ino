#include <Servo.h>


/** SERVO ##############################################################################################################
 *
 * #####################################################################################################################
 */
// Servo macros
#define SERVO_PIN               PA8

// Servo vars
const float G = 9.80665;
const int SERVO_MIN_DEG = 0;
const int SERVO_MAX_DEG = 180;
int pos = 90;                               // position in degrees

// Servo instance
Servo servo;


void cfgServo(void) {
    /* Configures the servo.
    */
    servo.attach(SERVO_PIN, 1100, 2200, 0, 180);
    servo.write(convert_output(0));
}


int convert_output(float output) {
    pos = ((output + G) * (SERVO_MAX_DEG - SERVO_MIN_DEG) / (2 * G)) + SERVO_MIN_DEG;
    return pos;
}


void setup(void) {
    cfgServo();
}


void loop(void) {
    servo.write(convert_output(0));

}