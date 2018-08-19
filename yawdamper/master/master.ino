/******************************************************************************
 * The MIT License
 *
 * Copyright (c) 2010 Perry Hung.
 * Copyright (c) 2012 LeafLabs, LLC.
 *
 * Permission is hereby granted, free of charge, to any person
 * obtaining a copy of this software and associated documentation
 * files (the "Software"), to deal in the Software without
 * restriction, including without limitation the rights to use, copy,
 * modify, merge, publish, distribute, sublicense, and/or sell copies
 * of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS
 * BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
 * ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *****************************************************************************/

/**
 * @file yawdamper/yawdamper.ino
 * @author Fernando Canteruccio <tau@megali.com>
 * @brief Yawdamper device module.
 *
 */

/**
 * Tau, 15/06/2018
 * Yawdamper master module
 *
 * TODO:
 *      UI
 *
 * DESCRIPTION:
 *
 * This project uses a stm32f103c8, mpu6050 and ssd1306 as a yawdamper module for
 * light experimental aircraft applications.
 *
 * ## PINMAP ###########################################################################################################
 *
 *  OLED TODO
 *      PA4  -> RES
 *      PA5  -> SCK
 *      PA6  -> CS
 *      PA7  -> SDA
 *      PB0  -> DC
 *
 *  BUTTONS
 *      PB13 -> MODE
 *      PB14 -> PLUS
 *      PB15 -> MINUS
 *
 *  SENSOR
 *      PB12 -> INT
 *      PB7  -> SDA
 *      PB6  -> SCL
 *      GND  -> GND
 *      3.3  -> VCC
 *
 *  SERVO
 *      PA8  -> PWM
 *
 *  CONTROLLER OUTPUT CONNECTOR
 *      PIN1  ->
 *      PIN2  ->
 *      PIN3  ->
 *      PIN4  ->
 *      PIN5  -> SERVO GND
 *      PIN6  ->
 *      PIN7  ->
 *      PIN8  ->
 *      PIN9  -> GND
 *      PIN10 -> PA8 -> PWM
 *      PIN11 ->
 *      PIN12 ->
 *      PIN13 ->
 *      PIN14 ->
 *      PIN15 -> 12V
 *
 *  SERVO CONNECTOR
 *      PIN1 -> VCC
 *      PIN3 -> PWM
 *      PIN9 -> GND
 *
 */

/*
 * PID coef
 *
 * KP = 2.7
 * KI = 2.4
 * KD = 1.2
 *
 * ALPHA = 30
 */

#include <Arduino.h>
#include <Adafruit_SSD1306.h>
#include <EEPROM.h>
#include <MPU6050.h>
#include <PID_v1.h>
#include <Filters.h>
#include <Servo.h>
#include <Wire.h>
#include <libmaple/iwdg.h>
#include <HardwareTimer.h>

#include <libmaple/i2c.h>
#include <libmaple/systick.h>

#include <libmaple/rcc.h>
#include <libmaple/util.h>
#include <libmaple/scb.h>


/**
 * #####################################################################################################################
 * ###                                                                                                               ###
 * ###                                              MEMORY MANAGEMENT                                                ###
 * ###                                                                                                               ###
 * #####################################################################################################################
 */

#define DEBUG
#define PROBE_PIN               PB12
#define DEBUG_LEVEL DEBUG_NONE
//#define I2C_DEBUG

/** Time ###############################################################################################################
 *
 * #####################################################################################################################
 */
#ifdef DEBUG
unsigned int loopFreq;
#endif
const unsigned long loopTime = 1999;
volatile unsigned int i = 1;

// Watchdog
#define IWDG_NUM                210 //210
#define IWDG_PRESCALER          IWDG_PRE_256


/** SENSOR #############################################################################################################
 *
 * #####################################################################################################################
 */
// Sensor macros
// #define SENSOR_SCL           PB6                  // m16
// #define SENSOR_SDA           PB7                  // m15
#define SENSOR_VCC              PB4
//#define INT_PIN
#define SDA                     PB7
#define SCL                     PB6
#define DLPF                    MPU6050_DLPF_6       // 5hz - 19ms delay

#define SENSOR_IWDG             13000
#define T                       Timer2
#define c                       2


// Sensor vars
const unsigned int SENSOR_MOD = 2;
const unsigned int CALIBRATION_SAMPLES = 10000;                // calibration mean n of samples
const float G = 9.80665;                             // gravity as written in the sensor header

float pitch = 0.0;                            // pitch measurement
float roll = 0.0;                             // roll measurement
float yaw = 0.0;                              // yaw measurement
float offsetPitch = 0.0;                          // offset after calibration
float offsetRoll = 0.0;                           // offset after calibration
float offsetYaw = 0.0;                            // offset after calibration
//
//float offsetGx;                             // offset after calibration
//float offsetGy;                             // offset after calibration
//float offsetGz;                             // offset after calibration

float gyroX = 0.0;
float gyroY = 0.0;
float gyroZ = 0.0;
float gyroThreshold = 0.0;

float alpha = 1.0;                          // exponential filter decay
float *usedAxis;                            // pointer to used axis
float *usedGAxis;                            // pointer to used axis

int axis = 0;                               // axis selection helper variable
int sensorReverse = 1;

volatile bool canRead = true;

// Sensor instances
Vector normA;                                        // Accelerometer vector
Vector normG;                                        // Gyro vector
MPU6050 mpu;

// Input filters
FilterOnePole accelPole1(LOWPASS, alpha * 2);
FilterOnePole accelPole2(LOWPASS, alpha * 3);
FilterOnePole accelPole3(LOWPASS, alpha * 4);
FilterOnePole gyroPole1(LOWPASS, alpha * 2);
FilterOnePole gyroPole2(LOWPASS, alpha * 3);
FilterOnePole gyroPole3(LOWPASS, alpha * 4);


/** PID ################################################################################################################
 *
 * #####################################################################################################################
 */

// Time step vars
const double PID_FREQ = 250; //166.6666;
const double TIME_STEP = 1 / PID_FREQ;
const double ACCEL_MULTIPLIER = 10;
const double GYRO_MULTIPLIER = 50;
const double KD_MULTIPLIER = 0.3;

double gyroDot = 0.0;
double input = 0.0;
double output = 0.0;
double setpoint = 0.0;
double gain = 0.0;
double gainG = 0.0;
double gyroT = 0.0;
double KP = 0.0;                            // proportional
double KI = 0.0;                            // integral
double KD = 0.0;                            // derivative
double sensitivity = 0.0;
double nl = 0.0;

int pidMode = 0;                            // 0 -> DIRECT, 1 -> REVERSE

bool pidOn = false;                         // true if in automatic mode, false if in manual

// PID instance
PID pid(&input, &output, &setpoint, gain * KP, gain * KI, gain * KD_MULTIPLIER * KD * (1 + sensitivity), pidMode);


/** DISPLAY ############################################################################################################
 *
 * #####################################################################################################################
 */
// Display macros
#define OLED_DC                 PB0  // 7
#define OLED_CS                 SS  // 5
#define OLED_RESET              MISO  // 8

// Display vars
const int DISPLAY_MOD = 3;
long time;

int j = 0; // activity counter
const int MSG_DELAY = 20;
int puMsg = 0;
int e1 = 0;
int e2 = 0;
int e3 = 0;
int e4 = 0;
int e5 = 0;
int e6 = 0;
int pidCalib = 0;

// Display instance
Adafruit_SSD1306 display(OLED_DC, OLED_RESET, OLED_CS);


/** SERVO ##############################################################################################################
 *
 * #####################################################################################################################
 */
// Servo macros
#define SERVO_PIN               PA8

// Servo vars
const int SERVO_MOD = 3;
const int SERVO_MIN_DEG = 0;
const int SERVO_MAX_DEG = 180;
int pos = 90;                               // position in degrees
float trimValue = 0;                        // value of output when pid is on manual mode

// Servo instance
Servo servo;


/** INTERFACE ##########################################################################################################
 *
 * #####################################################################################################################
 */
// Buttons macros
#define ON_OFF_PIN              PB13
#define PLUS_PIN                PB14
#define MINUS_PIN               PB15

// Buttons vars
const int B_MOD = 5;
bool onOff;
bool pidOnOff;
bool minusB;
bool plusB;
bool prevOnOff;
bool prevPidOnOff;
bool prevCalibration;
bool prevB;
bool prevPB;
unsigned long pressTime;
unsigned long pidPressTime;
unsigned long releaseTime = -1500; // initialized as -1500 so that printTuning is not called at the first loops
const unsigned long calibrationTime = 1000;
const unsigned long debounceDelay = 300;
bool powerUser = false;

/** EEPROM #############################################################################################################
 *
 * #####################################################################################################################
 */
// EEPROM macros.
#define GYROT_ADDRESS           0x1F
#define SENS_ADDRESS            0x1E
#define NL_ADDRESS              0x1D
#define GAING_ADDRESS           0x1C
#define AXIS_ADDRESS            0x1B
#define SENSOR_REVERSE_ADDRESS  0x1A
#define ALPHA_ADDRESS           0x19
#define PID_MODE_ADDRESS        0x18
#define KP_ADDRESS              0x17
#define KI_ADDRESS              0x16
#define KD_ADDRESS              0x15
#define GAIN_ADDRESS            0x14
#define TRIM_ADDRESS            0x13
#define OFFSET_PITCH_ADDRESS    0x12
#define OFFSET_ROLL_ADDRESS     0x11
#define OFFSET_YAW_ADDRESS      0x10

// EEPORM instances
uint16 dataWrite;
uint16 dataRead;
uint16 eepromStatus;


/** END MEMORY MANAGEMENT ##############################################################################################
 */


/**
 * #####################################################################################################################
 * ###                                                                                                               ###
 * ###                                                  PROCEDURES                                                   ###
 * ###                                                                                                               ###
 * #####################################################################################################################
 */

// FAULT HANDLING ######################################################################################################
#ifdef DEBUG
#define LED_PIN                 PC13

void cfgProbes(void) {
    pinMode(PROBE_PIN, OUTPUT);
    digitalWrite(PROBE_PIN, LOW);

}


void flipP1(void) {
    digitalWrite(PROBE_PIN, !digitalRead(PROBE_PIN));
}
#endif

// SENSOR PROCEDURES ###################################################################################################
void readSensor(void) {
    /**
     * Reads values from the sensor registers
     */

    // Read sensor
    normA = mpu.readNormalizeAccel();
    normG = mpu.readNormalizeGyro();

    // Apply exponential filtering to chosen axis
    switch (axis) {
        case 0:
            pitch = sensorReverse * (
                    accelPole3.input(
                    accelPole2.input(
                    accelPole1.input(
                            normA.YAxis
                    ))) - offsetPitch);
            gyroZ = sensorReverse * gyroPole3.input(gyroPole2.input(gyroPole1.input(normG.ZAxis)));
            break;
        case 1:
            roll = sensorReverse * (
                    accelPole3.input(
                    accelPole2.input(
                    accelPole1.input(
                            normA.XAxis
                    ))) - offsetRoll);
            gyroZ = sensorReverse * gyroPole3.input(gyroPole2.input(gyroPole1.input(normG.ZAxis)));
            break;
        case 2:
            yaw = sensorReverse * (
                    accelPole3.input(
                    accelPole2.input(
                    accelPole1.input(
                            normA.ZAxis
                    ))) - offsetYaw);
            gyroX = sensorReverse * gyroPole3.input(gyroPole2.input(gyroPole1.input(normG.XAxis)));
            break;
        default:
            break;
    }
}


void calibrateAccelerometer(void) {
    /* Reads a "calibrationSamples" amount of values from the sensor
     saving the mean to the offset variables.
    */

    iwdg_feed();
    feedSensorWtdg();

    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.setCursor(16, 12);
    display.println("Calibrando Sensor");
    display.display();

    Vector normC;

    offsetPitch = 0;
    offsetRoll = 0;
    offsetYaw = 0;

    double offset1 = 0;
    double offset2 = 0;
    double offset3 = 0;

    int k = 0;
    while (k <= CALIBRATION_SAMPLES) {
        iwdg_feed();
        feedSensorWtdg();
        delay(1);
        k++;
        normC = mpu.readNormalizeAccel();

        offset1 += normC.YAxis;
        offset2 += normC.XAxis;
        offset3 += normC.ZAxis;

        if (k % 10 == 0) {
            display.drawRect(16, 24, 97, 20, WHITE);
            display.fillRect(16, 24, (1 + (k / double(CALIBRATION_SAMPLES)) * 96), 20, WHITE);
            display.display();
        }
    }

    offsetPitch = offset1 / CALIBRATION_SAMPLES;
    offsetRoll = offset2 / CALIBRATION_SAMPLES;
    offsetYaw = offset3 / CALIBRATION_SAMPLES;

    writeEEPROM(OFFSET_PITCH_ADDRESS, offsetPitch);
    writeEEPROM(OFFSET_ROLL_ADDRESS, offsetRoll);
    writeEEPROM(OFFSET_YAW_ADDRESS, offsetYaw);
}


void recoverSensor(void) {
    // Power cycle MPU for fresh start
    digitalWrite(SENSOR_VCC, LOW);

    i2c_disable(I2C1);
    i2c_master_enable(I2C1, I2C_BUS_RESET);

    // Turn sensor off and wait for 100ms while feeding the watchdogs
    for (int i = 0; i < 100; i++) {
        feedSensorWtdg();
        iwdg_feed();
        delay(1);
    }

    digitalWrite(SENSOR_VCC, HIGH);

    for (int i = 0; i < 100; i++) {
        feedSensorWtdg();
        iwdg_feed();
        delay(1);
    }

    mpu.writeRegister8(0x24, 0b00001001);                                   // 400khz clock

    // Initialize MPU6050
    while(!mpu.recover(MPU6050_SCALE_250DPS, MPU6050_RANGE_2G)) {
        feedSensorWtdg();
        iwdg_feed();
        // While not initialized, display error msg
        display.clearDisplay();
        display.setTextSize(1);
        display.setTextColor(WHITE);
        display.setCursor(12, 25);
        display.println("FALHA NO SENSOR");
        display.display();

    };

    // Configure mpu
    mpu.setDLPFMode(DLPF);                                                  // Set low pass filter band
    mpu.setTempEnabled(false);                                              // disable temperature sensor
//    mpu.setAccelPowerOnDelay(MPU6050_DELAY_1MS);                          // delay start for compatibility issues
    mpu.writeRegister8(0x23, 0b00000000);                                   // Disable FIFO queues

    mpu.setThreshold(gyroT / 10);

    for (int i = 0; i < 10; i++) {
        feedSensorWtdg();
        iwdg_feed();
        delay(1);
    }

    // update fail counter
//    writeEEPROM(FAIL_ADDRESS, failCount++);
}


void cfgSensor(void) {
    /**
     * Configures MPU6050.
     */

    // Feed watchdog
    iwdg_feed();

    // Configure controller VCC pin as output
    pinMode(SENSOR_VCC, OUTPUT);
    digitalWrite(SENSOR_VCC, HIGH);

    // Initialize MPU6050
    while(!mpu.begin(MPU6050_SCALE_250DPS, MPU6050_RANGE_2G)) {
        iwdg_feed();
        // While not initialized, display error msg
        display.clearDisplay();
        display.setTextSize(1);
        display.setTextColor(WHITE);
        display.setCursor(12, 25);
        display.println("FALHA NO SENSOR");
        display.display();
    };

    // Configure mpu
    mpu.writeRegister8(0x24, 0b00001001);                                   // 400khz clock
    mpu.writeRegister8(MPU6050_REG_GYRO_CONFIG, 0b00000000);                // Gyro self test disable
    mpu.setDLPFMode(DLPF);                                                  // Set low pass filter band
    mpu.setTempEnabled(false);                                              // disable temperature sensor
    mpu.writeRegister8(0x23, 0b00000000);                                   // Disable FIFO queues
//    mpu.setAccelPowerOnDelay(MPU6050_DELAY_1MS);                          // delay start for compatibility issues
//    mpu.writeRegister8(0x37, 0b00010000);                                 // Interruption pin config
//    mpu.writeRegister8(0x38, 0b00000001);                                 // Interruption config

    // Read offsets from EEPROM
    offsetPitch = readEEPROM(OFFSET_PITCH_ADDRESS);
    offsetRoll = readEEPROM(OFFSET_ROLL_ADDRESS);
    offsetYaw = readEEPROM(OFFSET_YAW_ADDRESS);
    // Read used axis from EEPROM
    axis = readEEPROM(AXIS_ADDRESS);
    // Read all the other configs
    gyroT = readEEPROM(GYROT_ADDRESS);


    // Update pointer to used axis
    if (axis == 0) {
        usedAxis = &pitch;
        usedGAxis = &gyroZ;
    } else if (axis == 1) {
        usedAxis = &roll;
        usedGAxis = &gyroZ;
    } else if (axis == 2) {
        usedAxis = &yaw;
        usedGAxis = &gyroX;
    }

    // Feed watchdog
    iwdg_feed();

    displayMsg("Calibrando Gyro...");
    mpu.setThreshold(gyroT / 10);
    mpu.calibrateGyro(200);
}


// TIMER PROCEDURES ####################################################################################################
void resetSensor(void) {
    canRead = false;
}


void feedSensorWtdg(void) {
    T.refresh();
}


void cfgSensorWtdg(void) {
    T.setPeriod(SENSOR_IWDG);
    T.attachInterrupt(c, resetSensor);
}


// PID procedures ######################################################################################################
static inline float sgn(float val) {
    if (val < 0) return -1;
    if (val==0) return 0;
    return 1;
}


void computePID(void) {
    // Calculates the output of the PID
    input = ACCEL_MULTIPLIER * (sgn(*usedAxis) * pow(abs(*usedAxis), 1 + nl));

    if (pid.Compute()) {
        // Add gyro differential control
        output = constrain(output - gain * gainG * GYRO_MULTIPLIER * (*usedGAxis - gyroDot), -G, G);

        // save data for the next loop
        gyroDot = *usedGAxis;
//        prevOutput = output;

        // Converts the output to a value in degree and drive servo
        servo.write(convert_output(output));
    }
}


void cfgPID(void) {
    /* Configures the PID.
    */
    setpoint = 0;
    pid.SetOutputLimits(-G, G);
    pid.SetSampleTime(1000 * TIME_STEP);

    sensorReverse = readEEPROM(SENSOR_REVERSE_ADDRESS);
    alpha = readEEPROM(ALPHA_ADDRESS);
    trimValue = readEEPROM(TRIM_ADDRESS);
    gain = readEEPROM(GAIN_ADDRESS);
    KP = readEEPROM(KP_ADDRESS);
    KI = readEEPROM(KI_ADDRESS);
    KD = readEEPROM(KD_ADDRESS);
    pidMode = readEEPROM(PID_MODE_ADDRESS);
    gainG = readEEPROM(GAING_ADDRESS);
    nl = readEEPROM(NL_ADDRESS);
    sensitivity = readEEPROM(SENS_ADDRESS);
    output = trimValue;

    accelPole1.setFrequency(alpha * 2);
    accelPole2.setFrequency(alpha * 3);
    accelPole3.setFrequency(alpha * 4);
    gyroPole1.setFrequency(alpha * 2);
    gyroPole2.setFrequency(alpha * 3);
    gyroPole3.setFrequency(alpha * 4);

    pid.SetControllerDirection(pidMode);
    pid.SetTunings(gain * KP, gain * KI, gain * KD_MULTIPLIER * KD * (1 + sensitivity));
}


// SERVO PROCEDURES ####################################################################################################
int convert_output(float output) {
    pos = ((output + G) * (SERVO_MAX_DEG - SERVO_MIN_DEG) / (2 * G)) + SERVO_MIN_DEG;
    return pos;
}


void cfgServo(void) {
    /* Configures the servo.
    */
    servo.attach(SERVO_PIN, 1100, 2200, 0, 180);
    servo.write(convert_output(trimValue));
}


// DISPLAY PROCEDURES ##################################################################################################
void refreshScreen(void) {

    if (pidCalib) {
        printPidTuning();
    } else if (pidOn && !puMsg && (plusB || minusB || (millis() - releaseTime) < 500)) {
        printBar("Gain:", 38, gain);
    } else if (e1 || e2 || e3) {
        printError();
    } else if (puMsg) {
        printPowerUser();
        puMsg--;
    } else {
        printControl();
    };
}


void printControl(void) {
    /* Prints the display graphics
    */

    int xOffset = 8;
    int yOffset = 2;

    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(WHITE);

    // ON-OFF sign
    if (pidOn) {
        display.setCursor(5 + xOffset, 4 + yOffset);
        display.println("ON");
    } else {
        display.setCursor(106 - xOffset, 4 + yOffset);
        display.println("OFF");
    };

    // Activity viewer
    if (j == 0) {
        display.setCursor(20 + xOffset, 4 + yOffset);
        display.println("|");
        j++;
    } else if (j == 1) {
        display.setCursor(20 + xOffset, 4 + yOffset);
        display.println("/");
        j++;
    } else if (j == 2) {
        display.setCursor(20 + xOffset, 4 + yOffset);
        display.println("-");
        j++;
    } else {
        display.setCursor(20 + xOffset, 4 + yOffset);
        display.println("\\");
        j = 0;
    }

    // Time counter

#ifdef DEBUG
    display.setCursor(28 + xOffset, 4 + yOffset);
    display.print("Freq");
    display.setCursor(54 + xOffset, 4 + yOffset);
    display.print(loopFreq);
//    display.print(input);
#else
    display.setCursor(28 + xOffset, 4 + yOffset);
    display.print("TON");
    display.setCursor(48 + xOffset, 4 + yOffset);
    display.println(systick_uptime() / 60000);
#endif

    // Ball
    display.fillCircle(constrain(int(56.5 + *usedAxis * (400 / G)) + xOffset, 18 + xOffset, 94 + xOffset), 26 + yOffset, 8, WHITE);

    // Gyro
    display.fillRect(constrain(int(55.5 - *usedGAxis * (96 / (2 * G))) + xOffset, 9 + xOffset, 101 + xOffset), 42 + yOffset, 3, 3, WHITE);

    // Yaw position
    display.fillRect(int(55.5 - output * (92 / (2 * G))) + xOffset, 46 + yOffset, 3, 4, WHITE);

    // Markers
    if (pidOn) {
        display.fillRect(5 + xOffset, 17 + yOffset, 4, 19, WHITE);
        display.fillRect(104 + xOffset, 17 + yOffset, 4, 19, WHITE);
        display.fillRect(5 + xOffset, 40 + yOffset, 4, 12, WHITE);
        display.fillRect(104 + xOffset, 40 + yOffset, 4, 12, WHITE);
    }

    display.fillRect(45 + xOffset, 17 + yOffset, 2, 19, WHITE);
    display.fillRect(66 + xOffset, 17 + yOffset, 2, 19, WHITE);
    display.fillRect(55 + xOffset, 40 + yOffset, 3, 2, WHITE);
    display.fillRect(55 + xOffset, 50 + yOffset, 3, 2, WHITE);

    // Refs
    display.fillRect(66 + xOffset, 50 + yOffset, 1, 2, WHITE);
    display.fillRect(46 + xOffset, 50 + yOffset, 1, 2, WHITE);
    display.fillRect(36 + xOffset, 50 + yOffset, 1, 2, WHITE);
    display.fillRect(76 + xOffset, 50 + yOffset, 1, 2, WHITE);
    display.fillRect(26 + xOffset, 50 + yOffset, 1, 2, WHITE);
    display.fillRect(86 + xOffset, 50 + yOffset, 1, 2, WHITE);
    display.fillRect(16 + xOffset, 50 + yOffset, 1, 2, WHITE);
    display.fillRect(96 + xOffset, 50 + yOffset, 1, 2, WHITE);

    // Print
    display.display();
}


void printBar(String name, int offset, float value) {
    /* Prints the tuning on screen
    when there is a gain adjustment
    */
    display.clearDisplay();
    display.setTextSize(2);
    display.setTextColor(WHITE);
    display.setCursor(offset, 8);
    display.print(name);

    display.setTextSize(1);
    display.setCursor(14, 48);
    display.print("0");
    display.setCursor(112, 48);
    display.print("1");
    display.setCursor(53, 48);
    display.print(value);

    display.writeFastVLine(64, 40, 3, WHITE);

    display.drawRect(16, 24, 97, 20, WHITE);
    display.fillRect(16, 24, (1 + value * 96), 20, WHITE);
    display.display();
}


void printPidTuning(void) {
    if (powerUser) {
        display.clearDisplay();
        display.setTextSize(2);
        display.setTextColor(WHITE);
        if (pidCalib == 1) {
            printBar("CutOff:", 28, alpha);
        } else if (pidCalib == 2) {
            printBar("KP:", 52, KP);
        } else if (pidCalib == 3) {
            printBar("KD:", 52, KD);
        } else if (pidCalib == 4) {
            printBar("KI:", 52, KI);
        } else if (pidCalib == 5) {
            printBar("Power:", 28, nl);
        } else if (pidCalib == 6) {
            printBar("Gr gain:", 20, gainG);
        } else if (pidCalib == 7) {
            printBar("Gr th:", 32, gyroT);
        } else if (pidCalib == 8) {
            display.setCursor(28, 12);
            display.println("Input:");
            if (sensorReverse == 1) {
                display.setCursor(24, 32);
                display.println("Direct");
            } else if (sensorReverse == -1) {
                display.setCursor(20, 32);
                display.println("Reverse");
            }
            display.display();
        } else if (pidCalib == 9) {
            display.setCursor(20, 12);
            display.println("Output:");
            if (pidMode == 0) {
                display.setCursor(24, 32);
                display.println("Direct");
            } else if (pidMode == 1) {
                display.setCursor(20, 32);
                display.println("Reverse");
            }
            display.display();
        } else if (pidCalib == 10) {
            display.setCursor(36, 12);
            display.println("Axis:");
            display.setCursor(60, 32);
            display.println(axis);
            display.display();
        }

    } else {
        display.clearDisplay();
        if (pidCalib == 1) {
            printBar("Sensib:", 22, sensitivity);
        }
    }
}


void printPowerUser(void) {
    display.clearDisplay();
    display.setTextSize(2);
    display.setTextColor(WHITE);
    display.setCursor(10, 12);
    display.println("PowerUser");
    if (powerUser == true) {
        display.setCursor(50, 32);
        display.println("ON");
    } else if (powerUser == false) {
        display.setCursor(44, 32);
        display.println("OFF");
    }
    display.display();
}


void printError(void) {

    if (e1 || e2 || e3 || e4 || e5 || e6) {
        display.clearDisplay();
        display.setTextSize(1);
        display.setTextColor(WHITE);
        display.setCursor(12, 4);
        display.println("FALHA NA MEMORIA");
    }
    if (e1 > 0) {
        display.setCursor(14, 35);
        display.println("ESCRITA GANHO");
        display.display();
        e1--;
    } else if (e2 > 0) {
        display.setCursor(14, 35);
        display.println("ESCRITA TRIM");
        display.display();
        e2--;
    } else if (e3 > 0) {
        display.setCursor(14, 35);
        display.println("ESCRITA OFFSET");
        display.display();
        e3--;
    } else if (e4 > 0) {
        display.setCursor(15, 35);
        display.println("GANHO RESETADO");
        display.display();
        e4--;
    } else if (e5 > 0) {
        display.setCursor(20, 35);
        display.println("TRIM RESETADO");
        display.display();
        e5--;
    } else if (e6 > 0) {
        display.setCursor(15, 35);
        display.println("CALIBRAGEM RESETADA");
        display.display();
        e6--;
    }
}


void displayMsg(String msg){
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.setCursor(12, 25);
    display.println(msg);
    display.display();
}


void cfgDisplay(void) {
    display.begin(SSD1306_SWITCHCAPVCC);

    display.clearDisplay();
}


// INTERFACE PROCEDURES ################################################################################################
void processInterface(void) {
    readB();
    readOnOff();
}


void readB(void) {
    /* Reads the Plus button, witch increases the variable gain if
     PID is on automatic mode and decreases the output if PID
     is on manual mode.
    */
    plusB = digitalRead(PLUS_PIN);
    minusB = digitalRead(MINUS_PIN);

    if (plusB && !minusB) {
        updateAdjusts(+1);

    } else if (minusB && !plusB) {
        updateAdjusts(-1);

    } else if (minusB && plusB) {
        switchPowerUser();

    } else {
        if(prevB == true) {
            releaseTime = millis();
            prevB = false;
            prevPB = false;
            if (pidOn) {
                writeEEPROM(GAIN_ADDRESS, gain);
            } else {
                writeEEPROM(TRIM_ADDRESS, trimValue);
            }
        };
    };
}


void switchPowerUser(void) {
    if (prevPB == false) {
        if (pidOn && !pidCalib) {
            powerUser = !powerUser;
            puMsg = 100;
            prevPB = true;
        }
    }
}


void updateAdjusts(int direction) {
    if (!powerUser) {
        if (prevB == false) {
            pressTime = millis();
            if (pidOn && !pidCalib) {
                gain = constrain(gain + direction * 0.01, 0, 1);
                pid.SetTunings(gain * KP, gain * KI, gain * KD_MULTIPLIER * KD * (1 + sensitivity));

            } else if (pidCalib == 1) {
                sensitivity = constrain(sensitivity + direction * 0.01, 0, 1);
                pid.SetTunings(gain * KP, gain * KI, gain * KD_MULTIPLIER * KD * (1 + sensitivity));

            } else {
                trimValue = constrain(trimValue - direction * 0.1, -G, G);
                output = trimValue;
                servo.write(convert_output(trimValue));
            }
            prevB = true;

        } else if ((millis() - pressTime) > 500) {
            if (pidOn && !pidCalib) {
                gain = constrain(gain + direction * 0.01, 0, 1);
                pid.SetTunings(gain * KP, gain * KI, gain * KD_MULTIPLIER * KD * (1 + sensitivity));

            } else if (pidCalib == 1) {
                sensitivity = constrain(sensitivity + direction * 0.01, 0, 1);
                pid.SetTunings(gain * KP, gain * KI, gain * KD_MULTIPLIER * KD * (1 + sensitivity));

            } else {
                trimValue = constrain(trimValue - direction * 0.1, -G, G);
                output = trimValue;
                servo.write(convert_output(trimValue));
            }
        }
    } else {
        if (prevB == false) {
            pressTime = millis();
            if (pidOn && !pidCalib) {
                gain = constrain(gain + direction * 0.01, 0, 1);
                pid.SetTunings(gain * KP, gain * KI, gain * KD_MULTIPLIER * KD * (1 + sensitivity));

            } else if (pidCalib == 1) {
                alpha = constrain(alpha + direction * 0.01, 0, 1);
                accelPole1.setFrequency(alpha * 2);
                accelPole2.setFrequency(alpha * 3);
                accelPole3.setFrequency(alpha * 4);
                gyroPole1.setFrequency(alpha * 2);
                gyroPole2.setFrequency(alpha * 3);
                gyroPole3.setFrequency(alpha * 4);

            } else if (pidCalib == 2) {
                KP = constrain(KP + direction * 0.01, 0, 1);
                pid.SetTunings(gain * KP, gain * KI, gain * KD_MULTIPLIER * KD * (1 + sensitivity));

            } else if (pidCalib == 3) {
                KD = constrain(KD + direction * 0.01, 0, 1);
                pid.SetTunings(gain * KP, gain * KI, gain * KD_MULTIPLIER * KD * (1 + sensitivity));

            } else if (pidCalib == 4) {
                KI = constrain(KI + direction * 0.01, 0, 1);
                pid.SetTunings(gain * KP, gain * KI, gain * KD_MULTIPLIER * KD * (1 + sensitivity));

            } else if (pidCalib == 5) {
                nl = constrain(nl + direction * 0.01, 0, 1);

            } else if (pidCalib == 6) {
                gainG = constrain(gainG + direction * .01, 0, 1);

            } else if (pidCalib == 7) {
                gyroT = constrain(gyroT + direction * .01, 0, 1);
                mpu.setThreshold(gyroT / 10);

            } else if (pidCalib == 8) {
                sensorReverse = constrain(sensorReverse + direction * 2, -1, 1);

            } else if (pidCalib == 9) {
                pidMode = constrain(pidMode + direction * 1, 0, 1);
                pid.SetControllerDirection(pidMode);

            } else if (pidCalib == 10) {
                axis = constrain(axis + direction * 1, 0, 2);

            } else {
                trimValue = constrain(trimValue - direction * 0.1, -G, G);
                output = trimValue;
                servo.write(convert_output(trimValue));
            }
            prevB = true;

        } else if ((millis() - pressTime) > 500) {
            if (pidOn && !pidCalib) {
                gain = constrain(gain + direction * 0.01, 0, 1);
                pid.SetTunings(gain * KP, gain * KI, gain * KD_MULTIPLIER * KD * (1 + sensitivity));

            } else if (pidCalib == 1) {
                alpha = constrain(alpha + direction * 0.01, 0, 1);
                accelPole1.setFrequency(alpha * 2);
                accelPole2.setFrequency(alpha * 3);
                accelPole3.setFrequency(alpha * 4);
                gyroPole1.setFrequency(alpha * 2);
                gyroPole2.setFrequency(alpha * 3);
                gyroPole3.setFrequency(alpha * 4);

            } else if (pidCalib == 2) {
                KP = constrain(KP + direction * 0.01, 0, 1);
                pid.SetTunings(gain * KP, gain * KI, gain * KD_MULTIPLIER * KD * (1 + sensitivity));

            } else if (pidCalib == 3) {
                KD = constrain(KD + direction * 0.01, 0, 1);
                pid.SetTunings(gain * KP, gain * KI, gain * KD_MULTIPLIER * KD * (1 + sensitivity));

            } else if (pidCalib == 4) {
                KI = constrain(KI + direction * 0.01, 0, 1);
                pid.SetTunings(gain * KP, gain * KI, gain * KD_MULTIPLIER * KD * (1 + sensitivity));

            } else if (pidCalib == 5) {
                nl = constrain(nl + direction * 0.01, 0, 1);

            } else if (pidCalib == 6) {
                gainG = constrain(gainG + direction * .01, 0, 1);

            } else if (pidCalib == 7) {
                gyroT = constrain(gyroT + direction * .01, 0, 1);
                mpu.setThreshold(gyroT / 10);

            } else if (pidCalib == 8) {
                ;

            } else if (pidCalib == 9) {
                ;

            } else if (pidCalib == 10) {
                ;

            } else {
                trimValue = constrain(trimValue - direction * 0.1, -G, G);
                output = trimValue;
                servo.write(convert_output(trimValue));
            }
        }
    }
}


void readOnOff(void) {
    /* Reads the On-Off switch, witch sets the PID to automatic mode
     if ON, to manual mode otherwise.
    */

    //sample the state of the button - is it pressed or not?
    if (!pidCalib) {
        onOff = digitalRead(ON_OFF_PIN);
    } else {
        pidOnOff = digitalRead(ON_OFF_PIN);
    }

    //filter out any noise by setting a time buffer
    if ((millis() - pressTime) > debounceDelay) {

        //if the button has been pressed, set debounce timer and state
        if (onOff && (onOff != prevOnOff)) {
            pressTime = millis();
            prevOnOff = onOff;
        }

        if (!onOff && (onOff != prevOnOff)) {
            if ((millis() - pressTime) < calibrationTime) {
                if (pidOn) {
                    pid.SetMode(MANUAL);
                    output = trimValue;
                    pos = convert_output(output);
                    servo.write(pos);
                    pidOn = false;
                } else {
                    pid.SetTunings(gain * KP, gain * KI, gain * KD_MULTIPLIER * KD * (1 + sensitivity));
                    pid.SetMode(AUTOMATIC);
                    pidOn = true;
                }
            } else {
                if (!pidOn) {
                    calibrateAccelerometer();
                } else {
                    pidCalib++;
                }
            }
            prevOnOff = onOff;
        }
    }

    // Pid Calibration menu
    if ((millis() - pidPressTime) > debounceDelay) {
        //if the button has been pressed, set debounce timer and state
        if (pidOnOff && (pidOnOff != prevPidOnOff)) {
            pidPressTime = millis();
            prevPidOnOff = pidOnOff;
        }
        if (powerUser) {
            if (pidCalib != 10 && !pidOnOff && (pidOnOff != prevPidOnOff)) {
                pidCalib++;

            } else if (pidCalib == 10 && !pidOnOff && (pidOnOff != prevPidOnOff)) {

                pidCalib = 0;

                writeEEPROM(AXIS_ADDRESS, axis);
                writeEEPROM(GAING_ADDRESS, gainG);
                writeEEPROM(GYROT_ADDRESS, gyroT);
                writeEEPROM(SENSOR_REVERSE_ADDRESS, sensorReverse);
                writeEEPROM(ALPHA_ADDRESS, alpha);
                writeEEPROM(KP_ADDRESS, KP);
                writeEEPROM(KI_ADDRESS, KI);
                writeEEPROM(KD_ADDRESS, KD);
                writeEEPROM(PID_MODE_ADDRESS, pidMode);
                writeEEPROM(NL_ADDRESS, nl);

                // Update pointer to used axis
                if (axis == 0) {
                    usedAxis = &pitch;
                    usedGAxis = &gyroZ;
                } else if (axis == 1) {
                    usedAxis = &roll;
                    usedGAxis = &gyroZ;
                } else if (axis == 2) {
                    usedAxis = &yaw;
                    usedGAxis = &gyroX;
                }
            }
            prevPidOnOff = pidOnOff;
        } else {
            if (pidCalib != 1 && !pidOnOff && (pidOnOff != prevPidOnOff)) {
                pidCalib++;

            } else if (pidCalib == 1 && !pidOnOff && (pidOnOff != prevPidOnOff)) {

                pidCalib = 0;

                writeEEPROM(SENS_ADDRESS, sensitivity);
            }
            prevPidOnOff = pidOnOff;
        }
    }
}


void cfgButtons(void) {
    /**
     * Configures the buttons and switches.
     */
    pinMode(ON_OFF_PIN, INPUT_PULLDOWN);
    pinMode(PLUS_PIN, INPUT_PULLDOWN);
    pinMode(MINUS_PIN, INPUT_PULLDOWN);
}


// EEPROM PROCEDURES ###################################################################################################
void writeEEPROM(uint16 writeAddress, float variableWrite) {
    /**
     * Writes variableWrite at writeAddress
     */
    int16 temp;
    temp = variableWrite*100;
    dataWrite = temp;

    eepromStatus = EEPROM.write(writeAddress, dataWrite);
    if (eepromStatus == EEPROM_OK) {
    } else {
        if (writeAddress == GAIN_ADDRESS) {
            e1 = MSG_DELAY;
        } else if (writeAddress == TRIM_ADDRESS) {
            e2 = MSG_DELAY;
        } else if (writeAddress == OFFSET_PITCH_ADDRESS ||
                   writeAddress == OFFSET_ROLL_ADDRESS ||
                   writeAddress == OFFSET_YAW_ADDRESS) {
            e3 = MSG_DELAY;
        }
    }
}


float readEEPROM(uint16 readAddress) {
    /**
     * Returns the value at readAddress, if accessible otherwise, returns 0.
     */
    int16 temp;

    eepromStatus = EEPROM.read(readAddress, &dataRead);
    if (eepromStatus == EEPROM_OK) {
        temp = dataRead;
        return float(temp)/100;
    } else {
        if (readAddress == GAIN_ADDRESS) {
            e4 = MSG_DELAY;
        } else if (readAddress == TRIM_ADDRESS) {
            e5 = MSG_DELAY;
        } else if (readAddress == OFFSET_PITCH_ADDRESS ||
                   readAddress == OFFSET_ROLL_ADDRESS ||
                   readAddress == OFFSET_YAW_ADDRESS) {
            e6 = MSG_DELAY;
        }
        return 0;
    }
}


void cfgEEPROM(void) {
    /**
     * Configures the EEPROM emulation.
     */
//    EEPROM.PageBase0 = 0x801F000;
//    EEPROM.PageBase1 = 0x801F800;
//    EEPROM.PageSize  = 0x400;

    eepromStatus = EEPROM.init();
}


// LED PROCEDURES ######################################################################################################
#ifdef DEBUG
void blinkLED(void) {
//    digitalWrite(PB1, !digitalRead(PB1));
//    digitalWrite(PB12, !digitalRead(PB12));
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
}


void cfgLED(void) {
    // Flash LED
//    pinMode(PB1, OUTPUT);
//    digitalWrite(PB1, HIGH);
//    pinMode(PB12, OUTPUT);
//    digitalWrite(PB12, HIGH);
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, HIGH);
}
#endif

/** MAIN PROCEDURES ####################################################################################################
 *
 * #####################################################################################################################
 */

void setup() {
//    cfgLED();
#ifdef DEBUG
    cfgProbes();
#endif

    cfgEEPROM();
    cfgPID();
    cfgServo();
    cfgDisplay();

    iwdg_init(IWDG_PRE_256, IWDG_NUM); // enable watchdog

    cfgSensor();
    cfgButtons();
    cfgSensorWtdg();

//    failCount = readEEPROM(FAIL_ADDRESS);
    feedSensorWtdg();

}


void loop() {
//#ifdef DEBUG
//    flipP1();
//#endif
    // Keep initial loop time
    time = micros();

    // Increase counter
    i++;

    if (!canRead) {

    }

    if (i % SENSOR_MOD == 0) {
        if (canRead) {
            // Feed the dogs...
            iwdg_feed();
            feedSensorWtdg();

            readSensor();

            computePID();

        } else {
            recoverSensor();

            i = 1;
            canRead = true;
        }

    } else if (i % DISPLAY_MOD == 0) {
        refreshScreen();

    } else if (i % B_MOD == 0) {
        processInterface();
        i = 1;
    }

    // Wait for loop to complete
    while ((micros() - time) < loopTime) {
        if (I2C1->state == I2C_STATE_ERROR) {
            canRead = false;
            i = 1;
            break;
        }
    }
#ifdef DEBUG
    loopFreq = 1000000 / (micros() - time);
#endif
#ifdef DEBUG
    flipP1();
#endif
} // END LOOP
