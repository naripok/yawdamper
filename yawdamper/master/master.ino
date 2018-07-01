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
 *      PA7  -> SDA
 *      PA3/mPB0  -> DC
 *      PA2/mPB2  -> CS
 *
 *  BUTTONS TODO
 *      PB10 -> PLUS  -> PIN12
 *      PB11 -> MINUS -> PIN11
 *      PB1/mPB3  -> MODE  -> PIN13
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
 *      PIN10 -> PA9 -> PWM
 *      PIN11 -> PB11
 *      PIN12 -> PB10
 *      PIN13 -> PB3
 *      PIN14 ->
 *      PIN15 -> 12V
 *
 *  SERVO CONNECTOR
 *      PIN1 -> VCC
 *      PIN3 -> PWM
 *      PIN9 -> GND
 *
 */

#include <Arduino.h>
#include <Adafruit_SSD1306.h>
#include <EEPROM.h>
#include <MPU6050.h>
#include <PID_v1.h>
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
#define PROBE_PIN               PB11
#define DEBUG_LEVEL DEBUG_NONE
//#define I2C_DEBUG

// Time ################################################################################################################
const unsigned long loopTime = 3000;
volatile unsigned int i = 1;

// Watchdog ############################################################################################################
#define IWDG_NUM                210
#define IWDG_PRESCALER          IWDG_PRE_256


// SENSOR ##############################################################################################################
// Sensor macros
// #define SENSOR_SCL           PB6                  // m16
// #define SENSOR_SDA           PB7                  // m15
#define SENSOR_VCC              PB4
//#define INT_PIN
#define SDA                     PB7
#define SCL                     PB6
#define DLPF                    MPU6050_DLPF_5       // 5hz - 19ms delay

#define SENSOR_IWDG             12000
#define T                       Timer2
#define c                       2


// Sensor vars
const unsigned int SENSOR_MOD = 2;
const unsigned int CALIBRATION_SAMPLES = 5000;                // calibration mean n of samples
const float G = 9.80665;                             // gravity as written in the sensor header

float pitch = 0;                            // pitch measurement
float roll = 0;                             // roll measurement
float yaw = 0;                              // yaw measurement

float offsetPitch;                          // offset after calibration
float offsetRoll;                           // offset after calibration
float offsetYaw;                            // offset after calibration
//
//float offsetGx;                             // offset after calibration
//float offsetGy;                             // offset after calibration
//float offsetGz;                             // offset after calibration

float gyroX = 0;
float gyroY = 0;
float gyroZ = 0;

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


// PID #################################################################################################################
// Time step vars
const double PID_FREQ = 168;
const double TIME_STEP = 1 / PID_FREQ;
double input = 0.0;
double output = 0.0;
double filteredOutput = 0.0;
double setpoint = 0.0;
double gain = 0.0;
double gainG = 0.0;
double gyroT = 0.0;
double KP = 0.0;                            // proportional
double KI = 0.0;                            // integral
double KD = 0.0;                            // derivative
int pidMode = 0;                            // 0 -> DIRECT, 1 -> REVERSE
bool pidOn = false;                         // true if in automatic mode, false if in manual


// PID instance
PID pid(&input, &output, &setpoint, gain*KP, gain*KI, gain*KD/5, pidMode);


// DISPLAY #############################################################################################################
// Display macros
#define OLED_DC                 PB0  // 7
#define OLED_CS                 SS  // 5
#define OLED_RESET              MISO  // 8

// Display vars
const int DISPLAY_MOD = 3;
long time;

int j = 0; // activity counter
const int MSG_DELAY = 20;
int e1 = 0;
int e2 = 0;
int e3 = 0;
int e4 = 0;
int e5 = 0;
int e6 = 0;
int pidCalib = 0;
//volatile int overflowCounter = 0;

// Display instance
Adafruit_SSD1306 display(OLED_DC, OLED_RESET, OLED_CS);


// SERVO ###############################################################################################################
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


// BUTTONS #############################################################################################################
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
bool prevPlusB;
bool prevMinusB;
unsigned long pressTime;
unsigned long pidPressTime;
unsigned long releaseTime = -1500; // initialized as -1500 so that printTuning is not called at the first loops
const unsigned long calibrationTime = 1000;
const unsigned long debounceDelay = 300;


// EEPROM ##############################################################################################################
// EEPROM macros
#define GYROT_ADDRESS           0x1F
#define FAIL_ADDRESS            0x1E
#define LASTSTATE_ADDRESS       0x1D
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


//// TIMER
//HardwareTimer sensorWtdg(4);

// END MEMORY MANAGEMENT ###############################################################################################


/**
 * #####################################################################################################################
 * ###                                                                                                               ###
 * ###                                                  PROCEDURES                                                   ###
 * ###                                                                                                               ###
 * #####################################################################################################################
 */

// FAULT HANDLING ######################################################################################################
#define LED_PIN                 PC13

volatile unsigned long failCount = 0;


void cfgProbes(void) {
    pinMode(PROBE_PIN, OUTPUT);
    digitalWrite(PROBE_PIN, LOW);

}


void flipP1(void) {
    digitalWrite(PROBE_PIN, !digitalRead(PROBE_PIN));
}


// TIMER PROCEDURES ####################################################################################################
void cfgSensorWtdg(void) {
    T.setPeriod(SENSOR_IWDG);
    T.attachInterrupt(c, resetSensor);

}


void resetSensor(void) {
    failCount++;

    canRead = false;
    writeEEPROM(FAIL_ADDRESS, failCount);
//    writeEEPROM(LASTSTATE_ADDRESS, pidOn);
}


void feedSensorWtdg(void) {
    T.refresh();
}


// SENSOR PROCEDURES ###################################################################################################
void readSensor(void) {
    /**
     * Reads values from the sensor registers
     */

    // Read sensor
    normA = mpu.readNormalizeAccel();
    normG = mpu.readNormalizeGyro();

    // Apply exponential filtering to chosen axis
    roll = (roll * (1 - (alpha / 20)) + sensorReverse * (alpha / 20) * (normA.XAxis - offsetRoll));
    pitch = (pitch * (1 - (alpha / 20)) + sensorReverse * (alpha / 20) * (normA.YAxis - offsetPitch));
    yaw = (yaw * (1 - (alpha / 20)) + sensorReverse * (alpha / 20) * (normA.ZAxis - offsetYaw));

    gyroX = (gyroX * (1 - (alpha / 20)) + sensorReverse * (alpha / 20) * (normG.XAxis / 5));
    gyroY = (gyroY * (1 - (alpha / 20)) + sensorReverse * (alpha / 20) * (normG.YAxis / 5));
    gyroZ = (gyroZ * (1 - (alpha / 20)) + sensorReverse * (alpha / 20) * (normG.ZAxis / 5));
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
//    mpu.writeRegister8(MPU6050_REG_GYRO_CONFIG, 0b00000000);              // Gyro self test disable
    mpu.setDLPFMode(DLPF);                                                  // Set low pass filter band
    mpu.setTempEnabled(false);                                              // disable temperature sensor
//    mpu.setAccelPowerOnDelay(MPU6050_DELAY_1MS);                          // delay start for compatibility issues
    mpu.writeRegister8(0x23, 0b00000000);                                   // Disable FIFO queues

    mpu.setThreshold(gyroT);
}


void cfgSensor(void) {
    /* Configures MPU6050.
    */

    // Feed watchdog
    iwdg_feed();
    feedSensorWtdg();

    // Configure controller VCC pin as output
    pinMode(SENSOR_VCC, OUTPUT);

    digitalWrite(SENSOR_VCC, HIGH);

    // Initialize MPU6050
    while(!mpu.begin(MPU6050_SCALE_250DPS, MPU6050_RANGE_2G)) {
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
    mpu.writeRegister8(0x24, 0b00001001);                                   // 400khz clock

    mpu.writeRegister8(MPU6050_REG_GYRO_CONFIG, 0b00000000);                // Gyro self test disable

    feedSensorWtdg();

    mpu.setDLPFMode(DLPF);                                                  // Set low pass filter band
    mpu.setTempEnabled(false);                                              // disable temperature sensor
//    mpu.setAccelPowerOnDelay(MPU6050_DELAY_1MS);                          // delay start for compatibility issues

    mpu.writeRegister8(0x23, 0b00000000);                                   // Disable FIFO queues
//    mpu.writeRegister8(0x37, 0b00010000);                                 // Interruption pin config
//    mpu.writeRegister8(0x38, 0b00000001);                                 // Interruption config

    feedSensorWtdg();

    // Read offsets from EEPROM
    offsetPitch = readEEPROM(OFFSET_PITCH_ADDRESS);
    offsetRoll = readEEPROM(OFFSET_ROLL_ADDRESS);
    offsetYaw = readEEPROM(OFFSET_YAW_ADDRESS);

    feedSensorWtdg();

    // Read used axis from EEPROM
    axis = readEEPROM(AXIS_ADDRESS);

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
    feedSensorWtdg();

    displayMsg("Calibrando Gyro...");

    gyroT = readEEPROM(GYROT_ADDRESS);

    mpu.setThreshold(gyroT);
    mpu.calibrateGyro(200);
}


// PID procedures ######################################################################################################
void computePID(void) {
    // Calculates the output of the PID
    input = setpoint - *usedAxis - *usedGAxis * gainG;

    if (pid.Compute()) {
#ifdef DEBUG
        flipP1();
#endif

        // Filter output for smoothness
        filteredOutput = filteredOutput * (1 - alpha) + alpha * output;

        // Converts the output to a value in degree
        pos = convert_output(filteredOutput);

        driveServo();
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
//    pidOn = readEEPROM(LASTSTATE_ADDRESS);

//    if (pidOn) {
//        filteredOutput = output = trimValue;
//        pid.SetTunings(gain*KP, gain*KI, gain*KD/5);
//        pid.SetMode(AUTOMATIC);
//        writeEEPROM(LASTSTATE_ADDRESS, 0);
//    } else {
    filteredOutput = trimValue;
//    }
}


// SERVO PROCEDURES ####################################################################################################
void driveServo(void) {
    servo.write(convert_output(filteredOutput));
}


int convert_output(float output) {
    pos = ((output + G) * (SERVO_MAX_DEG - SERVO_MIN_DEG) / (2 * G)) + SERVO_MIN_DEG;
    return pos;
}


void cfgServo(void) {
    /* Configures the servo.
    */
//    servo.attach(SERVO_PIN, 5000 - 2400, 5000 - 544, 0, 180);
//    servo.attach(SERVO_PIN, 1000, 2000, 0, 180);
    servo.attach(SERVO_PIN, 1200, 2000, 0, 180);
    servo.write(convert_output(trimValue));
}


// DISPLAY PROCEDURES ##################################################################################################
void refreshScreen(void) {

    if (pidCalib) {
        printPidTuning();
    } else if (pidOn && (plusB || minusB || (millis() - releaseTime) < 500)) {
        printTuning();
    } else if (e1 || e2 || e3){
        printError();
    } else {
        printControl();
    };
}


void displayMsg(String msg){
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.setCursor(12, 25);
    display.println(msg);
    display.display();
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

    // Temperature measurement
//    display.setCursor(28 + xOffset, 4 + yOffset);
//    display.println("T");
//    display.setCursor(36 + xOffset, 4 + yOffset);
//    display.println(int(temp));

    // Time overflow counter
    display.setCursor(28 + xOffset, 4 + yOffset);
    display.println("TON");
    display.setCursor(48 + xOffset, 4 + yOffset);
//    display.println((millis() - initTime) / 60000);
    display.println(systick_uptime() / 60000);
//    display.println(*usedGAxis);


    // Ball
    display.fillCircle(constrain(int(56.5 + *usedAxis * (258 / (2 * G))) + xOffset, 14 + xOffset, 98 + xOffset), 22 + yOffset, 5, WHITE);

    // Gyro
    display.fillRect(constrain(int(55.5 + *usedGAxis * (96 / (2 * G))) + xOffset, 9 + xOffset, 101 + xOffset), 37 + yOffset, 3, 6, WHITE);

    // Yaw position
    display.fillRect(constrain(int(55.5 - filteredOutput * (96 / (2 * G))) + xOffset, 9 + xOffset, 101 + xOffset), 44 + yOffset, 3, 6, WHITE);

    // Markers
    if (pidOn) {
        display.fillRect(5 + xOffset, 36 + yOffset, 4, 15, WHITE);
        display.fillRect(104 + xOffset, 36 + yOffset, 4, 15, WHITE);
        display.fillRect(5 + xOffset, 15 + yOffset, 4, 15, WHITE);
        display.fillRect(104 + xOffset, 15 + yOffset, 4, 15, WHITE);
    }

    display.fillRect(49 + xOffset, 15 + yOffset, 2, 15, WHITE);
    display.fillRect(62 + xOffset, 15 + yOffset, 2, 15, WHITE);
    display.fillRect(55 + xOffset, 35 + yOffset, 3, 2, WHITE);
    display.fillRect(55 + xOffset, 50 + yOffset, 3, 2, WHITE);

    // Print
    display.display();
}


void printTuning(void) {
    /* Prints the tuning on screen
    when there is a gain adjustment
    */
    int offset = 12;
    display.clearDisplay();
    display.setTextSize(2);
    display.setTextColor(WHITE);
    display.setCursor(35, 6);
    display.print("GANHO");

    display.setTextSize(1);
    display.setCursor(2 + offset, 48);
    display.print("0");
    display.setCursor(122 - offset, 48);
    display.print("1");
    display.setCursor(53, 48);
    display.print(gain);

//    display.fillRect(63, 38, 3, 5, WHITE);
//    display.writeFastVLine(19 + offset, 40, 3, WHITE);
//    display.writeFastVLine(37 + offset, 40, 3, WHITE);
//    display.writeFastVLine(55 + offset, 40, 3, WHITE);
    display.writeFastVLine(64, 40, 3, WHITE);
//    display.writeFastVLine(73 - offset, 40, 3, WHITE);
//    display.writeFastVLine(91 - offset, 40, 3, WHITE);
//    display.writeFastVLine(109 - offset, 40, 3, WHITE);

    display.drawRect(4 + offset, 24, 121 - 2 * offset, 20, WHITE);
    display.fillRect(4 + offset, 24, (1 + gain * (120 - 2 * offset)), 20, WHITE);
    display.display();
}


void printPidTuning(void) {
    display.clearDisplay();
    display.setTextSize(2);
    display.setTextColor(WHITE);
    if (pidCalib == 1) {
        display.setCursor(26, 24);
        display.println("Kp:");
        display.setCursor(60, 24);
        display.println(KP);
    } else if (pidCalib == 2) {
        display.setCursor(26, 24);
        display.println("Ki:");
        display.setCursor(60, 24);
        display.println(KI);
    } else if (pidCalib == 3) {
        display.setCursor(26, 24);
        display.println("Kd:");
        display.setCursor(60, 24);
        display.println(KD);
    } else if (pidCalib == 4) {
        display.setCursor(20, 12);
        display.println("Reverse:");
        display.setCursor(60, 32);
        display.println(pidMode);
    } else if (pidCalib == 5) {
        display.setCursor(30, 12);
        display.println("Alpha:");
        display.setCursor(40, 32);
        display.println(alpha);
    } else if (pidCalib == 6) {
        display.setCursor(22, 12);
        display.println("Sensor");
        display.setCursor(22, 32);
        display.println("Rev:");
        display.setCursor(68, 32);
        display.println(sensorReverse);
    } else if (pidCalib == 7) {
        display.setCursor(36, 12);
        display.println("Axis:");
        display.setCursor(60, 32);
        display.println(axis);
    } else if (pidCalib == 8) {
        display.setCursor(22, 12);
        display.println("Gr gain:");
        display.setCursor(38, 32);
        display.println(gainG);
    } else if (pidCalib == 9) {
        display.setCursor(22, 12);
        display.println("Gr th:");
        display.setCursor(38, 32);
        display.println(gyroT);
    } else if (pidCalib == 10) {
        display.setCursor(38, 12);
        display.println("Fail:");
        display.setCursor(50, 32);
        display.println(failCount);
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


void cfgDisplay(void) {
    display.begin(SSD1306_SWITCHCAPVCC);

    display.clearDisplay();
    display.setTextSize(3);
    display.setTextColor(WHITE);

    display.setCursor(8, 8);
    display.println("NOCTUA");

    // Print
    display.display();
}


// INTERFACE PROCEDURES ################################################################################################
void processInterface(void) {
    readPlusB();
    readMinusB();
    readOnOff();
}


void readPlusB(void) {
    /* Reads the Plus button, witch increases the variable gain if
     PID is on automatic mode and decreases the output if PID
     is on manual mode.
    */
    plusB = digitalRead(PLUS_PIN);

    if (plusB && !minusB) {
        if (prevPlusB == false) {
            pressTime = millis();
            if (pidOn && !pidCalib) {
                gain = constrain(gain + 0.01, 0, 1);
                pid.SetTunings(gain*KP, gain*KI, gain*KD/5);
            } else if (pidCalib == 1) {
                KP = constrain(KP + 0.01, 0, 9);
                pid.SetTunings(gain*KP, gain*KI, gain*KD/5);
            } else if (pidCalib == 2) {
                KI = constrain(KI + 0.01, 0, 9);
                pid.SetTunings(gain*KP, gain*KI, gain*KD/5);
            } else if (pidCalib == 3) {
                KD = constrain(KD + 0.01, 0, 9);
                pid.SetTunings(gain*KP, gain*KI, gain*KD/5);
            } else if (pidCalib == 4) {
                pidMode = constrain(pidMode + 1, 0, 1);
                pid.SetControllerDirection(pidMode);
            } else if (pidCalib == 5) {
                alpha = constrain(alpha + 0.01, 0, 1);
            } else if (pidCalib == 6) {
                sensorReverse = constrain(sensorReverse + 2, -1, 1);
            } else if (pidCalib == 7) {
                axis = constrain(axis + 1, 0, 2);
            } else if (pidCalib == 8) {
                gainG = constrain(gainG + .01, 0, 1);
            } else if (pidCalib == 9) {
                gyroT = constrain(gyroT + .01, 0, 1);
                mpu.setThreshold(gyroT);
            } else {
                trimValue = constrain(trimValue - 0.1, -G, G);
                filteredOutput = trimValue;
                servo.write(convert_output(trimValue));
            };
            prevPlusB = true;

        } else if ((millis() - pressTime) > 500){
            if (pidOn && !pidCalib) {
                gain = constrain(gain + 0.01, 0, 1);
                pid.SetTunings(gain*KP, gain*KI, gain*KD/5);
            } else if (pidCalib == 1) {
                KP = constrain(KP + 0.01, 0, 9);
                pid.SetTunings(gain*KP, gain*KI, gain*KD/5);
            } else if (pidCalib == 2) {
                KI = constrain(KI + 0.01, 0, 9);
                pid.SetTunings(gain*KP, gain*KI, gain*KD/5);
            } else if (pidCalib == 3) {
                KD = constrain(KD + 0.01, 0, 9);
                pid.SetTunings(gain*KP, gain*KI, gain*KD/5);
            } else if (pidCalib == 4) {
                ;
            } else if (pidCalib == 5) {
                alpha = constrain(alpha + 0.01, 0, 1);
            } else if (pidCalib == 6) {
                ;
            } else if (pidCalib == 7) {
                ;
            } else if (pidCalib == 8) {
                gainG = constrain(gainG + .01, 0, 1);
            } else if (pidCalib == 9) {
                gyroT = constrain(gyroT + .01, 0, 1);
                mpu.setThreshold(gyroT);
            } else if (pidCalib == 10) {
                ;
            } else {
                trimValue = constrain(trimValue - 0.1, -G, G);
                filteredOutput = trimValue;
                servo.write(convert_output(trimValue));
            };
        };

    } else {
        if(prevPlusB == true) {
            releaseTime = millis();
            prevPlusB = false;
            if (pidOn) {
                writeEEPROM(GAIN_ADDRESS, gain);
            } else {
                writeEEPROM(TRIM_ADDRESS, trimValue);
            }
        };
    };
}


void readMinusB(void) {
    /* Reads the Minus button, reducing the variable gain if
     PID is on automatic mode and reducing the output if PID
     is on manual mode.
    */
    minusB = digitalRead(MINUS_PIN);

    if (minusB && !plusB) {
        if (prevMinusB == false) {
            pressTime = millis();
            if (pidOn && !pidCalib) {
                gain = constrain(gain - 0.01, 0, 1);
                pid.SetTunings(gain*KP, gain*KI, gain*KD/5);
            } else if (pidCalib == 1) {
                KP = constrain(KP - 0.01, 0, 9);
                pid.SetTunings(gain*KP, gain*KI, gain*KD/5);
            } else if (pidCalib == 2) {
                KI = constrain(KI - 0.01, 0, 9);
                pid.SetTunings(gain*KP, gain*KI, gain*KD/5);
            } else if (pidCalib == 3) {
                KD = constrain(KD - 0.01, 0, 9);
                pid.SetTunings(gain*KP, gain*KI, gain*KD/5);
            } else if (pidCalib == 4) {
                pidMode = constrain(pidMode - 1, 0, 1);
                pid.SetControllerDirection(pidMode);
            } else if (pidCalib == 5) {
                alpha = constrain(alpha - 0.01, 0, 1);
            } else if (pidCalib == 6) {
                sensorReverse = constrain(sensorReverse - 2, -1, 1);
            } else if (pidCalib == 7) {
                axis = constrain(axis - 1, 0, 2);
            } else if (pidCalib == 8) {
                gainG = constrain(gainG - .01, 0, 1);
            } else if (pidCalib == 9) {
                gyroT = constrain(gyroT - .01, 0, 1);
                mpu.setThreshold(gyroT);
            } else if (pidCalib == 10) {
                failCount = 0;
                writeEEPROM(FAIL_ADDRESS, failCount);
            } else {
                trimValue = constrain(trimValue + 0.1, -G, G);
                filteredOutput = trimValue;
                servo.write(convert_output(trimValue));
            }
            prevMinusB = true;

        } else if ((millis() - pressTime) > 500){
            if (pidOn && !pidCalib) {
                gain = constrain(gain - 0.01, 0, 1);
                pid.SetTunings(gain*KP, gain*KI, gain*KD/5);
            } else if (pidCalib == 1) {
                KP = constrain(KP - 0.01, 0, 9);
                pid.SetTunings(gain*KP, gain*KI, gain*KD/5);
            } else if (pidCalib == 2) {
                KI = constrain(KI - 0.01, 0, 9);
                pid.SetTunings(gain*KP, gain*KI, gain*KD/5);
            } else if (pidCalib == 3) {
                KD = constrain(KD - 0.01, 0, 9);
                pid.SetTunings(gain*KP, gain*KI, gain*KD/5);
            } else if (pidCalib == 4) {
                ;
            } else if (pidCalib == 5) {
                alpha = constrain(alpha - 0.01, 0, 1);
            } else if (pidCalib == 6) {
                ;
            } else if (pidCalib == 7) {
                ;
            } else if (pidCalib == 8) {
                gainG = constrain(gainG - .01, 0, 1);
            } else if (pidCalib == 9) {
                gyroT = constrain(gyroT - .01, 0, 1);
                mpu.setThreshold(gyroT);
            } else if (pidCalib == 10) {
                ;
            } else {
                trimValue = constrain(trimValue + 0.1, -G, G);
                filteredOutput = trimValue;
                servo.write(convert_output(trimValue));
            }
        }

    } else {
        if(prevMinusB == true) {
            releaseTime = millis();
            prevMinusB = false;
            if (pidOn) {
                writeEEPROM(GAIN_ADDRESS, gain);
            } else {
                writeEEPROM(TRIM_ADDRESS, trimValue);
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
                    filteredOutput = trimValue;
                    pos = convert_output(filteredOutput);
                    servo.write(pos);
                    pidOn = false;
                } else {
//                    trimValue = output;
                    filteredOutput = output = trimValue;
                    pid.SetTunings(gain*KP, gain*KI, gain*KD/5);
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


/**
 * MAIN PROCEDURES #####################################################################################################
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

    failCount = readEEPROM(FAIL_ADDRESS);
    feedSensorWtdg();

}


void loop() {

    // Keep initial loop time
    time = micros();

    // Increase counter
    i++;

    if (!canRead) {
        i2c_disable(I2C1);
        i2c_master_enable(I2C1, I2C_BUS_RESET);

        recoverSensor();

        for (int i = 0; i < 10; i++) {
            feedSensorWtdg();
            iwdg_feed();
            delay(1);
        }

        i = 1;
        canRead = true;

    }

    if (i % SENSOR_MOD == 0) {
        if (canRead) {

            // Feed the dogs...
            iwdg_feed();
            feedSensorWtdg();

            readSensor();
        }
        computePID();

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

} // END LOOP
