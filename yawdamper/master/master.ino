/**
 * Tau, 15/06/2018
 * Yawdamper master module
 *
 * TODO:
 *     EEPROM
 *     Sensor read <-
 *     PID
 *     Servo
 *     Watchdog
  * This project uses a stm32f103c6, mpu5060, ssd1306 as a yawdamper module for
 * light aircraft applications
 *
 * ## PINMAP ###########################################################################################################
 *
 *  OLED
 *      PA4  -> RES
 *      PA5  -> SCK
 *      PA7  -> SDA
 *      PA3/mPB0  -> DC
 *      PA2/mPB2  -> CS
 *
 *  BUTTONS
 *      PB10 -> PLUS  -> PIN12
 *      PB11 -> MINUS -> PIN11
 *      PB1/mPB3  -> MODE  -> PIN13
 *
 *  SENSOR
 *      PB12 -> INT   -> PIN1
 *      PB7  -> SDA   -> PIN5
 *      PB6  -> SCL   -> PIN6
 *      GND  -> GND   -> PIN7
 *      3.3  -> VCC   -> PIN8
 *
 *  SERVO
 *      PA0  -> PWM   -> PIN10
 *
 *  CONTROLLER OUTPUT CONNECTOR
 *      PIN1  -> PB12
 *      PIN2  ->
 *      PIN3  ->
 *      PIN4  ->
 *      PIN5  -> PB7
 *      PIN6  -> PB6
 *      PIN7  -> SENSOR GND
 *      PIN8  -> 3.3V
 *      PIN9  -> GND
 *      PIN10 -> PA9
 *      PIN11 -> PB11
 *      PIN12 -> PB10
 *      PIN13 -> PB3
 *      PIN14 -> GND
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
#include <libmaple/nvic.h>
//#include <series/nvic.h>
#include "stm32f1/include/series/nvic.h"
#include <libmaple/libmaple.h>
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


// Time ################################################################################################################
volatile long loopTime = 3000;
volatile long i = 1;
volatile long waitTime = 0;

// Watchdog ############################################################################################################
#define IWDG_NUM                300
#define IWDG_PRESCALER          IWDG_PRE_256


// SENSOR ##############################################################################################################
// Sensor macros
// #define SENSOR_SCL           PB6                  // m16
// #define SENSOR_SDA           PB7                  // m15
#define SENSOR_VCC              PB4                  // m26
#define INT_PIN                 PB12//PA8                  // m27
#define SDA                     PB7//PA7                  // m4
#define SCL                     PB6//PA5                  // m6
#define DLPF                    MPU6050_DLPF_6       // 5hz - 19ms delay

// Sensor vars
//const int SENSOR_MOD = 5;
const int SENSOR_MOD = 2;
const int CALIBRATION_SAMPLES = 5000;                // calibration mean n of samples
const float G = 9.80665;                             // gravity as written in the sensor header

volatile float pitch = 0;                            // pitch measurement
volatile float roll = 0;                             // roll measurement
volatile float yaw = 0;                              // yaw measurement

volatile float offsetPitch;                          // offset after calibration
volatile float offsetRoll;                           // offset after calibration
volatile float offsetYaw;                            // offset after calibration

volatile float offsetGx;                             // offset after calibration
volatile float offsetGy;                             // offset after calibration
volatile float offsetGz;                             // offset after calibration

volatile float gyroX = 0;
volatile float gyroY = 0;
volatile float gyroZ = 0;

volatile float alpha = 0.1;                          // exponential filter decay
volatile float *usedAxis;                            // pointer to used axis
volatile float *usedGAxis;                            // pointer to used axis
volatile int axis = 0;                               // axis selection helper variable
volatile int sensorReverse = 1;

// Sensor instances
Vector normA;                                        // Accelerometer vector
Vector normG;                                        // Gyro vector
MPU6050 mpu;


// PID #################################################################################################################
// Time step vars
const double PID_FREQ = 60;
const double TIME_STEP = 1 / PID_FREQ;
double input = 0.0;
double output = 0.0;
double filteredOutput = 0.0;
double setpoint = 0.0;
double gain = 0.0;
double gainG = 0.0;
volatile double KP = 0.0;                            // proportional
volatile double KI = 0.0;                            // integral
volatile double KD = 0.0;                            // derivative
volatile int pidMode = 0;                            // 0 -> DIRECT, 1 -> REVERSE
volatile bool pidOn = false;                         // true if in automatic mode, false if in manual


// PID instance
PID pid(&input, &output, &setpoint, gain*KP, gain*KI, gain*KD/5, pidMode);


// DISPLAY #############################################################################################################
// Display macros
#define OLED_DC                 PA3//PB0                  // m3
#define OLED_CS                 PA2//PB2                  // m2
#define OLED_RESET              PA4                       // m7

// Display vars
//const int DISPLAY_MOD = 23;
const int DISPLAY_MOD = 5;
volatile long time = micros();
volatile long initTime = time / 1000;

volatile int j = 0; // activity counter
const int MSG_DELAY = 20;
int e1 = 0;
int e2 = 0;
int e3 = 0;
int e4 = 0;
int e5 = 0;
int e6 = 0;
volatile int pidCalib = 0;
volatile int overflowCounter = 0;

// Display instance
Adafruit_SSD1306 display(OLED_DC, OLED_RESET, OLED_CS);


// SERVO ###############################################################################################################
// Servo macros
#define SERVO_PIN               PA10//PA1                  // m27

// Servo vars
//const int SERVO_MOD = 11;
const int SERVO_MOD = 3;
const int SERVO_MIN_DEG = 0;
const int SERVO_MAX_DEG = 180;
volatile int pos = 90;                               // position in degrees
volatile float trimValue = 0;                        // value of output when pid is on manual mode

// Servo instance
Servo servo;


// BUTTONS #############################################################################################################
// Buttons macros
#define ON_OFF_PIN              PB1//PB3                  // m19
#define PLUS_PIN                PB10                 // m1
#define MINUS_PIN               PB11                 // m0

// Buttons vars
//const int B_MOD = 49;
const int B_MOD = 7;
volatile bool onOff;
volatile bool pidOnOff;
volatile bool minusB;
volatile bool plusB;
volatile bool prevOnOff;
volatile bool prevPidOnOff;
volatile bool prevCalibration;
volatile bool prevPlusB;
volatile bool prevMinusB;
volatile unsigned long pressTime;
volatile unsigned long pidPressTime;
volatile unsigned long releaseTime = -1500; // initialized as -1500 so that printTuning is not called at the first loops
volatile unsigned long calibrationTime = 1000;
volatile unsigned long debounceDelay = 300;


// EEPROM ##############################################################################################################
// EEPROM macros
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
#define DEBUG_LEVEL DEBUG_NONE
//#define I2C_DEBUG

#define LED_PIN                 PC13
#define PROBE_PIN               PA15
#define PROBE2_PIN              PB13
#define SENSOR_IWDG             10000

volatile bool canRead = true;
volatile int intCount = 0;


void cfgProbes(void) {
    pinMode(PROBE_PIN, OUTPUT);
    digitalWrite(PROBE_PIN, HIGH);
    pinMode(PROBE2_PIN, OUTPUT);
    digitalWrite(PROBE2_PIN, HIGH);
}

void flipP1(void) {
    digitalWrite(PROBE_PIN, !digitalRead(PROBE_PIN));
}

void flipP2(void) {
    digitalWrite(PROBE2_PIN, !digitalRead(PROBE2_PIN));
}

// TIMER PROCEDURES ####################################################################################################

#define T                   Timer4
#define c                   1

void cfgSensorWtdg(void) {
    T.setPeriod(SENSOR_IWDG);
    T.attachInterrupt(c, resetSensor);

//    nvic_irq_disable(NVIC_I2C1_ER);
//    nvic_irq_disable(NVIC_HARDFAULT);
//    nvic_irq_disable(NVIC_BUS_FAULT);
//    nvic_irq_disable(NVIC_USAGE_FAULT);

}


void resetSensor(void) {

    intCount++;
//    flipP2();

    digitalWrite(SENSOR_VCC, LOW);
    digitalWrite(SDA, LOW);
    delay_us(10);
    digitalWrite(SCL, LOW);
    delay_us(10);
    digitalWrite(SCL, HIGH);

    digitalWrite(SENSOR_VCC, HIGH);

    nvic_globalirq_enable();

    canRead = false;

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

    gyroX = (gyroX * (1 - (alpha / 20)) + sensorReverse * (alpha / 20) * (normG.XAxis / 5 - offsetGx));
    gyroY = (gyroY * (1 - (alpha / 20)) + sensorReverse * (alpha / 20) * (normG.YAxis / 5 - offsetGy));
    gyroZ = (gyroZ * (1 - (alpha / 20)) + sensorReverse * (alpha / 20) * (normG.ZAxis / 5 - offsetGz));
}


void calibrateAccelerometer(void) {
    /* Reads a "calibrationSamples" amount of values from the sensor
     saving the mean to the offset variables.
    */

    iwdg_feed();
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


void cfgSensor(void) {
    /* Configures MPU6050.
    */

    // Feed watchdog
    iwdg_feed();
    feedSensorWtdg();

    // Configure controller VCC pin as output
    pinMode(SENSOR_VCC, OUTPUT);

    // Config interruption pin
    // Set INT controller port to input and attach interruption to it
    pinMode(INT_PIN, INPUT_PULLUP);
//    attachInterrupt(digitalPinToInterrupt(INT_PIN), dataReady, FALLING);

    // Power cycle MPU for fresh start
    digitalWrite(SENSOR_VCC, LOW);
    delay(3);
//
    digitalWrite(SENSOR_VCC, HIGH);
    delay(1);

    // Initialize MPU6050
    while(!mpu.begin(MPU6050_SCALE_250DPS, MPU6050_RANGE_2G)) {

        // While not initialized, display error msg
        display.clearDisplay();
        display.setTextSize(1);
        display.setTextColor(WHITE);
        display.setCursor(12, 25);
        display.println("FALHA NO SENSOR");
        display.display();
        delay(1);
    };

    // Configure mpu
    mpu.writeRegister8(0x24, 0b00001001);               // 400khz clock

    mpu.writeRegister8(MPU6050_REG_GYRO_CONFIG, 0b00000000);               // Gyro self test disable

    mpu.setDLPFMode(DLPF);                              // Set low pass filter band
    mpu.setTempEnabled(false);                          // disable temperature sensor
    mpu.setAccelPowerOnDelay(MPU6050_DELAY_1MS);        // delay start for compatibility issues

    mpu.writeRegister8(0x23, 0b00000000);               // Disable FIFO queues
//    mpu.writeRegister8(0x37, 0b00010000);               // Interruption pin config
//    mpu.writeRegister8(0x38, 0b00000001);               // Interruption config

    // Read offsets from EEPROM
    offsetPitch = readEEPROM(OFFSET_PITCH_ADDRESS);
    offsetRoll = readEEPROM(OFFSET_ROLL_ADDRESS);
    offsetYaw = readEEPROM(OFFSET_YAW_ADDRESS);

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

    mpu.setThreshold(0.1);

//    mpu.calibrateGyro(50);
}


// PID procedures ######################################################################################################
void computePID(void) {
    // Calculates the output of the PID
    input = setpoint - *usedAxis - *usedGAxis * gainG;

    if (pid.Compute()) {
        // Filter output for smoothness
        filteredOutput = filteredOutput * (1 - alpha) + alpha * output;

        // Converts the output to a value in degree
        pos = convert_output(filteredOutput);
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

    filteredOutput = trimValue;
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
    servo.attach(SERVO_PIN, 1000, 2000, 0, 180);
    servo.write(convert_output(trimValue));
}


// DISPLAY PROCEDURES ##################################################################################################
void refreshDisplay(int l, long time, double freq) {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(WHITE);

    display.setCursor(2, 2);
    display.print(l);

    display.setCursor(2, 22);
    display.print(time);

    display.setCursor(2, 42);
    display.print(freq);

    display.display();
}


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
//    display.println(intCount);


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
    display.println("GANHO");

    display.setTextSize(1);
    display.setCursor(2 + offset, 48);
    display.println("0");
    display.setCursor(122 - offset, 48);
    display.println("1");
    display.setCursor(53, 48);
    display.println(gain);

    display.fillRect(63, 38, 3, 5, WHITE);
    display.writeFastVLine(19 + offset, 40, 3, WHITE);
    display.writeFastVLine(34 + offset, 40, 3, WHITE);
    display.writeFastVLine(49 + offset, 40, 3, WHITE);
    display.writeFastVLine(79 - offset, 40, 3, WHITE);
    display.writeFastVLine(94 - offset, 40, 3, WHITE);
    display.writeFastVLine(109 - offset, 40, 3, WHITE);

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
    plusB = !digitalRead(PLUS_PIN);

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
            } else if (pidCalib == 5) {
                alpha = constrain(alpha + 0.01, 0, 1);
            } else if (pidCalib == 8) {
                gainG = constrain(gainG + .01, 0, 1);
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
    minusB = !digitalRead(MINUS_PIN);

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
            } else if (pidCalib == 5) {
                alpha = constrain(alpha - 0.01, 0, 1);
            } else if (pidCalib == 8) {
                gainG = constrain(gainG - .01, 0, 1);
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
        onOff = !digitalRead(ON_OFF_PIN);
    } else {
        pidOnOff = !digitalRead(ON_OFF_PIN);
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

        if (pidCalib != 8 && !pidOnOff && (pidOnOff != prevPidOnOff)) {
            pidCalib++;

        } else if (pidCalib == 8 && !pidOnOff && (pidOnOff != prevPidOnOff)) {
            pidCalib = 0;

            writeEEPROM(AXIS_ADDRESS, axis);
            writeEEPROM(GAING_ADDRESS, gainG);
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
    pinMode(ON_OFF_PIN, INPUT_PULLUP);
    pinMode(PLUS_PIN, INPUT_PULLUP);
    pinMode(MINUS_PIN, INPUT_PULLUP);
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
//    cfgProbes();

    cfgDisplay();
    cfgEEPROM();

//    iwdg_init(IWDG_PRE_256, IWDG_NUM); // enable watchdog

    cfgSensor();
    cfgButtons();
    cfgPID();
    cfgServo();
    cfgSensorWtdg();
    feedSensorWtdg();
//    initTime = millis();
}


void loop() {
//    flipP1();
    // Feed the dog...
    iwdg_feed();
    feedSensorWtdg();

    if (!canRead) {

//        I2C1->state = I2C_STATE_ERROR;

        blinkLED();

//        i2c_stop_condition(I2C1);

        i2c_disable(I2C1);
        i2c_master_enable(I2C1, I2C_BUS_RESET);

        i2c_start_condition(I2C1);
        i2c_stop_condition(I2C1);

        nvic_globalirq_enable();

        cfgSensor();

//        I2C1->state = I2C_STATE_IDLE;

        canRead = true;
    }


    // Keep initial loop time
    time = micros();

    // Increase counter
    i++;

    if (i % SENSOR_MOD == 0) {
        if (canRead)
            feedSensorWtdg();
            readSensor();

        computePID();

//        blinkLED();

    } else if (i % SERVO_MOD == 0) {
        driveServo();

    } else if (i % DISPLAY_MOD == 0) {
        refreshScreen();

    } else if (i % B_MOD == 0) {
        processInterface();
        i = 1;

    }

    // Wait for loop to complete
    waitTime = loopTime - (micros() - time);

    if (waitTime > 0)
        delay_us(waitTime);

} // END LOOP