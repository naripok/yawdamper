/**
 * Tau, 15/06/2018
 * Yawdamper master module
 *
 * TODO:
 *     EEPROM
 *     PID
 *     Data Transmission
 */

#include <Arduino.h>
#include <SPI.h>
#include <Adafruit_SSD1306.h>
#include <EEPROM.h>
#include <MPU6050.h>
#include <PID_v1.h>


// SENSOR ##############################################################################################################
// Sensor macros
// #define SENSOR_SCL           PB6                  // m16
// #define SENSOR_SDA           PB7                  // m15
#define SENSOR_VCC              PA9                  // m26
#define INT_PIN                 PA8                  // m27
#define SDA                     PA7                  // m4
#define SCL                     PA5                  // m6
#define DLPF                    MPU6050_DLPF_6       // 5hz - 19ms delay

// Sensor vars
const int CALIBRATION_SAMPLES = 5000;                // calibration mean n of samples
const double G = 9.80665;                            // gravity as written in the sensor header

volatile double pitch = 0;                           // pitch measurement
volatile double roll = 0;                            // roll measurement
volatile double yaw = 0;                             // yaw measurement

volatile double offsetPitch;                         // offset after calibration
volatile double offsetRoll;                          // offset after calibration
volatile double offsetYaw;                           // offset after calibration

volatile double alpha = 0.0;                         // exponential filter decay
volatile int axis = 0;                               // axis selection helper variable
volatile double* usedAxis = &pitch;                  // pointer to used axis
volatile bool mpuInterrupt = false;                  // indicates whether MPU interrupt pin has gone high

// Sensor instance
MPU6050 mpu;


// SPI #################################################################################################################
// SPI macros
#define SS PB12   //SPI_2 Chip Select pin is PB12. You can change it to the STM32 pin you want.

// SPI instance
SPIClass SPI_2(2);  // Create an instance of the SPI Class called SPI_2 that uses the 2nd SPI Port


// DISPLAY #############################################################################################################
// Display macros
#define OLED_DC                 PB0                  // m3
#define OLED_CS                 PB2                  // m2
#define OLED_RESET              PA4                  // m7

// Display vars
const int DISPLAY_MOD = 1000;
volatile int i = 0;
const char* msg = "Hello, World!\n";

// Display instance
Adafruit_SSD1306 display(OLED_DC, OLED_RESET, OLED_CS);



// SPI procedures ######################################################################################################
void sendCharSPI2(const char* charMsg) {
    char c;

    SPI_2.beginTransaction(SPISettings(SPI_CLOCK_DIV16, MSBFIRST, SPI_MODE0));
    digitalWrite(SS, LOW);                                              // manually take CSN low for SPI_2 transmission

    // send test string
    for (int p = 0; c = charMsg[p]; p++) {
        delayMicroseconds(15);
        SPI_2.write(c);

        // Show activity
        blinkLED();
        refreshDisplay(c);
        Serial.print(c);
        // increment counters
        i++;
    }

    digitalWrite(SS, HIGH);                                             // manually take CSN high between spi transmissions
    SPI_2.endTransaction();                                             // transaction over
}


void sendIntSPI2(const int* intMsg) {
    char c;

    SPI_2.beginTransaction(SPISettings(SPI_CLOCK_DIV16, MSBFIRST, SPI_MODE0));
    digitalWrite(SS, LOW);                                              // manually take CSN low for SPI_2 transmission

    // send test string
    for (int p = 0; c = msg[p]; p++) {
        delayMicroseconds(15);
        SPI_2.write(c);

        // Show activity
        blinkLED();
        refreshDisplay(c);
        Serial.print(c);
        // increment counters
        i++;
    }

    digitalWrite(SS, HIGH);                                             // manually take CSN high between spi transmissions
    SPI_2.endTransaction();                                             // transaction over
}


void cfgSPI2(void) {
    // Setup SPI 2
    SPI_2.begin(); //Initialize the SPI_2 port.
//    SPI_2.setBitOrder(MSBFIRST); // Set the SPI_2 bit order
//    SPI_2.setDataMode(SPI_MODE0); //Set the  SPI_2 data mode 0
//    SPI_2.setClockDivider(SPI_CLOCK_DIV2);  // Use a different speed to SPI 1
    pinMode(SS, OUTPUT);
}


// Sensor procedures ###################################################################################################
void DataReady(void) {
    mpuInterrupt = true;
}


void readSensor(void) {
    /**
     * Reads the values from the sensor to be
     * stored at pitch, roll and yaw
     */

    // Read sensor
    Vector norm = mpu.readNormalizeAccel();

    // Apply exponential filtering to chosen axis
    pitch = (pitch * (1 - (alpha / 10)) + sensorReverse * (alpha / 10) * (norm.YAxis - offsetPitch));
    roll = (roll * (1 - (alpha / 10)) + sensorReverse * (alpha / 10) * (norm.XAxis - offsetRoll));
    yaw = (yaw * (1 - (alpha / 10)) + sensorReverse * (alpha / 10) * (norm.ZAxis - offsetYaw));

    // Update interruption flag
    mpuInterrupt = false;
}


void cfgSensor(void) {
    /* Configures MPU6050.
    */

    // Configure controller VCC pin as output
    pinMode(SENSOR_VCC, OUTPUT);

    // Config interruption pin
    // Set INT controller port to input and attach interruption to it
    pinMode(INT_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(INT_PIN), dmpDataReady, FALLING);

    // Feed watchdog
    iwdg_feed();

    // Power cycle MPU for fresh start
    digitalWrite(SENSOR_VCC, LOW);
    for (int i = 50; i > 0; i--) {
        iwdg_feed();
        delay(1);
    }
    digitalWrite(SENSOR_VCC, HIGH);
    for (int i = 100; i > 0; i--) {
        iwdg_feed();
        delay(1);
    }

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
    mpu.setDLPFMode(DLPF);                              // Set low pass filter band
    mpu.setTempEnabled(false);                          // disable temperature sensor
    mpu.setAccelPowerOnDelay(MPU6050_DELAY_3MS);        // delay start for compatibility issues

    mpu.writeRegister8(0x23, 0b00000000);               // Disable FIFO queues
    mpu.writeRegister8(0x37, 0b10110000);               // Interruption pin config
    mpu.writeRegister8(0x38, 0b00000001);               // Interruption config

//    // Read offsets from EEPROM
//    offsetPitch = readEEPROM(OFFSET_PITCH_ADDRESS);
//    offsetRoll = readEEPROM(OFFSET_ROLL_ADDRESS);
//    offsetYaw = readEEPROM(OFFSET_YAW_ADDRESS);
//
//    // Read used axis from EEPROM
//    axis = readEEPROM(AXIS_ADDRESS);

    // Update pointer to used axis
    if (axis == 0) {
        usedAxis = &pitch;
    } else if (axis == 1) {
        usedAxis = &roll;
    } else if (axis == 2) {
        usedAxis = &yaw;
    }
}


// Display procedures ##################################################################################################
void drawInfo(char l) {
    if (i % 14 == 0) {
        display.clearDisplay();
        display.setTextSize(1);
        display.setTextColor(WHITE);
    }

    display.setCursor(2 + (i % 10) * 5, 12);
    display.print(i % 10);
    display.setCursor(2 + (i % 14) * 5, 32);
    display.print(l);

    display.fillRect(2, 50, 128, 64, BLACK);
    display.setCursor(2, 50);
    display.print(i);
}


void refreshDisplay(char l) {
    drawInfo(l);
    display.display();
}


void cfgDisplay(void) {
    display.begin(SSD1306_SWITCHCAPVCC);
    display.clearDisplay();
}


// LED procedures ######################################################################################################
void blinkLED(void) {
    digitalWrite(PB1, !digitalRead(PB1));
}


void cfgLED(void) {
    // Flash LED
    pinMode(PB1, OUTPUT);
    digitalWrite(PB1, HIGH);
//    for (int i = 0; i < 10; i++) {
//        blinkLED();
//    }
}


// Main procedures #####################################################################################################
void setup() {
    cfgLED();
    cfgDisplay();
    cfgSPI2();
//    cfgSensor();
}


void loop() {
    // If there is new data on sensor registers, read it
//    if (mpuInterrupt) {
//        readSensor();
//    }

    sendCharSPI2(msg);

    delay(500);
} // END LOOP