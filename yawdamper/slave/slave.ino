/**
 * Tau, 15/06/2018
 * Yawdamper slave module
 *
 * TODO:
 *     EEPROM
 *     Servo
 *     Display
 *     Data Transmission
 */


#include <Arduino.h>
#include <SPI.h>
#include <Adafruit_SSD1306.h>


//#define DEBUG

// DISPLAY #############################################################################################################
// Display macros
#define OLED_DC                 PB0                 // m3
#define OLED_CS                 PB2                 // m2
#define OLED_RESET              PA4                 // m7

// Display vars
const int DISPLAY_MOD = 1000;
volatile int cycles = 0;

// Display instance
Adafruit_SSD1306 display(OLED_DC, OLED_RESET, OLED_CS);


// SPI #################################################################################################################
// SPI macros
#define SS PB12   //SPI_2 Chip Select pin is PB12. You can change it to the STM32 pin you want.

// SPI vars
char *str;
const int bufferSize = 64;
static char buf[bufferSize];
char c;
long time;


// SPI instance
SPIClass SPI_2(2); //Create an instance of the SPI Class called SPI_2 that uses the 2nd SPI Port


// Display procedures ##################################################################################################
void refreshDisplay(char *l, float i) {

    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.setCursor(2, 12);
    display.print(i);

    display.setCursor(2, 32);
    display.print(String(l));

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
}


// SPI procedures ######################################################################################################
void cfgSPI2(void) {
    // Setup SPI 2
    SPI_2.beginSlave(); //Initialize the SPI_2 port.
//    SPI_2.setBitOrder(MSBFIRST); // Set the SPI_2 bit order
//    SPI_2.setDataMode(SPI_MODE0); //Set the  SPI_2 data mode 0
//    SPI_2.setClockDivider(SPI_CLOCK_DIV16);  // Use a different speed to SPI 1
    pinMode(SS, INPUT);
}


char *readSPI2(int bytesToRead) {

    // clear buffer
    for (int pos = 0; pos < sizeof(buf) - 1; pos++)
        buf[pos] = '\0';

    // if you still have another byte to read:
    for (int pos = 0; pos < sizeof(buf) - 1; pos++) {
        c = SPI_2.read();

        if (c == '\n') {
            buf[pos] = '\0';
            break;
        }

        buf[pos] = c;
    }

    buf[sizeof(buf) - 1] = '\0';

    // return the result:
    return buf;
}


// Main procedures #####################################################################################################
void setup() {
    cfgLED();
    cfgDisplay();
    cfgSPI2();
}


void loop() {
    // Show activity
    time = millis();

//    if (i % 100 == 0)
    blinkLED();

    // increment counters
    cycles++;

    str = readSPI2(64);

    refreshDisplay(str, cycles);
//    Serial.println(&str);
} // END LOOP