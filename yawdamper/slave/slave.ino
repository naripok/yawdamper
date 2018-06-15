#include <Arduino.h>
#include <SPI.h>
#include <Adafruit_SSD1306.h>


// Display macros
#define OLED_DC                 PB0                 // m3
#define OLED_CS                 PB2                 // m2
#define OLED_RESET              PA4                 // m7

// SPI macros
#define SS PB12   //SPI_2 Chip Select pin is PB12. You can change it to the STM32 pin you want.

// Display vars
const int DISPLAY_MOD = 1000;
volatile int i = 0;

// SPI vars
uint8 JUNK = 0x00;
volatile byte pos = 0;
char *str;
const int bufferSize = 256;


SPIClass SPI_2(2); //Create an instance of the SPI Class called SPI_2 that uses the 2nd SPI Port
Adafruit_SSD1306 display(OLED_DC, OLED_RESET, OLED_CS);


// Display procedures
void drawInfo(char *l) {
    char c;
    int j = 0;

    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.setCursor(2, 12);
    display.print(i);
    for (l; c = *l; l++) {
        display.setCursor(2 + j * 5, 32);
        Serial.println(c);
        display.print(c);
        j++;
    }
}


void refreshDisplay(char *l) {
    drawInfo(l);
    display.display();
}


void cfgDisplay(void) {
    display.begin(SSD1306_SWITCHCAPVCC);
    display.clearDisplay();
}


// LED procedures
void blinkLED(void) {
    digitalWrite(PB1, !digitalRead(PB1));
}


void cfgLED(void) {
    // Flash LED
    pinMode(PB1, OUTPUT);
    digitalWrite(PB1, HIGH);
}

// SPI procedures

void cfgSPI2(void) {
    // Setup SPI 2
    SPI_2.beginSlave(); //Initialize the SPI_2 port.
//    SPI_2.setBitOrder(MSBFIRST); // Set the SPI_2 bit order
//    SPI_2.setDataMode(SPI_MODE0); //Set the  SPI_2 data mode 0
//    SPI_2.setClockDivider(SPI_CLOCK_DIV16);  // Use a different speed to SPI 1
    pinMode(SS, INPUT);
}


//unsigned int readSPI2(int bytesToRead) {
//    unsigned long result = 0;   // result to return
//    byte inByte = 0;           // incoming byte from the SPI
//
//    // send a value of 0 to read the first byte returned:
//    result = SPI_2.read();
//    // decrement the number of bytes left to read:
//    bytesToRead--;
//    // if you still have another byte to read:
//    if (bytesToRead > 0) {
//        // shift the first byte left, then get the second byte:
//        result = result << 8;
//        inByte = SPI_2.read();
//        // combine the byte you just got with the previous one:
//        result = result | inByte;
//        // decrement the number of bytes left to read:
//        bytesToRead--;
//    }
//    // return the result:
//    return (result);
//}


char *readSPI2(int bytesToRead) {
    static char buf[bufferSize];
    char c;

    // send a value of 0 to read the first byte returned:
//    buf[pos++] = SPI_2.read();

//    SPI_2.read();
    // if you still have another byte to read:
    for (int pos = 0; pos < sizeof(buf) - 1; pos++) {
        c = SPI_2.read();

        if (c == '\n') {
            buf[pos] = 0;
            break;
        }

        Serial.print(c);
        Serial.print(i);

        buf[pos] = c;
    }

    Serial.println("");
    Serial.println(buf);

    // return the result:
    return buf;
}


void setup() {
    cfgLED();
    cfgDisplay();
    cfgSPI2();
}


void loop() {
    // Show activity
//    if (i % 100 == 0)
    blinkLED();

    // increment counters
    i++;

    str = readSPI2(15);

    refreshDisplay(str);
//    Serial.println(&str);
} // END LOOP