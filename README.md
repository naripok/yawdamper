# Yawdamper module for light experimental aircrafts

## DESCRIPTION:

This project uses a stm32f103c8, mpu6050 and ssd1306 as a yawdamper module for
light experimental aircraft applications.

This program is distributed under MIT license.  
It is an experimental module.   
**USE AT YOUR OWN RISK**

## PINMAP 
  
OLED
    PA4  -> RES  
    PA5  -> SCK  
    PA7  -> SDA  
    PB0  -> DC  
    PA6  -> CS  
  
BUTTONS
    PB13 -> MODE
    PB14 -> PLUS  
    PB15 -> MINUS    

SENSOR  
    PB12 -> INT  
    PB7  -> SDA  
    PB6  -> SCL  
    GND  -> GND  
    3.3  -> VCC  

SERVO  
    PA8  -> PWM  

CONTROLLER OUTPUT CONNECTOR  
    PIN1  ->  
    PIN2  ->  
    PIN3  ->  
    PIN4  ->  
    PIN5  -> SERVO GND  
    PIN6  ->  
    PIN7  ->  
    PIN8  ->  
    PIN9  -> GND  
    PIN10 -> PA8 -> PWM  
    PIN11 -> PB11  
    PIN12 -> PB10  
    PIN13 -> PB3  
    PIN14 ->  
    PIN15 -> 12V  

SERVO CONNECTOR  
    PIN1 -> VCC  
    PIN3 -> PWM  
    PIN9 -> GND  

### TODO:
    Better user interface