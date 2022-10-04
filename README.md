# digitalelectronics2
Hello ! 

*welcome on the digital electronics page*

**new content will be added soon, stay tuned !**
```c
/***********************************************************************
 * 
 * Blink a LED in Arduino-style and use function from the delay library.
 * ATmega328P (Arduino Uno), 16 MHz, PlatformIO
 *
 * Copyright (c) 2022 Tomas Fryza
 * Dept. of Radio Electronics, Brno University of Technology, Czechia
 * This work is licensed under the terms of the MIT license.
 * 
 **********************************************************************/


/* Defines -----------------------------------------------------------*/
#define LED_GREEN PB5   // PB5 is AVR pin where green on-board LED 
                        // is connected
#define SHORT_DELAY 250 // Delay in milliseconds for the "."
#define SHORT_DELAY_2 750// Delay in milliseconds for the "-"
#define SHORT_DELAY_3 250 // Delay in milliseconds in between the dot and coma
#define BIG_DELAY 1000 // Delay in between the letters
#ifndef F_CPU
# define F_CPU 16000000 // CPU frequency in Hz required for delay funcs
#endif

/* Includes ----------------------------------------------------------*/
#include <avr/io.h>     // AVR device-specific IO definitions
#include <util/delay.h> // Functions for busy-wait delay loops


// -----
// This part is needed to use Arduino functions but also physical pin
// names. We are using Arduino-style just to simplify the first lab.
#include "Arduino.h"
#define PB5 13          // In Arduino world, PB5 is called "13"
// -----


/* Function definitions ----------------------------------------------*/
/**********************************************************************
 * Function: Main function where the program execution begins
 * Purpose:  Toggle one LED and use delay library.
 * Returns:  none
 **********************************************************************/
void morse_dot()
{
    digitalWrite(LED_GREEN,HIGH);
    _delay_ms(SHORT_DELAY);
    digitalWrite(LED_GREEN,LOW);
}
void morse_slash()
{
    digitalWrite(LED_GREEN,HIGH);
    _delay_ms(SHORT_DELAY_2);
    digitalWrite(LED_GREEN,LOW);
}

int main(void)
{
    uint8_t led_value = LOW;  // Local variable to keep LED status

    // Set pin where on-board LED is connected as output
    pinMode(LED_GREEN, OUTPUT);

    // Infinite loop
    while (1)
    {
        // Turn ON/OFF on-board LED
        digitalWrite(LED_GREEN, led_value);

        //Letter P in morse
        morse_dot();
         _delay_ms(SHORT_DELAY_3);
        morse_slash();
         _delay_ms(SHORT_DELAY_3);
        morse_slash();
         _delay_ms(SHORT_DELAY_3);
        morse_dot();
         _delay_ms(BIG_DELAY);
        
        //Letter A in morse
        morse_dot();
         _delay_ms(SHORT_DELAY_3);
        morse_slash();
         _delay_ms(BIG_DELAY);
        
        //Letter R in morse
        morse_dot();
         _delay_ms(SHORT_DELAY_3);
        morse_slash();
         _delay_ms(SHORT_DELAY_3);
        morse_dot();
         _delay_ms(BIG_DELAY);

        //Letter I in morse
        morse_dot();
         _delay_ms(SHORT_DELAY_3);
        morse_dot();
         _delay_ms(BIG_DELAY);

        //Letter S in morse
        morse_dot();
         _delay_ms(SHORT_DELAY_3);
        morse_dot();
         _delay_ms(SHORT_DELAY_3);
        morse_dot();
        _delay_ms(BIG_DELAY);

        // Pause several milliseconds
        //_delay_ms(SHORT_DELAY);

        /*Change LED value
        if (led_value == LOW)
            led_value = HIGH;
        else
            led_value = LOW;
        */

        
    }
}

```


