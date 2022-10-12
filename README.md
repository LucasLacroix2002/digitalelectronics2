# digitalelectronics2
Hello ! 

*welcome on the digital electronics page*

**new content will be added soon, stay tuned !**
# Lab 1: LUCAS LACROIX MORSE CODE

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












# Lab 2: LUCAS LACROIX TRAFFIC LIGHTS

### GPIO control registers


   | **DDRB** | **PORTB** | **Direction** | **Internal pull-up resistor** | **Description** |
   | :-: | :-: | :-: | :-: | :-- |
   | 0 | 0 | input | no | Tri-state, high-impedance |
   | 0 | 1 | input | no | Tri-state (Hi-Z) |
   | 1 | 0 | output| no | Output Low (Sink) |
   | 1 | 1 | output| no | Output High (Source) |

### GPIO library


   | **Version** | **Size [B]** |
   | :-- | :-: |
   | Ver. 1: Arduino-style | 480 |
   | Ver. 2: Registers | 182 |
   | Ver. 3: Library functions | 182 |

### Traffic light


![capture d'écran lab2 électronique](https://user-images.githubusercontent.com/114081959/194910487-30ddc376-13cc-4314-b94a-39678fdeb4e3.png)

with the 3 first leds defining our traffic light ( with 3 different colors ), and the two last for pedestrian traffic lights ( only red and green led )



# Lab 3: YOUR_FIRSTNAME LASTNAME

### Overflow times

1. Complete table with overflow times.

   | **Module** | **Number of bits** | **1** | **8** | **32** | **64** | **128** | **256** | **1024** |
   | :-: | :-: | :-: | :-: | :-: | :-: | :-: | :-: | :-: |
   | Timer/Counter0 | 8  | 16u | 128u | -- | | -- | | |
   | Timer/Counter1 | 16 |     |      | -- | | -- | | |
   | Timer/Counter2 | 8  |     |      |    | |    | | |

### Interrupts

2. In `timer.h` header file, define macros also for Timer/Counter2. Listing of part of the header file with settings for Timer/Counter2. Always use syntax highlighting, meaningful comments, and follow C guidelines:

   ```c
   /**
    * @name  Definitions for 8-bit Timer/Counter2
    * @note  t_OVF = 1/F_CPU * prescaler * 2^n where n = 8, F_CPU = 16 MHz
    */
   // WRITE YOUR CODE HERE
   ```

   //prescaler:TCCR01

   //TCCR0B  CS02/CS01/CS00
     TCNT0
     OCR0A
     OCR0B
     TIMSK0
     TIFR0
