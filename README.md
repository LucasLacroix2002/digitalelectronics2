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




   

# Lab 3: LUCAS LACROIX LAB3


| **Module** | **Number of bits** | **1** | **8** | **32** | **64** | **128** | **256** | **1024** |
   | :-: | :-: | :-: | :-: | :-: | :-: | :-: | :-: | :-: |
   | Timer/Counter0 | 8  | 16us | 128us | -- | 1ms | -- | 4ms | 16ms |
   | Timer/Counter1 | 16 | 4ms  | 33ms  | -- | 262ms | -- | 1s | 4.2s |
   | Timer/Counter2 | 8  | 16us | 128us | 512us | 1ms | 2ms | 4ms | 16ms |
   
   
```c
/** @brief Stop timer, prescaler 000 --> STOP */
#define TIM2_stop()             TCCR2B &= ~((1<<CS22) | (1<<CS21) | (1<<CS20));
/** @brief Set overflow 16us, prescaler 001 --> 1 */
#define TIM2_overflow_16us()    TCCR2B &= ~((1<<CS22) | (1<<CS21)); TCCR2B |= (1<<CS20);
/** @brief Set overflow 128us, prescaler 010 --> 8 */
#define TIM2_overflow_128us()   TCCR2B &= ~((1<<CS22) | (1<<CS20)); TCCR2B |= (1<<CS21);
/** @brief Set overflow 512us, prescaler 011 --> 32*/
#define TIM2_overflow_512us()   TCCR2B &= ~((1<<CS22); TCCR2B |= (1<<CS21) | (1<<CS20);
/** @brief Set overflow 1ms, prescaler 100 --> 64 */
#define TIM2_overflow_1ms()      TCCR2B &= ~(1<<CS21) | (1<<CS20); TCCR2B |= (1<<CS22);
/** @brief Set overflow 2ms, prescaler 101 --> 128 */
#define TIM2_overflow_2ms()      TCCR2B &= ~(1<<CS21); TCCR2B |= (1<<CS22) | (1<<CS20);
/** @brief Set overflow 4ms, prescaler 110 --> 256 */
#define TIM2_overflow_4ms()      TCCR2B &= ~((1<<CS20); TCCR2B |= (1<<CS22) | (1<<CS21);
/** @brief Set overflow 16ms, prescaler // 111 --> 1024 */
#define TIM2_overflow_16ms()     TCCR2B |= ((1<<CS20) | (1<<CS21) | (1<<CS22));

/** @brief Enable overflow interrupt, 1 --> enable */
#define TIM1_overflow_interrupt_enable()  TIMSK2 |= (1<<TOIE2);
/** @brief Disable overflow interrupt, 0 --> disable */
#define TIM1_overflow_interrupt_disable() TIMSK2 &= ~(1<<TOIE2);
```
## difference between Normal mode and CTC mode:
- the OCR0A Register is used for the CTC mode but the normal mode don't use special register
- The CTC mode allows greater control of the compare match output frequency
- There are no special cases to consider in the Normal mode, but in the CTC mode you can generate a interrupt
## difference between Fast PMW mode and Phase correct PMW mode:

- The fast PMW mode provides a high frequency PWM waveform generation option but the phase correct mode allows a high resolution phase correct PWM waveform
generation option, so the difference is between frequency / resolution



# Lab 4: LACROIX LUCAS

### Stopwatch

1. Draw a flowchart for `TIMER2_OVF_vect` interrupt service routine which overflows every 16&nbsp;ms but it updates the stopwatch LCD screen approximately every 100&nbsp;ms (6 x 16&nbsp;ms = 100&nbsp;ms). Display tenths of a second, seconds, and minutes and let the stopwatch counts from `00:00.0` to `59:59.9` and then starts again. The image can be drawn on a computer or by hand. Use clear descriptions of the individual steps of the algorithms.

   ![]()![image bonne](https://user-images.githubusercontent.com/114081959/197808748-9cb9e020-b266-45ed-bfc6-b8ccf68426b9.jpg)

## The 3 buttons corresponds to the "start button", the "+1 button" and finally the "-1 button"
### Kitchen alarm

2. Draw a schematic of the kitchen alarm application that counts down the time with an LCD, one LED and three push buttons: start, +1 minute, -1 minute. Use the +1/-1 minute buttons to increment/decrement the timer value. After pressing the Start button, the countdown starts. The countdown value is shown on the display in the form of mm.ss (minutes.seconds). At the end of the countdown, the LED will start blinking. The image can be drawn on a computer or by hand. Always name all components and their values.

   !![]()<img width="608" alt="Capture_d_écran_LB4_1" src="https://user-images.githubusercontent.com/114081959/197805744-fba501d9-6ad4-49c6-b7f4-55d588c859e7.png">
