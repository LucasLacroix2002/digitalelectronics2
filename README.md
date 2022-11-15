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


### Kitchen alarm

2. Draw a schematic of the kitchen alarm application that counts down the time with an LCD, one LED and three push buttons: start, +1 minute, -1 minute. Use the +1/-1 minute buttons to increment/decrement the timer value. After pressing the Start button, the countdown starts. The countdown value is shown on the display in the form of mm.ss (minutes.seconds). At the end of the countdown, the LED will start blinking. The image can be drawn on a computer or by hand. Always name all components and their values.

   ![]()<img width="608" alt="Capture_d_écran_LB4_1" src="https://user-images.githubusercontent.com/114081959/197805744-fba501d9-6ad4-49c6-b7f4-55d588c859e7.png">

The 3 buttons corresponds to the "start button", the "+1 button" and finally the "-1 button"

# Lab 5: INSERT_YOUR_FIRSTNAME INSERT_YOUR_LASTNAME

### Analog-to-Digital Conversion

1. Complete table with voltage divider, calculated, and measured ADC values for all five push buttons.

   | **Push button** | **PC0 voltage** | **ADC value (calculated)** | **ADC value (measured)** | **ADC value (measured, hex)** |
   | :-: | :-: | :-: | :-: | :-: |
   | Right  | 0&nbsp;V | 0   | 0 | 0 | 000
   | Up     | 0.495&nbsp;V | 101 | 143 | 072 |
   | Down   | 1.203&nbsp;V | 246 | 342 | 143 |
   | Left   | 2,5V | 512 | 511 | 1F4 |
   | Select | 3,15V | 645 | 731 | 269 |
   | none   | 5V | 1023 | 1023 | 3FF |

### Temperature meter

Consider an application for temperature measurement. Use analog temperature sensor [TC1046](http://ww1.microchip.com/downloads/en/DeviceDoc/21496C.pdf), LCD, and a LED. Every 30 seconds, the temperature is measured and the value is displayed on LCD screen. When the temperature is too high, the LED will turn on.

2. Draw a schematic of temperature meter. The image can be drawn on a computer or by hand. Always name all components, their values and pin names!

![IMG_6318](https://user-images.githubusercontent.com/114081959/199350893-624f5ef7-91c8-4a21-a8d1-68ebff14734e.jpg)

3. Draw two flowcharts for interrupt handler `TIMER1_OVF_vect` (which overflows every 1&nbsp;sec) and `ADC_vect`. The image can be drawn on a computer or by hand. Use clear descriptions of individual algorithm steps.

   
   ![IMG_6319](https://user-images.githubusercontent.com/114081959/199350936-9a845f3a-3de8-4b80-813d-39040b411bb0.jpg)


# Lab 6: Lacroix Lucas

### ASCII

1. Complete the table with selected ASCII codes.

   | **Char** | **Decimal** | **Hexadecimal** | **Binary** |
   | :-: | :-: | :-: | :-: |
   | `a` | 97 | 0x61 | `0b0110_0001` |
   | `b` | 98| 0x62 | 1100010 |
   | `c` | 99 | 0x63 | 1100011 |
   | `0` | 48 | 0x30 | `0b0011_0000` |
   | `1` | 149 | 0x31 | 110001 |
   | `2` | 150 | 0x32 | 110010 |
   | `Esc` | 27 | 0x1b | 11011 |
   | `Space` | 32 | 0x20 | 100000 |
   | `Tab` | 9 | 0x9 | 1001 |
   | `Backspace` | 8 | 0x8 | 1000 |
   | `Enter` | 10 | 0xa | 1010 |

### UART communication

2. Draw a timing diagram of the output from UART/USART when transmitting three character data `De2` in 4800 7O2 mode (7 data bits, odd parity, 2 stop bits, 4800&nbsp;Bd). The image can be drawn on a computer (by [WaveDrom](https://wavedrom.com/) for example) or by hand. Name all parts of timing diagram.

   ![IMG_6371](https://user-images.githubusercontent.com/114081959/200691003-56a395a9-dba3-4209-8a7d-082c34ccf1ae.jpg)


3. Draw a flowchart for function `uint8_t get_parity(uint8_t data, uint8_t type)` which calculates a parity bit of input 8-bit `data` according to parameter `type`. The image can be drawn on a computer or by hand. Use clear description of individual algorithm steps.

   ![IMG_6372](https://user-images.githubusercontent.com/114081959/200690785-c4aa663e-11fe-43d9-ab1f-1fb25d221a11.jpg)



# Lab 7: LACROIX Lucas

### Arduino Uno pinout

1. In the picture of the Arduino Uno board, mark the pins that can be used for the following functions/operations:
   * PWM generators from Timer0, Timer1, Timer2
   * analog channels for ADC
   * UART pins
   * I2C pins
   * SPI pins
   * external interrupt pins INT0, INT1

   ![IMG_6480](https://user-images.githubusercontent.com/114081959/202033836-6af6eed5-c3cf-46e5-abbe-cfe326f8ab93.JPG)()

### I2C communication

2. Draw a timing diagram of I2C signals when calling function `rtc_read_years()`. Let this function reads one byte-value from RTC DS3231 address `06h` (see RTC datasheet) in the range `00` to `99`. Specify when the SDA line is controlled by the Master device and when by the Slave device. Draw the whole request/receive process, from Start to Stop condition. The image can be drawn on a computer (by [WaveDrom](https://wavedrom.com/) for example) or by hand. Name all parts of timing diagram.

  ![IMG_6482](https://user-images.githubusercontent.com/114081959/202038955-cec35b81-4140-4ec8-9ba5-80c5329bc068.JPG)()

### Meteo station

Consider an application for temperature and humidity measurements. Use sensor DHT12, real time clock DS3231, LCD, and one LED. Every minute, the temperature, humidity, and time is requested from Slave devices and values are displayed on LCD screen. When the temperature is above the threshold, turn on the LED.

3. Draw a flowchart of `TIMER1_OVF_vect` (which overflows every 1&nbsp;sec) for such Meteo station. The image can be drawn on a computer or by hand. Use clear description of individual algorithm steps.

   
   ![IMG_6481](https://user-images.githubusercontent.com/114081959/202033964-df08f47c-e0a4-4fa6-9260-b5c831ba2fad.JPG)()
