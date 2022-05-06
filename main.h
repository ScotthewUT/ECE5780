#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>
#include <stdlib.h>
#include "stm32f0xx_hal.h"


void button_init(void);                    // Initializes the USER button
void encoder_init(void);                   // Initializes motor encoder interface
void LED_init(void);                       // Initializes the four board LEDs
void motor_speedController(void);          // Updates motor speed
void print_msg(char[], unsigned int);      // Displays char array on terminal
void PWM_init(void);                       // Initializes PWM; motor enable on PA4
void PWM_setDutyCycle(uint8_t duty);       // Sets duty-cycle of PWM
void serial_Tx_init(void);                 // Initializes USART3 for printing to terminal
void switch_init(void);                    // Initializes PA8 as output to drive switch
void timer_init(void);                     // Initializes TIM2 & TIM7 timers w/ interrupts

extern volatile uint8_t duty_cycle;        // Output PWM duty cycle
extern volatile int16_t error;             // Speed error signal
extern volatile int16_t motor_rpm;         // Measured motor speed in RPM
extern volatile int16_t motor_speed;       // Measured motor speed in encoder counts
extern volatile int16_t target_rpm;        // Desired speed target in RPM


#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
