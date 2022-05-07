#include "main.h"


volatile uint8_t duty_cycle = 0;       // Output PWM duty cycle
volatile int16_t error = 0;            // Speed error signal
volatile int16_t motor_rpm = 0;        // Measured motor speed in RPM
volatile int16_t motor_speed = 0;      // Measured motor speed in encoder counts
volatile int16_t target_rpm = 0;       // Desired speed target in RPM


/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
    HAL_Init();
    button_init();
    encoder_init();
    LED_init();
    PWM_init();
    serial_Tx_init();
    switch_init();
    timer_init();
    
    uint32_t debug_counter = 0;    // Used to periodically display info on console
    
    TIM2->CR1 |= TIM_CR1_CEN;      // Enable TIM2 counter
    TIM7->CR1 |= TIM_CR1_CEN;      // Enable TIM7 counter
    
    // Main application loop
    while (1) {
        debug_counter++;
        // Did you know 97879 is the 111th palindromic prime?
        if (debug_counter % 97879 == 111) {
            char msg[128];
            sprintf(msg, " Setpoint: %i  |  Speed: %i  |  Duty-Cycle: %i \n\r\n\r", target_rpm, motor_rpm, duty_cycle);
            unsigned int size = sizeof(msg) / sizeof(msg[0]);
            print_msg(msg, size);
        }
    }
}


/**
  * @brief  Initializes the USER button (PA0) on the Discovery board
  * @retval none
  */
void button_init(void) {
    // Enable the peripheral clock for GPIO port A
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
    // Set button to input mode (00), low speed (x0), & with pull-down resistor (10)
    GPIOA->MODER &= ~(GPIO_MODER_MODER0_0 | GPIO_MODER_MODER0_1); 
    GPIOA->OSPEEDR &= ~GPIO_OSPEEDER_OSPEEDR0_0;
    GPIOA->PUPDR &= ~GPIO_PUPDR_PUPDR0_0;
    GPIOA->PUPDR |= GPIO_PUPDR_PUPDR0_1;
}


/**
  * @brief  Sets up encoder interface to read motor speed
  * @retval none
  * @author Provided by ECE 5780 faculty
  */
void encoder_init(void) {
    
    // Set up encoder input pins (TIMER 3 CH1 and CH2)
    RCC->AHBENR |= RCC_AHBENR_GPIOBEN;

    GPIOB->MODER &= ~(GPIO_MODER_MODER4_0 | GPIO_MODER_MODER5_0);
    GPIOB->MODER |= (GPIO_MODER_MODER4_1 | GPIO_MODER_MODER5_1);
    GPIOB->AFR[0] |= ( (1 << 16) | (1 << 20) );

    // Set up encoder interface (TIM3 encoder input mode)
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
    TIM3->CCMR1 = 0;
    TIM3->CCER = 0;
    TIM3->SMCR = 0;
    TIM3->CR1 = 0;

    TIM3->CCMR1 |= (TIM_CCMR1_CC1S_0 | TIM_CCMR1_CC2S_0);   // TI1FP1 and TI2FP2 signals connected to CH1 and CH2
    TIM3->SMCR |= (TIM_SMCR_SMS_1 | TIM_SMCR_SMS_0);        // Capture encoder on both rising and falling edges
    TIM3->ARR = 0xFFFF;                                     // Set ARR to top of timer (longest possible period)
    TIM3->CNT = 0x7FFF;                                     // Bias at midpoint to allow for negative rotation
    // (Could also cast unsigned register to signed number to get negative numbers if it rotates backwards past zero
    //  just another option, the mid-bias is a bit simpler to understand though.)
    TIM3->CR1 |= TIM_CR1_CEN;                               // Enable timer

    // Configure a second timer (TIM6) to fire an ISR on update event
    // Used to periodically check and update speed variable
    RCC->APB1ENR |= RCC_APB1ENR_TIM6EN;
    
    // Select PSC and ARR values that give an appropriate interrupt rate
    TIM6->PSC = 111;     //  8 MHz / 112 = 71.4 kHz; 1/71.4 kHz = 14 us                  -- Default = 11
    TIM6->ARR = 1313;    // 1313 * 14 us = 18.4  ms; 1/18.4  ms = 54.4 Hz;  +/- 2.0 RPM  -- Default = 30000
    
    TIM6->DIER |= TIM_DIER_UIE;             // Enable update event interrupt
    TIM6->CR1 |= TIM_CR1_CEN;               // Enable Timer

    NVIC_EnableIRQ(TIM6_DAC_IRQn);          // Enable interrupt in NVIC
    NVIC_SetPriority(TIM6_DAC_IRQn,2);
}


/**
  * @brief  Initializes the four LEDs on the Discovery board
  * @retval none
  */
void LED_init(void) {
    // Enable the peripheral clock for GPIO port C
    RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
    // Set the four LEDs (PC6-9) to general purpose output mode (01)
    GPIOC->MODER |= GPIO_MODER_MODER6_0;
    GPIOC->MODER &= ~GPIO_MODER_MODER6_1;
    GPIOC->MODER |= GPIO_MODER_MODER7_0;
    GPIOC->MODER &= ~GPIO_MODER_MODER7_1;
    GPIOC->MODER |= GPIO_MODER_MODER8_0;
    GPIOC->MODER &= ~GPIO_MODER_MODER8_1;
    GPIOC->MODER |= GPIO_MODER_MODER9_0;
    GPIOC->MODER &= ~GPIO_MODER_MODER9_1;
    // Set LEDs to push-pull type (0)
    GPIOC->OTYPER &= ~GPIO_OTYPER_OT_6;
    GPIOC->OTYPER &= ~GPIO_OTYPER_OT_7;
    GPIOC->OTYPER &= ~GPIO_OTYPER_OT_8;
    GPIOC->OTYPER &= ~GPIO_OTYPER_OT_9;
    // Set LEDs to low speed (x0)
    GPIOC->OSPEEDR &= ~GPIO_OSPEEDER_OSPEEDR6_0;
    GPIOC->OSPEEDR &= ~GPIO_OSPEEDER_OSPEEDR7_0;
    GPIOC->OSPEEDR &= ~GPIO_OSPEEDER_OSPEEDR8_0;
    GPIOC->OSPEEDR &= ~GPIO_OSPEEDER_OSPEEDR9_0;
    // Disable pull-up/down resistors (00)
    GPIOC->PUPDR &= ~GPIO_PUPDR_PUPDR6_0;
    GPIOC->PUPDR &= ~GPIO_PUPDR_PUPDR6_1;
    GPIOC->PUPDR &= ~GPIO_PUPDR_PUPDR7_0;
    GPIOC->PUPDR &= ~GPIO_PUPDR_PUPDR7_1;
    GPIOC->PUPDR &= ~GPIO_PUPDR_PUPDR8_0;
    GPIOC->PUPDR &= ~GPIO_PUPDR_PUPDR8_1;
    GPIOC->PUPDR &= ~GPIO_PUPDR_PUPDR9_0;
    GPIOC->PUPDR &= ~GPIO_PUPDR_PUPDR9_1;
    // Disable/reset all 4 LEDs
    GPIOC->BSRR |= (GPIO_BSRR_BR_6 | GPIO_BSRR_BR_7 | GPIO_BSRR_BR_8 | GPIO_BSRR_BR_9);
}


/**
  * @brief  Controls motor speed based off setpoint and encoder readings
  * @retval none
  */
void motor_speedController(void) {
    error = target_rpm - motor_rpm;    // Calculate # of RPMs below setpoint
    if (abs(error) < 3) {
        return;                        // Within 3 RPMs of target is good enough
    }
    if (error > 0) {
        duty_cycle = duty_cycle + 4;   // Increase duty-cycle if below setpoint
    } else {
        duty_cycle = duty_cycle - 4;   // Decrease duty-cycle if above setpoint
    }
    if (duty_cycle > 100) {
        duty_cycle = 100;              // Maximum is 100%
    }
    else if (duty_cycle < 60) {
        duty_cycle = 60;               // At 6 volts, duty-cycles below 60 won't turn my motor
    }
    if (target_rpm == 0) {             // De-energize motor when setpoint is 0
        duty_cycle = 0;
	}
    
    PWM_setDutyCycle(duty_cycle);      // Update the duty-cycle signal to the motor
}


/**
  * @brief  Prints character array to terminal
  * @retval none
  */
void print_msg(char msg[], unsigned int msgSize) {
    int idx;
    for (idx = 0; idx < msgSize; idx++)
    {
        while(!(USART3->ISR & USART_ISR_TXE));
        USART3->TDR = msg[idx];
    }
}


/**
  * @brief  Sets up the PWM and direction signals to drive the H-Bridge
  * @retval none
  * @author Provided by ECE 5780 faculty
  */
void PWM_init(void) {
    
    // Set up pin PA4 for H-bridge PWM output (TIMER 14 CH1)
    GPIOA->MODER |= (1 << 9);
    GPIOA->MODER &= ~(1 << 8);

    // Set PA4 to AF4,
    GPIOA->AFR[0] &= 0xFFF0FFFF; // clear PA4 bits,
    GPIOA->AFR[0] |= (1 << 18);

    // Set up a PA5, PA6 as GPIO output pins for motor direction control
    GPIOA->MODER &= 0xFFFFC3FF; // clear PA5, PA6 bits,
    GPIOA->MODER |= (1 << 10) | (1 << 12);
    
    //Initialize one direction pin to high, the other low
    GPIOA->ODR |= (1 << 5);
    GPIOA->ODR &= ~(1 << 6);

    // Set up PWM timer
    RCC->APB1ENR |= RCC_APB1ENR_TIM14EN;
    TIM14->CR1 = 0;                         // Clear control registers
    TIM14->CCMR1 = 0;                       // (prevents having to manually clear bits)
    TIM14->CCER = 0;

    // Set output-compare CH1 to PWM1 mode and enable CCR1 preload buffer
    TIM14->CCMR1 |= (TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1PE);
    TIM14->CCER |= TIM_CCER_CC1E;           // Enable capture-compare channel 1
    TIM14->PSC = 1;                         // Run timer on 24Mhz
    TIM14->ARR = 1200;                      // PWM at 20kHz
    TIM14->CCR1 = 0;                        // Start PWM at 0% duty cycle
    
    TIM14->CR1 |= TIM_CR1_CEN;              // Enable timer
}


/**
  * @brief  Set the duty-cycle of the PWM
  * @param  duty - Accepts ints 0-100
  * @retval none
  * @author Provided by ECE 5780 faculty
  */
void PWM_setDutyCycle(uint8_t duty) {
    if(duty <= 100) {
        TIM14->CCR1 = ((uint32_t)duty*TIM14->ARR)/100;  // Use linear transform to produce CCR1 value
        // (CCR1 == "pulse" parameter in PWM struct used by peripheral library)
    }
}


/**
  * @brief  Initializes USART3 to transmit on PC4 to terminal for debugging
  * @retval none
  */
void serial_Tx_init(void) {
    RCC->APB1ENR |= RCC_APB1ENR_USART3EN;             // Enable the peripheral clock for USART3
    GPIOC->MODER &= ~GPIO_MODER_MODER4_0;             // Set PC4 to alternate function mode (10)
    GPIOC->MODER |= GPIO_MODER_MODER4_1;
    GPIOC->AFR[0] |= (0x01 << GPIO_AFRL_AFSEL4_Pos);  // Select AF1 (USART3_TX on PC4
    USART3->BRR = HAL_RCC_GetHCLKFreq() / 38400;      // Set baud rate to 38400
    USART3->CR1 |= USART_CR1_TE;                      // Enable transmitter
    USART3->CR1 |= USART_CR1_UE;                      // Enable USART3
}


/**
  * @brief  Initializes GPIO port A pin 8 to control transistor switch
  * @retval none
  */
void switch_init(void) {
    // Enable the peripheral clock for GPIO port A
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
    // Set PA8 to general purpose output mode (01)
    GPIOA->MODER |= GPIO_MODER_MODER8_0;
    GPIOA->MODER &= ~GPIO_MODER_MODER8_1;
    // Set PA8 to low speed (x0)
    GPIOA->OSPEEDR &= ~GPIO_OSPEEDER_OSPEEDR8_0;
    // Set pull-down (10) resistor for PA8
    GPIOA->PUPDR &= ~GPIO_PUPDR_PUPDR0_0;
    GPIOA->PUPDR |= GPIO_PUPDR_PUPDR0_1;
}


/**
  * @brief  Initializes TIM2 at 400Hz and TIM7 at 4000Hz with interrupts
  * @retval none
  */
void timer_init(void) {
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;     // Enable the peripheral clock for TIM2
    TIM2->PSC   = 199;                      // 8MHz / 200 = 40kHz; 1/40kHz = 25us
    TIM2->ARR   = 100;                      // 25us * 100 = 2.5ms; 1/2.5ms = 400Hz
    TIM2->DIER |= TIM_DIER_UIE;             // Enable update interrupt
    NVIC_EnableIRQ(TIM2_IRQn);              // Enable interrupts from TIM2
    NVIC_SetPriority(TIM2_IRQn, 2);         // Set priority to medium
    
    RCC->APB1ENR |= RCC_APB1ENR_TIM7EN;     // Enable the peripheral clock for TIM7
    TIM7->PSC   = 39;                       // 8MHz / 40 = 200kHz; 1/200kHz = 5us
    TIM7->ARR   = 50;                       // 5us  * 50 = 250us ; 1/250us  = 4kHz
    TIM7->DIER |= TIM_DIER_UIE;             // Enable update interrupt
    NVIC_EnableIRQ(TIM7_IRQn);              // Enable interrupts from TIM7
    NVIC_SetPriority(TIM7_IRQn, 1);         // Set priority to high
}
