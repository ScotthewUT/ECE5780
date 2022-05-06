#include "main.h"
#include "stm32f0xx_it.h"

volatile uint16_t inc_cnt = 0;         // For timing user button


/******************************************************************************/
/*           Cortex-M0 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
    while (1) {
    }
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
    while (1) {
    }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
    
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
    
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
    HAL_IncTick();
}


/******************************************************************************/
/* STM32F0xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f0xx.s).                    */
/******************************************************************************/

/**
  * @brief TIM2 interrupt which handles updating target RPM via button press
  */
void TIM2_IRQHandler(void)
{
    TIM2->SR &= ~(TIM_SR_UIF);     // Clear update interrupt flag
    
    if (GPIOA->IDR & GPIO_IDR_0) {
        inc_cnt++;
        if (inc_cnt > 1200) {      // Increase target RPM every 3 sec (1200 * 2.5ms)
            inc_cnt = 0;
            switch (target_rpm) {
                case 0:
                    target_rpm = 10;
                    PWM_setDutyCycle(60);
                    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_9);
                    break;
                case 10:
                    target_rpm = 24;
                    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_7);
                    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_9);
                    break;
                case 24:
                    target_rpm = 38;
                    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_8);
                    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_7);
                    break;
                case 38:
                    target_rpm = 52;
                    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_6);
                    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_8); 
                    break;
                case 52:
                    target_rpm = 66;
                    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_7);
                    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_8);
                    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_9);
            }
        }
    } else {
        inc_cnt = 0;
        target_rpm = 0;
        PWM_setDutyCycle(0);
        GPIOC->BSRR |= (GPIO_BSRR_BR_6 | GPIO_BSRR_BR_7 | GPIO_BSRR_BR_8 | GPIO_BSRR_BR_9);
    }     
}


/**
  * @brief TIM6 interrupt to calculate motor speed and call motor controller
  */
void TIM6_DAC_IRQHandler(void) {
    TIM6->SR &= ~(TIM_SR_UIF);             // Clear update interrupt flag
    motor_speed = (TIM3->CNT - 0x7FFF);    // Get latest encoder count
    motor_rpm = motor_speed * 2;           // TIM6->PSC & ARR were picked such that RPMs are double encoder pulses
    TIM3->CNT = 0x7FFF;                    // Reset motor encoder count
    motor_speedController();               // Ask controller to manage speed
}


/**
  * @brief TIM7 interrupt which handles coast detection and solenoid activation
  */
void TIM7_IRQHandler(void)
{
    TIM7->SR &= ~(TIM_SR_UIF);             // Clear update interrupt flag
    
    if (target_rpm == 0 && motor_rpm > 0) {
        GPIOA->ODR |= GPIO_ODR_8;          // Set PA8 (solenoid transistor) high
        GPIOC->ODR |= GPIO_ODR_6;          // Enable red LED
    } else {
        GPIOA->ODR &= ~(GPIO_ODR_8);       // Ground PA8; de-energize solenoid
    }
}
