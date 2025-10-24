#include "stm32f4xx_hal.h"
#include "stdlib.h"

#include "mod_enc.h"

#define CW 1
#define CCW 2

static const uint8_t state_table[4][4] = {
    // Previous -> Current State
    // 00   01   10   11  <- Current State
    {0, CW, CCW, 0}, // 00 Previous State
    {CCW, 0, 0, CW}, // 01
    {CW, 0, 0, CCW}, // 10
    {0, CCW, CW, 0}  // 11
};

static volatile int32_t encA_count = 0;
static volatile int32_t encB_count = 0;

static volatile uint8_t new_stateA = 0;
static volatile uint8_t new_stateB = 0;

static volatile uint8_t directionA = 0;
static volatile uint8_t directionB = 0;

static volatile uint8_t old_stateA = 0;
static volatile uint8_t old_stateB = 0;

int32_t mod_enc_get_countA(void)
{
    return encA_count;
}

int32_t mod_enc_get_countB(void)
{
    return encB_count;
}

void mod_enc_reset_countA(void)
{
    encA_count = 0;
}

void mod_enc_reset_countB(void)
{
    encB_count = 0;
}

// Configure GPIO pins and interrupts for encoders
void mod_enc_configure_hardware(void)
{

    // ENable clock B/C
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    // init PC0, PC1
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;

    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    // init PB4, PB5
    GPIO_InitStruct.Pin = GPIO_PIN_4 | GPIO_PIN_5;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;

    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    // Set priority of external interrupt lines 0, 1
    HAL_NVIC_SetPriority(EXTI0_IRQn, 0xFF, 0xFF);
    HAL_NVIC_SetPriority(EXTI1_IRQn, 0xFF, 0xFF);
    HAL_NVIC_SetPriority(EXTI4_IRQn, 0xFF, 0xFF);
    HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0xFF, 0xFF);

    // Enable extern interrupt
    HAL_NVIC_EnableIRQ(EXTI0_IRQn);
    HAL_NVIC_EnableIRQ(EXTI1_IRQn);
    HAL_NVIC_EnableIRQ(EXTI4_IRQn);
    HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
}

/******************************************
*********** ENCODER A CALLBACKS ***********
******************************************/
void handle_encoderA_interrupt(void);
void handle_encoderA_interrupt()
{
    // Read current state of both pins
    new_stateA = ((GPIOB->IDR & GPIO_PIN_4) ? 2 : 0) |
                 ((GPIOB->IDR & GPIO_PIN_5) ? 1 : 0);

    // Look up direction from state transition table
    directionA = state_table[old_stateA][new_stateA];

    // Update count based on valid state transition
    if (directionA == CW)
    {
        encA_count++;
    }
    else if (directionA == CCW)
    {
        encA_count--;
    }
    // Invalid transitions are ignored

    old_stateA = new_stateA;
}

void EXTI4_IRQHandler(void)
{
    handle_encoderA_interrupt();
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_4);
}

void EXTI9_5_IRQHandler(void)
{
    handle_encoderA_interrupt();
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_5);
}

/******************************************
*********** ENCODER B CALLBACKS ***********
******************************************/
void handle_encoderB_interrupt(void);
void handle_encoderB_interrupt()
{
    // Read current state of both pins
    new_stateB = ((GPIOC->IDR & GPIO_PIN_0) ? 2 : 0) |
                 ((GPIOC->IDR & GPIO_PIN_1) ? 1 : 0);

    // Look up direction from state transition table
    old_stateB = state_table[old_stateB][new_stateB];

    // Update count based on valid state transition
    if (old_stateB == CW)
    {
        encB_count++;
    }
    else if (old_stateB == CCW)
    {
        encB_count--;
    }

    old_stateB = new_stateB;
}

void EXTI0_IRQHandler(void)
{
    handle_encoderB_interrupt();
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_0);
}

void EXTI1_IRQHandler(void)
{
    handle_encoderB_interrupt();
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_1);
}