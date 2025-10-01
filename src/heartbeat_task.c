#include "stm32f4xx_hal.h"
#include "cmsis_os2.h"

#include "heartbeat_task.h"

#define PERIOD 10000
static osThreadId_t _heartbeatThreadID;
static osThreadAttr_t _heartbeatThreadAttr = {
    .name = "heartbeat",
    .priority = osPriorityIdle
};

static void _heartbeat_update(void *arg);

static uint8_t _is_running = 0;
static uint8_t _is_init = 0;

static TIM_HandleTypeDef _htim2;

void heartbeat_task_init(void)
{
    if (!_is_init)
    {
        _htim2.Instance = TIM2;
        _htim2.Init.Prescaler = 0;
        _htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
        _htim2.Init.Period = 9999;
        _htim2.Init.ClockDivision = 0;

        TIM_OC_InitTypeDef _sConfigPWM;
        _sConfigPWM.OCMode = TIM_OCMODE_PWM1;
        _sConfigPWM.Pulse = 0;
        _sConfigPWM.OCPolarity = TIM_OCPOLARITY_HIGH;
        _sConfigPWM.OCFastMode = TIM_OCFAST_DISABLE;

        HAL_TIM_PWM_Init(&_htim2);
        HAL_TIM_PWM_ConfigChannel(&_htim2, &_sConfigPWM, TIM_CHANNEL_1);

        /* Enable clock for Timer2 */
        HAL_TIM_PWM_Start(&_htim2, TIM_CHANNEL_1);

        _heartbeatThreadID = osThreadNew(_heartbeat_update, NULL, &_heartbeatThreadAttr);

        heartbeat_task_start();
        _is_init = 1;
    }
}

void heartbeat_task_start(void)
{
    if (!_is_running)
    {
        /* Start the heartbeat osTimer running at 10ms*/
        osTimerStart(_heartbeatThreadID, PERIOD);
        _is_running = 1;
    }
}

void heartbeat_task_stop(void)
{
    if (_is_running)
    {
        /* Stop the heartbeat osTimer */
        osTimerStop(_heartbeatThreadID);
        _is_running = 0;
    }
}

uint8_t heartbeat_task_is_running(void)
{
    return _is_running;
}

void _heartbeat_update(void *arg)
{
    UNUSED(arg);
    while (1)
    {
        for (int i = 0; i <= 200; i += 1)
        {
            __HAL_TIM_SET_COMPARE(&_htim2, TIM_CHANNEL_1, i * 50);
            osDelay(1);
        }
        for (int i = 200; i >= 0; i -= 2)
        {
            __HAL_TIM_SET_COMPARE(&_htim2, TIM_CHANNEL_1, i * 50);
            osDelay(1);
        }
        for (int i = 0; i <= 200; i += 1)
        {
            __HAL_TIM_SET_COMPARE(&_htim2, TIM_CHANNEL_1, i * 50);
            osDelay(1);
        }
        for (int i = 200; i >= 0; i -= 1)
        {
            __HAL_TIM_SET_COMPARE(&_htim2, TIM_CHANNEL_1, i * 50);
            osDelay(1);
        }
        osDelay(256);
    }
}

void heartbeat_task_deinit(void) // Used for `make test'
{
    _is_init = 0;
    _is_running = 0;
}