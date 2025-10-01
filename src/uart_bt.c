#include <stdio.h>
#include <malloc.h>
#include <string.h>
#include <stdarg.h>

#include "stm32f4xx_ll_bus.h"
#include "stm32f4xx_ll_gpio.h"
#include "stm32f4xx_ll_usart.h"
#include "stm32f4xx_hal.h" 

#include "cmsis_os2.h"

#include "uart_bt.h"
#include "uart_config.h"

/* USART1 instance is used. (TX on PA.09, RX on PA.10)
   Connected to Bluetooth HM-10 */
#define USARTx_INSTANCE USART1
#define USARTx_CLK_ENABLE() LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_USART1)
#define USARTx_IRQn USART1_IRQn
#define USARTx_IRQHandler USART1_IRQHandler

#define USARTx_GPIO_CLK_ENABLE() LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA)
#define USARTx_TX_PIN LL_GPIO_PIN_9
#define USARTx_TX_GPIO_PORT GPIOA
#define USARTx_SET_TX_GPIO_AF() LL_GPIO_SetAFPin_8_15(GPIOA, LL_GPIO_PIN_9, LL_GPIO_AF_7)
#define USARTx_RX_PIN LL_GPIO_PIN_10
#define USARTx_RX_GPIO_PORT GPIOA
#define USARTx_SET_RX_GPIO_AF() LL_GPIO_SetAFPin_8_15(GPIOA, LL_GPIO_PIN_10, LL_GPIO_AF_7)

#define APB_Div 1

//#define UART_BAUD_RATE 115200// HM-10 default
#define UART_BAUD_RATE 9600// HM-10 default

static osMessageQueueId_t _uart_bt_tx_queue_id;
static osMessageQueueId_t _uart_bt_rx_queue_id;

static osMutexId_t _uart_bt_write_mutex_id;
static osMutexAttr_t _uart_bt_write_mutex_attr = {
    .name = "btWriteMutex",
    .attr_bits = 0,
    .cb_mem = NULL,
    .cb_size = 0};

static uint8_t _uart_bt_is_init = 0;

void uart_bt_init(void)
{
    if (!_uart_bt_is_init)
    {
        setvbuf(stdin, NULL, _IONBF, 0);
        setvbuf(stdout, NULL, _IONBF, 0);
        setvbuf(stderr, NULL, _IONBF, 0);

        USARTx_GPIO_CLK_ENABLE();

        LL_GPIO_SetPinMode(USARTx_TX_GPIO_PORT, USARTx_TX_PIN, LL_GPIO_MODE_ALTERNATE);
        USARTx_SET_TX_GPIO_AF();
        LL_GPIO_SetPinSpeed(USARTx_TX_GPIO_PORT, USARTx_TX_PIN, LL_GPIO_SPEED_FREQ_HIGH);
        LL_GPIO_SetPinOutputType(USARTx_TX_GPIO_PORT, USARTx_TX_PIN, LL_GPIO_OUTPUT_PUSHPULL);
        LL_GPIO_SetPinPull(USARTx_TX_GPIO_PORT, USARTx_TX_PIN, LL_GPIO_PULL_UP);

        LL_GPIO_SetPinMode(USARTx_RX_GPIO_PORT, USARTx_RX_PIN, LL_GPIO_MODE_ALTERNATE);
        USARTx_SET_RX_GPIO_AF();
        LL_GPIO_SetPinSpeed(USARTx_RX_GPIO_PORT, USARTx_RX_PIN, LL_GPIO_SPEED_FREQ_HIGH);
        LL_GPIO_SetPinOutputType(USARTx_RX_GPIO_PORT, USARTx_RX_PIN, LL_GPIO_OUTPUT_PUSHPULL);
        LL_GPIO_SetPinPull(USARTx_RX_GPIO_PORT, USARTx_RX_PIN, LL_GPIO_PULL_UP);

        NVIC_SetPriority(USARTx_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 6, 0));
        NVIC_EnableIRQ(USARTx_IRQn);

        USARTx_CLK_ENABLE();

        LL_USART_SetTransferDirection(USARTx_INSTANCE, LL_USART_DIRECTION_TX_RX);
        LL_USART_ConfigCharacter(USARTx_INSTANCE, LL_USART_DATAWIDTH_8B, LL_USART_PARITY_NONE, LL_USART_STOPBITS_1);

        LL_USART_SetBaudRate(USARTx_INSTANCE, SystemCoreClock / APB_Div, LL_USART_OVERSAMPLING_16, UART_BAUD_RATE);

        LL_USART_Enable(USARTx_INSTANCE);

        LL_USART_ClearFlag_ORE(USARTx_INSTANCE);

        LL_USART_EnableIT_RXNE(USARTx_INSTANCE);
        LL_USART_EnableIT_ERROR(USARTx_INSTANCE);

        _uart_bt_tx_queue_id = osMessageQueueNew(256, sizeof(uint8_t), NULL);
        _uart_bt_rx_queue_id = osMessageQueueNew(64, sizeof(uint8_t), NULL);

        _uart_bt_write_mutex_id = osMutexNew(&_uart_bt_write_mutex_attr);

        _uart_bt_is_init = 1;
    }
}

int __wrap_printf_bt(char *fmt, ...)
{
    int result = 0;
    if (osMutexAcquire(_uart_bt_write_mutex_id, osWaitForever) == osOK)
    {
        va_list args;
        va_start(args, fmt);
        result = vprintf(fmt, args);
        va_end(args);
        osMutexRelease(_uart_bt_write_mutex_id);
    }
    return result;
}

#ifdef USE_BT
int __io_putchar(int c)
{
    osMessageQueuePut(_uart_bt_tx_queue_id, &c, 0U, osWaitForever);
    LL_USART_EnableIT_TXE(USARTx_INSTANCE);
    return c;
}

int __io_getchar(void)
{
    static uint8_t c;
    osMessageQueueGet(_uart_bt_rx_queue_id, &c, NULL, osWaitForever);
    return c;
}
#endif

void USARTx_IRQHandler(void)
{
    if (LL_USART_IsActiveFlag_RXNE(USARTx_INSTANCE) && LL_USART_IsEnabledIT_RXNE(USARTx_INSTANCE))
    {
        _uart_bt_rxne_isr();
    }

    if (LL_USART_IsEnabledIT_TXE(USARTx_INSTANCE) && LL_USART_IsActiveFlag_TXE(USARTx_INSTANCE))
    {
        _uart_bt_txe_isr();
    }

    if (LL_USART_IsEnabledIT_ERROR(USARTx_INSTANCE))
    {
        _uart_bt_error();
    }
}

void _uart_bt_rxne_isr(void)
{
    uint8_t c = LL_USART_ReceiveData8(USARTx_INSTANCE);
    osMessageQueuePut(_uart_bt_rx_queue_id, &c, 0U, 0);
}

void _uart_bt_txe_isr(void)
{
    static uint8_t c;
    osStatus_t evt = osMessageQueueGet(_uart_bt_tx_queue_id, &c, NULL, 0);
    switch (evt)
    {
    case osOK:
        LL_USART_TransmitData8(USARTx_INSTANCE, c);
        break;
    case osErrorTimeout:
    case osErrorResource:
        LL_USART_DisableIT_TXE(USARTx_INSTANCE);
        break;
    default:
        break;
    }
}

void _uart_bt_error(void)
{
    __IO uint32_t sr_reg;
    sr_reg = LL_USART_ReadReg(USARTx_INSTANCE, SR);

    if (sr_reg & LL_USART_SR_PE)
    {
        printf("*** USART1 parity error! ***\n");
    }
    if (sr_reg & LL_USART_SR_FE)
    {
        printf("*** USART1 frame error! ***\n");
    }
    if (sr_reg & LL_USART_SR_ORE)
    {
        printf("*** USART1 overrun error! ***\n");
    }
    if (sr_reg & LL_USART_SR_NE)
    {
        printf("*** USART1 noise error! ***\n");
    }
}
