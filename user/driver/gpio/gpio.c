

#include "FreeRTOS.h"
#include "gpio.h"

#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"

#ifndef NULL
#define NULL 0
#endif

typedef struct
{
    GPIO_TypeDef *port;
    uint16_t pin;
    uint8_t mode;
} Pin_t;

GPIO_TypeDef *EnumeratPort(uint8_t port)
{
    GPIO_TypeDef *ret = NULL;
    switch (port)
    {
    case PORTA:
        ret = GPIOA;
        break;
    case PORTB:
        ret = GPIOB;
        break;
    case PORTC:
        ret = GPIOC;
        break;
    case PORTD:
        ret = GPIOD;
        break;
    case PORTE:
        ret = GPIOE;
        break;
    case PORTF:
        ret = GPIOF;
        break;
    default:
        break;
    }
    return ret;
}

void GPIOEnableClock(uint8_t port)
{
    switch (port)
    {
    case PORTA:
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
        break;
    case PORTB:
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
        break;
    case PORTC:
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
        break;
    case PORTD:
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);
        break;
    case PORTE:
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE);
        break;
    case PORTF:
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOF, ENABLE);
        break;
    default:
        break;
    }
}

Pin PinCreate(uint8_t port, uint16_t pin, uint8_t mode)
{
    Pin_t *p = pvPortMalloc(sizeof(Pin_t));
    p->mode = mode;
    p->pin = pin;
    p->port = EnumeratPort(port);

    GPIO_InitTypeDef gpio;
    gpio.GPIO_Mode = (mode == MODE_OUTPUT ? GPIO_Mode_Out_PP : GPIO_Mode_IN_FLOATING);
    gpio.GPIO_Pin = pin;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;

    GPIOEnableClock(port);

    GPIO_Init(p->port, &gpio);

    return p;
}
void PinWrite(Pin pin, bool val)
{
    Pin_t *p = (Pin_t *)pin;

    if (p->mode == MODE_OUTPUT)
    {
        GPIO_WriteBit(p->port, p->pin, val == true ? Bit_SET : Bit_RESET);
    }
}
bool PinRead(Pin pin)
{
    Pin_t *p = (Pin_t *)pin;
    bool ret = false;
    if(p->mode == MODE_INPUT)
        ret = (GPIO_ReadInputDataBit(p->port, p->pin) == Bit_SET? true : false);
    return ret;
}