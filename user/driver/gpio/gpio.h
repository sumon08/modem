


#ifndef GPIO_INCLUDE_H
#define GPIO_INCLUDE_H


#include <stdint.h>
#include <stdbool.h>

typedef void * Pin;


#define PORTA 0x00
#define PORTB 0x01
#define PORTC 0x02
#define PORTD 0x03
#define PORTE 0x04
#define PORTF 0x05
#define PORTG 0x06

#define PIN_0 0x0001
#define PIN_1 0x0002
#define PIN_2 0x0004
#define PIN_3 0x0008
#define PIN_4 0x0010
#define PIN_5 0x0020
#define PIN_6 0x0040
#define PIN_7 0x0080
#define PIN_8 0x0100
#define PIN_9 0x0200
#define PIN_10 0x0400
#define PIN_11 0x0800
#define PIN_12 0x1000
#define PIN_13 0x2000
#define PIN_14 0x4000
#define PIN_15 0x8000
#define PIN_ALL 0xFFFF

#define MODE_INPUT  0x01
#define MODE_OUTPUT 0x02

Pin PinCreate(uint8_t port, uint16_t pin, uint8_t mode);
void PinWrite(Pin pin, bool val);
bool PinRead(Pin pin);

#endif // GPIO_INCLUDE_H