//
// Created by YangYongbao on 2017/3/16.
//

#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include "stm32f10x.h"
#include "stm32f10x_conf.h"
#include "FreeRTOS.h"
#include "task.h"
#include "gpio.h"
#include "usart.h"
#include "crc.h"

UsartHandle mdm_port;
UsartHandle mstr_port;
volatile Pin vbat;
volatile Pin pwr;
volatile Pin rst;

uint8_t packet_buffer[512];
uint8_t mstr_rx_buf[1600];
uint8_t rx_buff[1600];

void SendPacket(uint8_t ch, uint8_t *data, uint16_t len)
{
    packet_buffer[0] = 0xF9;
    packet_buffer[1] = ch;
    packet_buffer[2] = len >> 8;
    packet_buffer[3] = len >> 0;
    if (len > 0)
    {
        memcpy(&packet_buffer[4], data, len);
        packet_buffer[len + 4] = MakeCrc8(packet_buffer, len + 4);
        UsartSendString(mstr_port, (const char *)packet_buffer, len + 5);
    }
    else
    {
        packet_buffer[4] = MakeCrc8(packet_buffer, 4);
        UsartSendString(mstr_port, (const char *)packet_buffer, 5);
    }
}

void SendString(uint8_t ch, char *str)
{
    SendPacket(ch, (uint8_t *)str, strlen(str));
}

void SendInt32(uint8_t ch, uint8_t tock, uint32_t data)
{
    uint8_t buffer[5];
    buffer[0] = tock;
    buffer[1] = data >> 24;
    buffer[2] = data >> 16;
    buffer[3] = data >> 8;
    buffer[4] = data >> 0;
    SendPacket(ch, buffer, 5);
}

void SendInt16(uint8_t ch, uint8_t tock, uint16_t data)
{
    uint8_t buffer[3];
    buffer[0] = tock;
    buffer[1] = data >> 8;
    buffer[2] = data >> 0;
    SendPacket(ch, buffer, 3);
}

void ModemRxTask(void *param)
{

    uint16_t rx_index = 0;
    while (true)
    {
        uint8_t byte;
        if (UsartReceiveByte(mdm_port, &byte) == 1)
        {
            rx_buff[rx_index] = byte;
            rx_index++;
            if (rx_index >= 1600)
            {
                SendPacket(2, rx_buff, rx_index);
                rx_index = 0;
            }
        }
        else
        {
            if (rx_index > 0)
            {
                SendPacket(2, rx_buff, rx_index);
                rx_index = 0;
            }
        }
    }
}

void MasterRxTask(void *param)
{

    uint16_t rx_index = 0;
    bool packet_start = false;
    uint16_t packet_length = 0;
    uint8_t packet_cmd = 0;
    SendString(3, "Hello");
    while (true)
    {
        uint8_t byte;
        if (UsartReceiveByte(mstr_port, &byte) == 1)
        {
            if (byte == 0xF9 && packet_start == false)
            {
                rx_index = 1;
                packet_start = true;
                mstr_rx_buf[0] = byte;
            }
            else if (packet_start == true)
            {
                mstr_rx_buf[rx_index] = byte;
                if (rx_index == 2)
                {
                    packet_length = byte;
                }
                else if (rx_index == 3)
                {
                    packet_length <<= 8;
                    packet_length |= byte;
                }
                else if (rx_index == packet_length + 4)
                {
                    uint8_t rcrc = byte;
                    uint8_t ccrc = MakeCrc8(mstr_rx_buf, packet_length + 4);
                    if (rcrc == ccrc)
                    {
                        if (mstr_rx_buf[1] == 0x01)
                        {
                            packet_cmd = mstr_rx_buf[4];
                            switch (packet_cmd)
                            {
                            case 0x01:
                            {
                                PinWrite(vbat, false);
                                vTaskDelay(1000);
                                PinWrite(vbat, true);
                                vTaskDelay(500);
                                PinWrite(pwr, true);
                                vTaskDelay(1000);
                                PinWrite(pwr, false);
                                SendString(3, "Modem power probed!");
                            }
                            break;

                            default:
                                break;
                            }
                        }
                        else if(mstr_rx_buf[1] == 0x02)
                        {
                            UsartSendString(mdm_port, (const char *)&mstr_rx_buf[4], packet_length);
                        }
                    }
                    packet_start = false;
                }
                rx_index++;
                if (rx_index >= 1600)
                {
                    rx_index = 0;
                    packet_start = false;
                }
            }
        }
    }
}

int main()
{
    SystemCoreClockUpdate();
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
    GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);
    mstr_port = InitUsart(COM1, 115200, 0, 512);

    SendString(3, "Device initialized.");
    SendString(3, "Md. Mahmudul Hasan Sumon");
    SendString(3, "Afra Technology");
    SendInt32(3, 1, SystemCoreClock);

    mdm_port = InitUsart(COM3, 115200, 0, 512);

    vbat = PinCreate(PORTA, PIN_15, MODE_OUTPUT);
    pwr = PinCreate(PORTB, PIN_0, MODE_OUTPUT);
    rst = PinCreate(PORTB, PIN_4, MODE_OUTPUT);

    PinWrite(rst, false);
    PinWrite(pwr, false);
    PinWrite(vbat, false);
    xTaskCreate(ModemRxTask, "mdm", 512, NULL, 2, NULL);
    xTaskCreate(MasterRxTask, "mstr", 512, NULL, 1, NULL);
    SendString(3, "Running shcedular.");
    uint32_t freemem = xPortGetFreeHeapSize();
    SendInt32(3, 4, freemem);
    vTaskStartScheduler();

    SendString(3, "Code shouldn't reach here.");
    while (1)
    {
    }

    return 0;
}
