/**
 ******************************************************************************
 * @file    hello_world.c
 * @author  William Xu
 * @version V1.0.0
 * @date    21-May-2015
 * @brief   First MiCO application to say hello world!
 ******************************************************************************
 *
 *  The MIT License
 *  Copyright (c) 2016 MXCHIP Inc.
 *
 *  Permission is hereby granted, free of charge, to any person obtaining a copy
 *  of this software and associated documentation files (the "Software"), to deal
 *  in the Software without restriction, including without limitation the rights
 *  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *  copies of the Software, and to permit persons to whom the Software is furnished
 *  to do so, subject to the following conditions:
 *
 *  The above copyright notice and this permission notice shall be included in
 *  all copies or substantial portions of the Software.
 *
 *  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 *  WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR
 *  IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 ******************************************************************************
 */

#include "mico.h"

#define uart_log(format, ...)  custom_log("", format, ##__VA_ARGS__)
#define UART_BUFFER_LENGTH 2048
#define UART_ONE_PACKAGE_LENGTH 20

#define data "12345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890"
//#define data "1234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456"
//#define data "1234567890"

volatile ring_buffer_t rx_buffer;
volatile uint8_t rx_data[UART_BUFFER_LENGTH];

int _uart_get_one_packet(uint8_t *inBuf, int inBufLen, uint32_t timeout);

bool is_stoprx = false;
bool is_stoptx = true;
int count = 0, recv_count = 0;

static void stoprx_mode(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv)
{
    UNUSED_PARAMETER(pcWriteBuffer);
    UNUSED_PARAMETER(xWriteBufferLen);
    UNUSED_PARAMETER(argc);
    UNUSED_PARAMETER(argv);

    if (!is_stoprx) {
        is_stoprx = true;
        uart_log("stop rx");
        uart_log("recv %d", recv_count);
    } else {
        is_stoprx = false;
        uart_log("start rx");
    }
}

static void stoptx_mode(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv)
{
    UNUSED_PARAMETER(pcWriteBuffer);
    UNUSED_PARAMETER(xWriteBufferLen);
    UNUSED_PARAMETER(argc);
    UNUSED_PARAMETER(argv);

    is_stoptx = !is_stoptx;
    uart_log("stop tx");
}

static const struct cli_command user_clis[] = {
        {"rx", "stop recv", stoprx_mode},
        {"tx", "stop send", stoptx_mode},
};

void user_commands_register()
{
    cli_register_commands(user_clis, sizeof(user_clis) / sizeof(struct cli_command));
}

void uartRecv_thread(uint32_t arg)
{
    int recvlen = 0;
    uint8_t *inDataBuffer;

    inDataBuffer = (uint8_t *)malloc(UART_ONE_PACKAGE_LENGTH);

    while (true) {

        if (is_stoprx == true) {
            mico_rtos_thread_msleep(10);
            continue;
        }

        recvlen = MicoUartRecv(MICO_UART_FOR_APP, inDataBuffer, UART_ONE_PACKAGE_LENGTH, 500);
        if (recvlen != kNoErr) {
            uart_log("MicoUartRecv failed: %d", recvlen);
            continue;
        }
        recv_count += UART_ONE_PACKAGE_LENGTH;
        uart_log("count %ld, recv %d", recv_count, UART_ONE_PACKAGE_LENGTH);
    }
}

int app_uart_test()
{
    mico_uart_config_t uart_config = {
        .baud_rate = 921600,
        .data_width = DATA_WIDTH_8BIT,
        .parity = NO_PARITY,
        .stop_bits = STOP_BITS_1,
        .flow_control = FLOW_CONTROL_CTS_RTS,
        .flags = UART_WAKEUP_DISABLE
    };

    /* Start MiCO system functions according to mico_config.h*/
    mico_system_context_init(0);

    user_commands_register();

    ring_buffer_init((ring_buffer_t *) &rx_buffer, (uint8_t *) rx_data, UART_BUFFER_LENGTH);
//    MicoUartInitialize(MICO_UART_FOR_APP, &uart_config, (ring_buffer_t *) &rx_buffer);
    MicoUartInitialize(MICO_UART_FOR_APP, &uart_config, NULL);

    /* Output on debug serial port */
    uart_log("%s: Hello world!", __FUNCTION__);

    mico_rtos_create_thread(NULL, MICO_APPLICATION_PRIORITY, "UART Recv", uartRecv_thread,
                            0x800, 0);

    int is_log = 0;
    while (true) {
        if (is_stoptx) {
            if (is_log == 1) {
                uart_log("send %d", count);
                is_log = 0;
            }

            mico_rtos_thread_msleep(10);
            continue;
        }
        MicoUartSend(MICO_UART_FOR_APP, data, strlen(data));
        count += strlen(data);
        is_log = 1;
        uart_log("send %d", count);
    }
}

static int uart_getchar(uint8_t *inbuf, uint32_t len, uint32_t timeout)
{
    if (MicoUartRecv(MICO_UART_FOR_APP, inbuf, len, timeout) == 0)
        return 1;
    else
        return 0;
}

/* Packet format: BB 00 CMD(2B) Status(2B) datalen(2B) data(x) checksum(2B)
* copy to buf, return len = datalen+10
*/
int _uart_get_one_packet(uint8_t *inBuf, int inBufLen, uint32_t timeout)
{
    int datalen = 0;

    while (true) {
        if (uart_getchar(&inBuf[datalen], 1, timeout) == 1) {

            (datalen)++;
            if (datalen >= inBufLen) {
                return datalen;
            }
        } else {
            return datalen;
        }
    }
}
