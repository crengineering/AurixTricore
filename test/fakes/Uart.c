/* Host fake for the debug console. Captures the last printed line so a test
 * can assert what the driver assembled, instead of only counting sentences.
 *
 * No include guard: this is a translation unit, not a header.
 */

#include "Uart.h"
#include <string.h>

#define FAKE_UART_LINE_MAX 128u

static char   s_lastLine[FAKE_UART_LINE_MAX] = "";
static uint32 s_lineCount;

void FakeUart_reset(void)
{
    s_lastLine[0] = '\0';
    s_lineCount   = 0u;
}

const char *FakeUart_lastLine(void)
{
    return s_lastLine;
}

uint32 FakeUart_lineCount(void)
{
    return s_lineCount;
}

void Uart_print(const char *str)
{
    (void)str;   /* the driver only uses println */
}

void Uart_println(const char *str)
{
    size_t len = strlen(str);

    if (len >= FAKE_UART_LINE_MAX)
    {
        len = FAKE_UART_LINE_MAX - 1u;
    }
    memcpy(s_lastLine, str, len);
    s_lastLine[len] = '\0';
    s_lineCount++;
}
