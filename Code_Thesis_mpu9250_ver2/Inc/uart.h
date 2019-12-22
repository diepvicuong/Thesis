#ifndef __UART_H
#define __UART_H

#ifdef __GNUC__
	#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
	#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__*/

void MX_USART1_UART_Init(void);
void MX_USART6_UART_Init(void);
#endif
