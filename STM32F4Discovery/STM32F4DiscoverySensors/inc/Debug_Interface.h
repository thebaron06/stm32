#ifndef DEBUG_INTERFACE_H_
#define DEBUG_INTERFACE_H_
/**
  ******************************************************************************
  * @file    Debug_Interface.h
  * @author  Roland Jung - SmartHomeEasy Prj.
  * @version V1.23.01.14
  * @date    23.01.2014 11:12:29
  * @brief   Debug_Interface
  ******************************************************************************
  * @attention
  *
  ******************************************************************************
  */
#include <stdint.h>

void debug_Init();
/** \brief: small version of printf:
* accepted formats:
** %s = arg is a string
** %d = arg is a signed decimal
** %x = arg is a hex
** %X = arg is a hex
** %u = arg is a unsigned decimal
** %c = arg is a character
*/
void debug_printformatted(const char *format, ...);

/* tested on: 23.01.14:
* UARTC_BUFFERSIZE		( 0x7F)
* UARTC_BAUDRATE		56000
* DEBUG_PUTC(ch) (uartc_putc(ch))
* uartc_init();
		debug_printformatted("A string:%s", "2nd string");
		debug_printformatted("A decimal:%d", -100);
		debug_printformatted("A small hex:%x", 202);
		debug_printformatted("A big hex:%X", 175);
		debug_printformatted("A unsigned:%u", 100);
		debug_printformatted("A char:%c", 'A');;
*/
#endif /* DEBUG_INTERFACE_H_ */
