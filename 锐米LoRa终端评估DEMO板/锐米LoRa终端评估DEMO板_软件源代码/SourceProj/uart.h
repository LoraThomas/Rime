/**
 * \file
 *         uart.c
 * \description
 *         UARTx port for communication to RNDU470T
 * \author
 *         XXXXXXXXXXXXXXXXXXXXXXXXXX
 * \date
 *         2016-01-22 11:54
 * \copyright
 *         (c) RimeLink (www.rimelink.com)  All Rights Reserved.
 */


#ifndef __UART_H__
#define __UART_H__


/* Private macro -------------------------------------------------------------*/
/* EXPLAIN: switch USARTx for COMM as
    USART1    #define CPC_PORT_IS_USART1 in "CommPC.h"
    USART2    #define CPC_PORT_IS_USART2 in "CommPC.h"
    USART3    #define CPC_PORT_IS_USART3 in "CommPC.h" */
#define CPC_PORT_IS_USART3

#define CPC_PORT_BAUD_RATE    115200u   


/* Private function prototypes -----------------------------------------------*/
extern void cpc_Init(void);
extern int8_t cpc_Tx(const void *p_vSrcBuf, int16_t nLen);
extern void cpc_DMATxIRQHandler(void);
extern void cpc_RxIRQHandler(void);
extern void cpc_ToggleNodeMode(void);
extern void cpc_SetNodeMode(bool IsTDMA);


#endif    /* __UART_H__ */

