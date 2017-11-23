/**
 * \file
 *         key.h
 * \description
 *         Deal with key input
 * \author
 *         XXXXXXXXXXXXXXXXXXXXXXXXXX
 * \date
 *         2016-04-16 15:30
 * \copyright
 *         (c) RimeLink (www.rimelink.com)  All Rights Reserved.
 */


#ifndef __KEY_H__
#define __KEY_H__


/* Private macro -------------------------------------------------------------*/
#define DEBUG_IOPORT    GPIOA
#define DEBUG_PIN    GPIO_Pin_2

/* Private function prototypes -----------------------------------------------*/
extern void key_Init(void);
extern void key_IRQHandler(void);
bool key_GetStatus(void);
void key_ClrStatus(void);

#endif    /* __KEY_H__ */


