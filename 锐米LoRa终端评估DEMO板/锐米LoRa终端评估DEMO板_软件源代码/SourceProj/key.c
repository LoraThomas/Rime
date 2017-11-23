/**
 * \file
 *         key.c
 * \description
 *         Deal with key input
 * \author
 *         XXXXXXXXXXXXXXXXXXXXXXXXXX
 * \date
 *         2016-04-16 15:30
 * \copyright
 *         (c) RimeLink (www.rimelink.com)  All Rights Reserved.
 */

 
/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <string.h>
#include "stm8l15x_gpio.h"
#include "key.h"

/* Private typedef -----------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
/* Flag indicated that Debug key is pressed */
static volatile bool    s_bKeyPress = FALSE;


/* Private function prototypes -----------------------------------------------*/
extern void KeySetEvent(void);


/* Private Constants ---------------------------------------------------------*/
/**
  * @brief  Initialize port for Debug key and Reset key.
  * @note  MUST disable global interrupts otherwise may incurred chaos of external interrupts.
  * @param  None.
  * @retval  None.
  */
void key_Init(void)
{
    GPIO_Init(DEBUG_IOPORT, DEBUG_PIN, GPIO_Mode_In_FL_IT);
    EXTI_SetPinSensitivity(EXTI_Pin_2, EXTI_Trigger_Rising);

    return;
}

/*---------------------------------------------------------------------------------------------*/
void key_IRQHandler(void)
{
    EXTI_ClearITPendingBit(EXTI_IT_Pin2);

    s_bKeyPress = TRUE;
    KeySetEvent();

    return;
}

/**
  * @brief  Get key status
  * @note  None
  * @param  None
  * @retval  bool    TRUE=key is pressed; FALSE=key is not pressed.
  */
bool key_GetStatus(void)
{
    return (s_bKeyPress);
}

/*---------------------------------------------------------------------------------------------*/
void key_ClrStatus(void)
{
    s_bKeyPress = FALSE;

    return;
}


/*--------------------------------------------------------------------------------------------------------
                   									  0ooo
                   								ooo0     (   )
                								(   )     ) /
                								 \ (     (_/
                								  \_)
----------------------------------------------------------------------------------------------------------*/

