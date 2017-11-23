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

 
/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <string.h>
#include "stm8l15x_clk.h"
#include "stm8l15x_usart.h"
#include "stm8l15x_dma.h"
#include "stm8l15x_gpio.h"
#include "uart.h"

/* Private typedef -----------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/
/* Get offset of a specified field in a struct */
#define GET_ST_FLD_OFFSET(ST_TYPE, FLD)    (uint32_t)(&((ST_TYPE *)0)->FLD)

#if defined CPC_PORT_IS_USART1
    #define CPC_PORT    USART1
    #define CPC_TX_DMA_CH    DMA1_Channel1
    #define CPC_TX_DMA_IT_TC    DMA1_IT_TC1
#elif defined CPC_PORT_IS_USART2
    #define CPC_PORT    USART2
    #define CPC_TX_DMA_CH    DMA1_Channel0
    #define CPC_TX_DMA_IT_TC    DMA1_IT_TC0
#elif defined CPC_PORT_IS_USART3
    #define CPC_PORT    USART3
    #define CPC_TX_DMA_CH    DMA1_Channel1
    #define CPC_TX_DMA_IT_TC    DMA1_IT_TC1
#endif

#define CPC_PORT_BAUD_RATE    115200u   
#define USART_DR_OFFSET    GET_ST_FLD_OFFSET(USART_TypeDef, DR)

/* Allocate 2 pins to wake node for compatibility of LoraNode and LoraWAN-Node. */
#define WAKE_NODE_PORT    GPIOA
#define WAKE_NODE_CON1_PIN    GPIO_Pin_4 /* CON1=PA4 */
#define WAKE_NODE_CON2_PIN    GPIO_Pin_5 /* CON2=PA5 */
#define WAKE_NODE_CON3_PIN    GPIO_Pin_6 /* CON3=PA6 */


/* Private variables ---------------------------------------------------------*/
/* Buffer for CPC TX */
#define SIZE_CPC_TX_BUF    255
static uint8_t    s_abyCPCTxBuf[SIZE_CPC_TX_BUF];

/* Flag indicated that DMA is busy */
static volatile bool    s_bIsDMABusy = FALSE;


/* External function prototypes ---------------------------------------------------------*/
extern void DelayMs(uint16_t wMs);


/* Private function prototypes -----------------------------------------------*/
#define INIT_WAKE_NODE_PIN()    do {    \
        GPIO_Init(WAKE_NODE_PORT, WAKE_NODE_CON1_PIN, GPIO_Mode_In_FL_No_IT);    \
        GPIO_Init(WAKE_NODE_PORT, WAKE_NODE_CON2_PIN, GPIO_Mode_Out_PP_Low_Fast);    \
        GPIO_Init(WAKE_NODE_PORT, WAKE_NODE_CON3_PIN, GPIO_Mode_In_PU_No_IT);    \
    } while (0)

static bool    s_bNodeIsTDMA = TRUE;

/*---------------------------------------------------------------------------------------------*/
static void ConfigConPins(void)
{
    if (s_bNodeIsTDMA) /* TDMA */
    {
        GPIO_Init(WAKE_NODE_PORT, WAKE_NODE_CON1_PIN, GPIO_Mode_In_FL_No_IT);
        GPIO_Init(WAKE_NODE_PORT, WAKE_NODE_CON2_PIN, GPIO_Mode_Out_PP_Low_Fast);
    }
    else /* LoRaWAN */
    {
        GPIO_Init(WAKE_NODE_PORT, WAKE_NODE_CON1_PIN, GPIO_Mode_Out_PP_Low_Fast);
        GPIO_Init(WAKE_NODE_PORT, WAKE_NODE_CON2_PIN, GPIO_Mode_In_FL_No_IT);
    }

    return;
}

/*---------------------------------------------------------------------------------------------*/
static void SetWakeNodePinLow(void)
{
    if (s_bNodeIsTDMA) /* TDMA */
    {
        GPIO_ResetBits(WAKE_NODE_PORT, WAKE_NODE_CON2_PIN);
    }
    else /* LoRaWAN */
    {
        GPIO_ResetBits(WAKE_NODE_PORT, WAKE_NODE_CON1_PIN);
    }

    return;
}

/*---------------------------------------------------------------------------------------------*/
static void SetWakeNodePinHigh(void)
{
    if (s_bNodeIsTDMA) /* TDMA */
    {
        GPIO_SetBits(WAKE_NODE_PORT, WAKE_NODE_CON2_PIN);
    }
    else /* LoRaWAN */
    {
        GPIO_SetBits(WAKE_NODE_PORT, WAKE_NODE_CON1_PIN);
    }

    return;
}

/*---------------------------------------------------------------------------------------------*/
void cpc_ToggleNodeMode(void)
{
    s_bNodeIsTDMA = !s_bNodeIsTDMA;
    ConfigConPins();
    DelayMs(1); /* For steady of GPIO. */

    return;
}

/*---------------------------------------------------------------------------------------------*/
void cpc_SetNodeMode(bool IsTDMA)
{
    if (IsTDMA != s_bNodeIsTDMA)
    {
        s_bNodeIsTDMA = IsTDMA;
        ConfigConPins();
        DelayMs(1); /* For steady of GPIO. */
    }

    return;
}

/* Private Constants ---------------------------------------------------------*/

/**
  * @brief  Transmit data by CPC port
  * @note  It may block procedure while on previous transmission;
  * @param  const void * p_vSrcBuf    point to buffer which saved the desired TX data
  * @param  int16_t nLen    length of desired TX data
  * @retval  int8_t    -1=Tx data too long; -2=UART hardware error; 0=TX OK.
  */
int8_t cpc_Tx(const void *p_vSrcBuf, int16_t nLen)
{
    int16_t    nCnt;

    if (nLen > SIZE_CPC_TX_BUF)
    {
        return -1; /* Tx data too long */
    }

    /* Waiting until the DMA is not busy */
    nCnt = 0;	
    while (s_bIsDMABusy)
    {
        /* 0x7FFF=>24.5ms, 115200/9/1000*24.5=313.6B > 255B */
        if (++nCnt >= 0x7FFF)
        {
            return -2; /* UART hardware error: Send used too long time */
        }
    }

    /* Pull up AUX that awake the RNDU470T. */
    SetWakeNodePinHigh();

    /* Wait 1ms for Node ready. */
    DelayMs(1);

    /* Copy data into buffer */
    memcpy(s_abyCPCTxBuf, p_vSrcBuf, nLen);	

    s_bIsDMABusy = TRUE;

    /* Write this must DISABLE DMA_Channelx */
    DMA_SetCurrDataCounter(CPC_TX_DMA_CH, (uint8_t)nLen); 

    DMA_Cmd(CPC_TX_DMA_CH, ENABLE); /* Enable DMA1_Channelx */

    return 0;
}


/**
  * @brief  Initialize USARTx port for communication to PC.
  * @note  None.
  * @param  None.
  * @retval  None.
  */
void cpc_Init(void)
{
    INIT_WAKE_NODE_PIN();

    /* Enable clock to USARTx and DMA1 */
    CLK_PeripheralClockConfig(CLK_Peripheral_DMA1, ENABLE);	

#if defined CPC_PORT_IS_USART1
    CLK_PeripheralClockConfig(CLK_Peripheral_USART1, ENABLE);
#elif defined CPC_PORT_IS_USART2
    CLK_PeripheralClockConfig(CLK_Peripheral_USART2, ENABLE);
#elif defined CPC_PORT_IS_USART3
    CLK_PeripheralClockConfig(CLK_Peripheral_USART3, ENABLE);
#endif

    /* Configure USART Tx and RX as alternate function push-pull(software pull up) */
#if defined CPC_PORT_IS_USART1
    GPIO_ExternalPullUpConfig(GPIOC, GPIO_Pin_2, ENABLE);
    GPIO_ExternalPullUpConfig(GPIOC, GPIO_Pin_3, ENABLE);
#elif defined CPC_PORT_IS_USART2
    GPIO_ExternalPullUpConfig(GPIOE, GPIO_Pin_3, ENABLE);
    GPIO_ExternalPullUpConfig(GPIOE, GPIO_Pin_4, ENABLE);
#elif defined CPC_PORT_IS_USART3
    GPIO_ExternalPullUpConfig(GPIOE, GPIO_Pin_6, ENABLE);
    GPIO_ExternalPullUpConfig(GPIOE, GPIO_Pin_7, ENABLE);
#endif

    /* Initialize USARTx as well as enable TX and RX */
    USART_Init( CPC_PORT, 
                       CPC_PORT_BAUD_RATE, 
                       USART_WordLength_8b, 
                       USART_StopBits_1, 
                       USART_Parity_No, 
                       (USART_Mode_TypeDef)(USART_Mode_Tx | USART_Mode_Rx) );

    /* Enable the USARTx Receive interrupt: this interrupt is generated when the USARTx
        receive data register is not empty */
    USART_ITConfig(CPC_PORT, USART_IT_RXNE, ENABLE);

    /* Deinitialize DMA channels */
    DMA_GlobalDeInit();
    DMA_DeInit(CPC_TX_DMA_CH);

    /* Initialize TX DMA */
    DMA_Init( CPC_TX_DMA_CH, 
                    (uint16_t)&s_abyCPCTxBuf[0],
                    (uint16_t)(CPC_PORT) + USART_DR_OFFSET,
                    SIZE_CPC_TX_BUF,
                    DMA_DIR_MemoryToPeripheral,
                    DMA_Mode_Normal,
                    DMA_MemoryIncMode_Inc,
                    DMA_Priority_High,
                    DMA_MemoryDataSize_Byte );

    /* Turn on UART_TX but not for DMA_TX */
    USART_DMACmd(CPC_PORT, USART_DMAReq_TX, ENABLE);
    DMA_Cmd(CPC_TX_DMA_CH, DISABLE); 

    /* Enable DMA Transaction Complete Interrupt */
    DMA_ITConfig(CPC_TX_DMA_CH, DMA_ITx_TC, ENABLE);

    /* Initialize DMA for TX and RX */
    DMA_GlobalCmd(ENABLE); /* Enable DMA1 */

    return;
}
extern void RxUartData(uint8_t byData);
/*---------------------------------------------------------------------------------------------*/
void cpc_RxIRQHandler(void)
{
    uint8_t    byData;

    /* Check whether is an error of overrun */
    if (USART_GetFlagStatus(CPC_PORT, USART_FLAG_OR))
    {
        /* It is cleared by a software sequence: a read to the USART_SR register
            followed by a read to the USART_DR register. */
    }	

    /* EXPLAIN: Interrupt flag of "RXNE" is cleared by a read to the USART_DR register */
    byData = USART_ReceiveData8(CPC_PORT);

    RxUartData(byData);

    return;
}

/*---------------------------------------------------------------------------------------------*/
void cpc_DMATxIRQHandler(void)
{
    s_bIsDMABusy = FALSE; /* Transfer completed */

    DMA_ClearITPendingBit(CPC_TX_DMA_IT_TC);

    DMA_Cmd(CPC_TX_DMA_CH, DISABLE); /* Disable DMA1_Channelx */

    /* Pull down AUX indicated that TX a UART frame is done. */
    SetWakeNodePinLow();

    return;
}


/*--------------------------------------------------------------------------------------------------------
                   									  0ooo
                   								ooo0     (   )
                								(   )     ) /
                								 \ (     (_/
                								  \_)
----------------------------------------------------------------------------------------------------------*/


