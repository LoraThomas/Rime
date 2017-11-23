 /**
 * \file
 *         main.c
 * \description
 *         Entry for LoRaNode DEMO project.
 * \author
 *         Jiang Jun <jj@rimelink.com>
 * \date
 *         2017-06-21 10:25
 * \copyright
 *         (c) RimeLink 2017-2022 (www.rimelink.com) All Rights Reserved.
 * \platform
 *         MCU=STM8L151C8 OS=None
 */


/* Includes ------------------------------------------------------------------*/
#include <string.h>
#include <stdio.h>
#include <stdint.h>
#include <assert.h>
#include "stm8l15x_clk.h"
#include "stm8l15x_rtc.h"
#include "SHT7x.h"
#include "timer.h"
#include "uart.h"
#include "key.h"
#include "DebugPort.h"


/* Compile switch -------------------------------------------------------------*/
#define ASSERT(cond)    null()
#define EN_TEST_NODE_UART    0 /* 0=release, 1=debug */


/* Private macro -------------------------------------------------------------*/
#define MAX_LEN_COMM_TRM_DATA    255u
#define MAX_LEN_UART_FRAME_DATA    \
    (MAX_LEN_COMM_TRM_DATA - sizeof(COMM_FRAME_HEAD) - sizeof(COMM_FRAME_TAIL))

#define COMM_TRM_HEAD    0x3Cu
#define COMM_TRM_TAIL    0x0Du

/* Define LED and function. */
#define LED1_IOPORT    GPIOD
#define LED1_PIN    GPIO_Pin_4

#define LED2_IOPORT    GPIOD
#define LED2_PIN    GPIO_Pin_5

#define LED3_IOPORT    GPIOD
#define LED3_PIN    GPIO_Pin_6

#define LED4_IOPORT    GPIOD
#define LED4_PIN    GPIO_Pin_7

#define UPLINK_LED_IOPORT    LED4_IOPORT
#define UPLINK_LED_PIN    LED4_PIN

#define KEY_LED_IOPORT    LED3_IOPORT
#define KEY_LED_PIN    LED3_PIN

#define DOWNLOAD_LED_IOPORT    LED2_IOPORT
#define DOWNLOAD_LED_PIN    LED2_PIN

#define ALARM_LED_IOPORT    LED1_IOPORT
#define ALARM_LED_PIN    LED1_PIN

/* Distinguish type of system events: RTC, KEY, GATEWAY. */
#define SYST_EVENT_RTC    0x01
#define SYST_EVENT_KEY    0x02
#define SYST_EVENT_GATEWAY    0x04


/* Private typedef -----------------------------------------------------------*/
/**
* @brief  Configure of MCU GPIO. 
*/
typedef struct
{
    uint8_t    byNum; /* 1~48 */
    GPIO_TypeDef    *p_byPort; /* A~F */
    uint8_t    byPin; /* 0~7 */
    bool    bNeedSWMode; /* TRUE=Switch mode when enter low power, FALSE=otherwise */
    GPIO_Mode_TypeDef    eModeMCURun; /* GPIO mode on MCU run */
    GPIO_Mode_TypeDef    eModeLowPower; /* GPIO mode on low power */	
} CONFIG_MCU_GPIO;
/**
* @brief  Status of received communication frame
*/
typedef enum
{
    STATUS_IDLE = (uint8_t)0,
    STATUS_HEAD, /* Rx Head=0x3C */
    STATUS_TYPE, /* Rx Type */
    STATUS_DATA, /* Data field */
    STATUS_TAIL, /* Tail=0x0D */
    STATUS_END, /* End of this frame */
} COMM_TRM_STATUS_TypeDef;

/**
* @brief  Data object for received communication frame
*/
typedef struct
{
    uint8_t    byCnt; /* Count of 1 field */
    uint8_t    byDataLen; /* Length of data field */
    uint8_t    byFrameLen; /* Length of frame */
    COMM_TRM_STATUS_TypeDef    eRxStatus;
    uint8_t    a_byRxBuf[MAX_LEN_COMM_TRM_DATA];	
} COMM_TRM_DATA;

/**
* @brief  Type of received communication frame
*/
typedef enum
{
    TYPE_INVALID_MIN = (uint8_t)0,
    TYPE_GET_VER, /*!< User System get version of this node. */
    TYPE_TX_RF_DATA, /*!< User System send data that need to TX by RF. */
    TYPE_SET_RF_SETTINGS, /*!< User System set settings of RF. */
    TYPE_GET_RF_SETTINGS, /*!< User System get settings of RF. */
    TYPE_GET_NET_SETTINGS, /*!< User System get network settings of this node. */
    TYPE_QUIT_NET, /*!< User System set this node quit from current network. */
    TYPE_SET_TX_PWR, /*!< User System set TX power of RF. */
    TYPE_GET_TX_PWR, /*!< User System get TX power of RF. */
    TYPE_WAKE_ACK, /*!< User System send wake ack to node. */
    TYPE_GET_PKT_RSSI, /*!< User System get the RSSI of the latest packet received. */
    TYPE_INVALID_MAX,

    TYPE_WAKE_DATA = ((uint8_t)0xC0), /*!< Node send wake data to User System. */
} COMM_FRAME_TYPE_TypeDef;

typedef struct
{
    uint8_t    byHead;
    COMM_FRAME_TYPE_TypeDef    eType;
    uint8_t    byDataSize;
} COMM_FRAME_HEAD;

typedef struct
{
    uint8_t    byCS;
    uint8_t    byTail;
} COMM_FRAME_TAIL;

typedef struct _temp_humi
{
    float    fTemp; /* temperature */
    float    fHumi; /* humidity */
    float    fDewPoint; /* dew point */
} TEMP_HUMI;

typedef volatile uint8_t    Atomic_t;

typedef uint8_t    halIntState_t; /*< Used for save interrupt state */


/* Private variables ---------------------------------------------------------*/
/**
* @brief  Data object for received communication frame.
*/
static COMM_TRM_DATA    s_stComm2TrmData;

static uint8_t    s_abyTxUARTFrame[MAX_LEN_COMM_TRM_DATA];

static volatile bool    s_bRxUARTFrame = FALSE;

static TEMP_HUMI    s_stTempHumi;

/* ATTENTION: MUST avoid race condition on "s_tSystEvent" that accessed by ISR and procedure. */
static Atomic_t    s_tSystEvent = 0;


/* Private function prototypes -----------------------------------------------*/
#define null()
#define SIZE_OF_ARRAY(a)    (sizeof(a)/sizeof(a[0]))

/**
* @brief  Make response type of UART frame. 
*/
#define MAKE_UART_TYPE_RESP(byType)    (0x80u + (byType))

 /* Interrupt Macros */
#define HAL_ENABLE_INTERRUPTS()    __enable_interrupt()
#define HAL_DISABLE_INTERRUPTS()    __disable_interrupt()

/* Save interrupt mask and disable interrupts */
#define HAL_ENTER_CRITICAL_SECTION(intState)    \
    do    \
    {    \
        intState = __get_interrupt_state();    \
        __disable_interrupt();    \
    } while (0)

 /* Restore interrupts */
#define HAL_EXIT_CRITICAL_SECTION(intState)    \
    do    \
    {    \
        __set_interrupt_state(intState);    \
    } while (0)

/* Initialize and Pull Up/Down of NODE_RESET. */
#define RESET_NODE_IOPORT    GPIOA
#define RESET_NODE_PIN    GPIO_Pin_7

#define INIT_RESET_NODE_PIN()    \
    GPIO_Init(RESET_NODE_IOPORT, RESET_NODE_PIN, GPIO_Mode_Out_PP_High_Slow)

#define PULL_DOWN_RESET_NODE_PIN()    \
    GPIO_ResetBits(RESET_NODE_IOPORT, RESET_NODE_PIN)

#define PULL_UP_RESET_NODE_PIN()    \
    GPIO_SetBits(RESET_NODE_IOPORT, RESET_NODE_PIN)

/**
* @brief  Configure of MCU GPIO. 
*/
const static CONFIG_MCU_GPIO    s_astConfigGPIO[] =
{
    /* have not connected anything */
    {4, GPIOA, GPIO_Pin_3, FALSE, GPIO_Mode_Out_PP_Low_Slow, GPIO_Mode_Out_PP_Low_Slow},
    {14, GPIOE, GPIO_Pin_0, FALSE, GPIO_Mode_Out_PP_Low_Slow, GPIO_Mode_Out_PP_Low_Slow},
    {15, GPIOE, GPIO_Pin_1, FALSE, GPIO_Mode_Out_PP_Low_Slow, GPIO_Mode_Out_PP_Low_Slow},
    {16, GPIOE, GPIO_Pin_2, FALSE, GPIO_Mode_Out_PP_Low_Slow, GPIO_Mode_Out_PP_Low_Slow},
    {19, GPIOE, GPIO_Pin_5, FALSE, GPIO_Mode_Out_PP_Low_Slow, GPIO_Mode_Out_PP_Low_Slow},
    {20, GPIOD, GPIO_Pin_0, FALSE, GPIO_Mode_Out_PP_Low_Slow, GPIO_Mode_Out_PP_Low_Slow},
    {21, GPIOD, GPIO_Pin_1, FALSE, GPIO_Mode_Out_PP_Low_Slow, GPIO_Mode_Out_PP_Low_Slow},
    {32, GPIOF, GPIO_Pin_0, FALSE, GPIO_Mode_Out_PP_Low_Slow, GPIO_Mode_Out_PP_Low_Slow},
    {43, GPIOC, GPIO_Pin_4, FALSE, GPIO_Mode_Out_PP_Low_Slow, GPIO_Mode_Out_PP_Low_Slow},
    {44, GPIOC, GPIO_Pin_5, FALSE, GPIO_Mode_Out_PP_Low_Slow, GPIO_Mode_Out_PP_Low_Slow},
    {45, GPIOC, GPIO_Pin_6, FALSE, GPIO_Mode_Out_PP_Low_Slow, GPIO_Mode_Out_PP_Low_Slow},
    {46, GPIOC, GPIO_Pin_7, FALSE, GPIO_Mode_Out_PP_Low_Slow, GPIO_Mode_Out_PP_Low_Slow},

    /* MCU output */
    {1, GPIOA, GPIO_Pin_0, FALSE, GPIO_Mode_Out_PP_High_Slow, GPIO_Mode_Out_PP_High_Slow},
    {5, GPIOA, GPIO_Pin_4, FALSE, GPIO_Mode_Out_PP_High_Slow, GPIO_Mode_Out_PP_High_Slow},
    {7, GPIOA, GPIO_Pin_6, FALSE, GPIO_Mode_Out_PP_Low_Slow, GPIO_Mode_Out_PP_Low_Slow},
    {8, GPIOA, GPIO_Pin_7, FALSE, GPIO_Mode_Out_PP_Low_Slow, GPIO_Mode_Out_PP_Low_Slow},
    {17, GPIOE, GPIO_Pin_3, FALSE, GPIO_Mode_Out_PP_Low_Slow, GPIO_Mode_Out_PP_Low_Slow},
    {18, GPIOE, GPIO_Pin_4, FALSE, GPIO_Mode_Out_PP_Low_Slow, GPIO_Mode_Out_PP_Low_Slow},
    //{17, GPIOE, GPIO_Pin_3, FALSE, GPIO_Mode_In_FL_No_IT, GPIO_Mode_In_FL_No_IT},
    //{18, GPIOE, GPIO_Pin_4, FALSE, GPIO_Mode_Out_PP_High_Slow, GPIO_Mode_Out_PP_High_Slow},
    {22, GPIOD, GPIO_Pin_2, FALSE, GPIO_Mode_Out_PP_Low_Slow, GPIO_Mode_Out_PP_Low_Slow},
    {23, GPIOD, GPIO_Pin_3, FALSE, GPIO_Mode_Out_PP_Low_Slow, GPIO_Mode_Out_PP_Low_Slow},
    {28, GPIOB, GPIO_Pin_4, FALSE, GPIO_Mode_Out_PP_Low_Slow, GPIO_Mode_Out_PP_Low_Slow},
    {33, GPIOD, GPIO_Pin_4, FALSE, GPIO_Mode_Out_PP_High_Slow, GPIO_Mode_Out_PP_High_Slow},
    {34, GPIOD, GPIO_Pin_5, FALSE, GPIO_Mode_Out_PP_High_Slow, GPIO_Mode_Out_PP_High_Slow},
    {35, GPIOD, GPIO_Pin_6, FALSE, GPIO_Mode_Out_PP_High_Slow, GPIO_Mode_Out_PP_High_Slow},
    {36, GPIOD, GPIO_Pin_7, FALSE, GPIO_Mode_Out_PP_High_Slow, GPIO_Mode_Out_PP_High_Slow},
    {37, GPIOC, GPIO_Pin_0, FALSE, GPIO_Mode_Out_PP_Low_Slow, GPIO_Mode_Out_PP_Low_Slow},
    {38, GPIOC, GPIO_Pin_1, FALSE, GPIO_Mode_Out_PP_Low_Slow, GPIO_Mode_Out_PP_Low_Slow},
    {41, GPIOC, GPIO_Pin_2, FALSE, GPIO_Mode_Out_PP_Low_Slow, GPIO_Mode_Out_PP_Low_Slow},
    {42, GPIOC, GPIO_Pin_3, FALSE, GPIO_Mode_Out_PP_Low_Slow, GPIO_Mode_Out_PP_Low_Slow},
    {47, GPIOE, GPIO_Pin_6, FALSE, GPIO_Mode_Out_PP_High_Slow, GPIO_Mode_Out_PP_High_Slow},
    {6, GPIOA, GPIO_Pin_5, FALSE, GPIO_Mode_Out_PP_Low_Slow, GPIO_Mode_Out_PP_Low_Slow},

    /* MCU input */
    {2, GPIOA, GPIO_Pin_1, FALSE, GPIO_Mode_In_FL_No_IT, GPIO_Mode_In_FL_No_IT},
    {3, GPIOA, GPIO_Pin_2, FALSE, GPIO_Mode_In_FL_IT, GPIO_Mode_In_FL_No_IT},
    
    {24, GPIOB, GPIO_Pin_0, FALSE, GPIO_Mode_In_FL_No_IT, GPIO_Mode_In_FL_No_IT},
    {25, GPIOB, GPIO_Pin_1, FALSE, GPIO_Mode_In_FL_No_IT, GPIO_Mode_In_FL_No_IT},
    {26, GPIOB, GPIO_Pin_2, FALSE, GPIO_Mode_In_FL_No_IT, GPIO_Mode_In_FL_No_IT},
    {27, GPIOB, GPIO_Pin_3, FALSE, GPIO_Mode_In_FL_No_IT, GPIO_Mode_In_FL_No_IT},
    {29, GPIOB, GPIO_Pin_5, FALSE, GPIO_Mode_In_PU_No_IT, GPIO_Mode_In_PU_No_IT},
    {30, GPIOB, GPIO_Pin_6, FALSE, GPIO_Mode_In_PU_No_IT, GPIO_Mode_In_PU_No_IT},
    {31, GPIOB, GPIO_Pin_7, FALSE, GPIO_Mode_In_FL_No_IT, GPIO_Mode_In_FL_No_IT},
    {48, GPIOE, GPIO_Pin_7, FALSE, GPIO_Mode_In_FL_No_IT, GPIO_Mode_In_FL_No_IT},
};

/* Private Constants ---------------------------------------------------------*/
/*-------------------------------------------------------------------------*/
static void GPIO_LowPower_Init(void)
{
    int16_t    nCnt;
    
    /* Set all pins to mode of low power. */
    for (nCnt = 0; nCnt < SIZE_OF_ARRAY(s_astConfigGPIO); ++nCnt)
    {
        GPIO_Init( s_astConfigGPIO[nCnt].p_byPort,
                         s_astConfigGPIO[nCnt].byPin,
                         s_astConfigGPIO[nCnt].eModeLowPower );
    }
}

/*-------------------------------------------------------------------------*/
static void led_Init()
{
    GPIO_Init(LED1_IOPORT, LED1_PIN, GPIO_Mode_Out_PP_High_Slow);
    GPIO_Init(LED2_IOPORT, LED2_PIN, GPIO_Mode_Out_PP_High_Slow);
    GPIO_Init(LED3_IOPORT, LED3_PIN, GPIO_Mode_Out_PP_High_Slow);
    GPIO_Init(LED4_IOPORT, LED4_PIN, GPIO_Mode_Out_PP_High_Slow);
    GPIO_ResetBits(LED1_IOPORT, LED1_PIN);
    GPIO_ResetBits(LED2_IOPORT, LED2_PIN);
    GPIO_ResetBits(LED3_IOPORT, LED3_PIN);
    GPIO_ResetBits(LED4_IOPORT, LED4_PIN);
}

/*-------------------------------------------------------------------------*/
uint8_t CalcCS(const void *p_vBuf, int16_t nSize)
{
    uint8_t    bySum;
    const uint8_t    *p_byBuf;

    bySum = 0;
    p_byBuf = (const uint8_t *)p_vBuf;
    
    while (nSize-- > 0)
    {
        bySum += *p_byBuf++;
    }

    return bySum;
}


/*-------------------------------------------------------------------------*/
static void ClearCommFrame(void)
{
     /* Clear frame for next frame */
    s_stComm2TrmData.byCnt = 0;
    s_stComm2TrmData.byDataLen = 0;
    s_stComm2TrmData.byFrameLen = 0;
    s_stComm2TrmData.eRxStatus = STATUS_IDLE;

    return;
}

/**
  * @brief  Put a data that received by UART into buffer.
  * @note  Prevent race condition this called by ISR. 
  * @param  uint8_t byData: the data received by UART.
  * @retval  None
  */
void RxUartData(uint8_t byData)
{
    /* Update status according to the received data */
    switch (s_stComm2TrmData.eRxStatus)
    {
        case STATUS_IDLE:
            if (COMM_TRM_HEAD == byData) /* Is Head */
            {
                s_stComm2TrmData.eRxStatus = STATUS_HEAD;
            }
            else
            {
                goto rx_exception;
            }
            break;
        case STATUS_HEAD:
            if ( (TYPE_WAKE_DATA == byData) ||
                 (MAKE_UART_TYPE_RESP(TYPE_INVALID_MIN) < byData) && 
                 (byData < MAKE_UART_TYPE_RESP(TYPE_INVALID_MAX)) ) /* Valid type */
            {
                s_stComm2TrmData.eRxStatus = STATUS_TYPE;
                if (TYPE_WAKE_DATA == byData)
                {
                    GPIO_SetBits(DOWNLOAD_LED_IOPORT, DOWNLOAD_LED_PIN);
                }
            }
            else
            {
                goto rx_exception;
            }
            break;
        case STATUS_TYPE:
            if (byData <= MAX_LEN_UART_FRAME_DATA) /* Valid data size */
            {
                s_stComm2TrmData.eRxStatus = STATUS_DATA;
                s_stComm2TrmData.byDataLen = byData;
            }
            else
            {
                goto rx_exception;
            }
            break;
        case STATUS_DATA:
            if (s_stComm2TrmData.byCnt < s_stComm2TrmData.byDataLen)
            {
                ++s_stComm2TrmData.byCnt;
            }
            else
            {
                s_stComm2TrmData.eRxStatus = STATUS_TAIL;
            }
            break;
        case STATUS_TAIL:
            if (COMM_TRM_TAIL == byData)
            {
                s_bRxUARTFrame = TRUE;

                /* Tell main procedure to process WakeData. */
                const COMM_FRAME_HEAD    *p_stFrameHead;
                p_stFrameHead = (const COMM_FRAME_HEAD *)s_stComm2TrmData.a_byRxBuf;
                if (TYPE_WAKE_DATA == p_stFrameHead->eType)
                {
                    s_tSystEvent |= SYST_EVENT_GATEWAY;
                    GPIO_ResetBits(DOWNLOAD_LED_IOPORT, DOWNLOAD_LED_PIN);
                }
            }
            else
            {
                goto rx_exception;
            }
            break;
        default:
            ASSERT(!"Error: Bad status of comm2trm_RxUartData().\r\n");
            break;
    }

    /* Save the received data */
    s_stComm2TrmData.a_byRxBuf[s_stComm2TrmData.byFrameLen++] = byData;
    return;

rx_exception:
    GPIO_ResetBits(DOWNLOAD_LED_IOPORT, DOWNLOAD_LED_PIN);
    ClearCommFrame(); /* For RX next frame */

    return;	
}


/**
  * @brief  Delay a few of millisecond.
  * @note  This function ONLY delay a few of millisecond since that waiting and loop 
  *            would consumes CPU.
  * @note  Be careful of overflow that capability of "uint16_t" is 65535, so the MAX count
  *            of rtimer equal=(65535 / RTIMER_ARCH_SECOND) seconds.
  * @param  wMs: count of millisecond.
  * @retval  None
  */
void DelayMs(uint16_t wMs)
{
    uint16_t    wCount;
    rtimer_clock_t    tStart;

    /* Conver millisecond to Hz of rtimer */
    wCount = (uint32_t)wMs * RTIMER_ARCH_SECOND / 1000;

    tStart = RTIMER_NOW();
    while (RTIMER_NOW() - tStart < wCount)
    {
        null();
    }

    return;
}

/*-------------------------------------------------------------------------*/
static int8_t SendUARTFrame(const void *p_vData, uint8_t bySize, COMM_FRAME_TYPE_TypeDef eType)
{
    COMM_FRAME_HEAD    *p_stHead;
    COMM_FRAME_TAIL    *p_stTail;

    if (MAX_LEN_UART_FRAME_DATA < bySize)
    {
        return -1; /* Bad size of data. */
    }

    /* Make head of frame. */
    p_stHead = (COMM_FRAME_HEAD *)&s_abyTxUARTFrame[0];
    p_stHead->byHead = COMM_TRM_HEAD;
    p_stHead->eType = eType;
    p_stHead->byDataSize = bySize;

    /* Copy payload into body of frame. */
    if (p_vData && (0 < bySize))
    {
        memcpy(&s_abyTxUARTFrame[sizeof(COMM_FRAME_HEAD)], p_vData, bySize);
    }

    /* Make tail of frame. */
    p_stTail = (COMM_FRAME_TAIL *)&s_abyTxUARTFrame[sizeof(COMM_FRAME_HEAD) + bySize];
    p_stTail->byCS = CalcCS(s_abyTxUARTFrame, bySize + sizeof(COMM_FRAME_HEAD));
    p_stTail->byTail = COMM_TRM_TAIL;

    /* Send this UART frame to RNDU470T */
    cpc_Tx( s_abyTxUARTFrame, 
                 bySize + sizeof(COMM_FRAME_HEAD) + sizeof(COMM_FRAME_TAIL));
    
    return 0;
}

/*-------------------------------------------------------------------------*/
static void InitRTCEnableWakeUpInt(void)
{
    /* Configure RTC */
    CLK_RTCClockConfig(CLK_RTCCLKSource_LSI, CLK_RTCCLKDiv_1);
    CLK_PeripheralClockConfig(CLK_Peripheral_RTC, ENABLE);

    RTC_WakeUpClockConfig(RTC_WakeUpClock_CK_SPRE_16bits);
    RTC_ITConfig(RTC_IT_WUT, ENABLE);

    RTC_SetWakeUpCounter(10); /* 10s */
    RTC_WakeUpCmd(ENABLE);

    /* Save more energy when enter Halt mode. */
    PWR_UltraLowPowerCmd(ENABLE);

    return;
}

/*-------------------------------------------------------------------------*/
static void InitMCUClock(void)
{
    int16_t    nCnt;

    /* Set high speed internal clock prescaler=1(16MHz) */
    CLK_SYSCLKDivConfig(CLK_SYSCLKDiv_1);

    /* Enable LSI(Internal Low Speed oscillator) clock for IWDG */
    CLK_LSICmd(ENABLE);
    nCnt = 0x7FFF; /* Wait for LSI clock to be ready */
    while ((RESET == CLK_GetFlagStatus(CLK_FLAG_LSIRDY)) && (nCnt > 0))
    {
        --nCnt;
    }
  
    return;
}

/*-------------------------------------------------------------------------*/
static int8_t GetHumiTemp(void)
{
    float    dew_point;
    unsigned char    error,checksum;
    sht_value    humi_val,temp_val;

    error = 0;
    error += s_measure((unsigned char*) &humi_val.i, &checksum, HUMI);  //measure humidity
    error += s_measure((unsigned char*) &temp_val.i, &checksum, TEMP);  //measure temperature

    if (0 != error)
    {
        s_connectionreset(); //in case of an error: connection reset
        return error;
    }

    /* Convert temperature and humidity to human friendly. */
    humi_val.f = (float)humi_val.i; //converts integer to float
    temp_val.f = (float)temp_val.i; //converts integer to float
    calc_sth11(&humi_val.f, &temp_val.f); //calculate humidity, temperature
    dew_point = calc_dewpoint(humi_val.f,temp_val.f); //calculate dew point

    /* Send temperature and humidity to SINK. */
    s_stTempHumi.fTemp = temp_val.f; /* temperature */
    s_stTempHumi.fHumi = humi_val.f; /* humidity */
    s_stTempHumi.fDewPoint = dew_point; /* dew point */

    return 0;
}

/*-------------------------------------------------------------------------*/
static bool SendHumiTemp(COMM_FRAME_TYPE_TypeDef tFrameType)
{
    rtimer_clock_t    tTimeEnd = 0;

    GPIO_SetBits(UPLINK_LED_IOPORT, UPLINK_LED_PIN);

    /* MUST set FALSE before Send UART frame, otherwise incurred race condition! */
    s_bRxUARTFrame = FALSE;
    
    /* Send the hum_temp to RNDU470T by UART port. */
    SendUARTFrame(&s_stTempHumi, sizeof(s_stTempHumi), tFrameType);

    /* Wait the responsed frame from RNDU470T. */
    tTimeEnd = RTIMER_NOW() + 100; /* 100ms > 1000Bytes at 115200 */
    while ((!s_bRxUARTFrame) && RTIMER_CLOCK_LT(RTIMER_NOW(), tTimeEnd))
    {
        null();
    }

    DelayMs(100); /* for LED on more long */
    GPIO_ResetBits(UPLINK_LED_IOPORT, UPLINK_LED_PIN);

    if (s_bRxUARTFrame)
    {
        /* For RX next frame */
        ClearCommFrame();
        return TRUE;
    }
    else
    {
        return FALSE;
    }
}

/*-------------------------------------------------------------------------*/
static void GetLoRaNodeVersion(void)
{
    const char    *p_chRespStr;
    uint8_t    byRespSize;
    rtimer_clock_t    tTimeEnd;

    static int8_t    s_chTryCnt = 0;

    do {    
        /* MUST set FALSE before Send UART frame, otherwise incurred race condition! */
        s_bRxUARTFrame = FALSE;

        SendUARTFrame(NULL, 0, TYPE_GET_VER);
        ++s_chTryCnt;

        tTimeEnd = RTIMER_NOW() + 100; /* 100ms > 1000Bytes at 115200 */
        while ((!s_bRxUARTFrame) && RTIMER_CLOCK_LT(RTIMER_NOW(), tTimeEnd))
        {
            null();
        }

        if (s_bRxUARTFrame)
        {
            p_chRespStr = (const char *)&s_stComm2TrmData.a_byRxBuf[sizeof(COMM_FRAME_HEAD)];
            byRespSize = strlen(p_chRespStr) + 1; /* Add 1 for '\0' */
            dp_Tx(p_chRespStr, byRespSize);

            /* Now, we can judge the node is TDMA or LoRaWAN! */
            if ('T' == p_chRespStr[7]) /* TDMA */
            {
                cpc_SetNodeMode(TRUE);
            }
            else /* LoRaWAN */
            {
                cpc_SetNodeMode(FALSE);
            }

            /* For RX next frame */
            ClearCommFrame();
            s_chTryCnt = 0;

            return;
        }
        else
        {
            if (10 <= s_chTryCnt)
            {
                /* The node is TDMA or LoRaWAN? Toggle mode and try it again! */
                cpc_ToggleNodeMode();
                s_chTryCnt = 0;
            }
        }
    } while (TRUE);
}

/*-------------------------------------------------------------------------*/
#if EN_TEST_NODE_UART
static void GetNetSettings(void)
{
    int8_t    chCnt;
    rtimer_clock_t    tTimeEnd;
    static int8_t    s_chMaxTry = 0;
    static int32_t    s_lTxCnt = 0;
    static int32_t    s_lRxCnt = 0;
    static int32_t    s_lFailedCnt = 0;
    static char    s_achPrintBuf[64];

#if 0 /* Change RF speed of NODE. */
    static const uint8_t    s_abySetBps7[] = 
    {0x3C, 0x03, 0x09, 0x55, 0xAA, 0x07, 0x1C, 0x03, 0xA1, 0x80, 0x6F, 0x3C, 0x39, 0x0D};

    while (TRUE)
    {
        /* MUST set FALSE before Send UART frame, otherwise incurred race condition! */
        s_bRxUARTFrame = FALSE;

        /* Send request command. */
        cpc_Tx(s_abySetBps7, sizeof(s_abySetBps7));

        /* Waiting responsed of net settings. */
        tTimeEnd = RTIMER_NOW() + 50; /* wait 50ms */
        while ((!s_bRxUARTFrame) && RTIMER_CLOCK_LT(RTIMER_NOW(), tTimeEnd))
        {
            null();
        }

        /* Print count of request and response. */
        if (s_bRxUARTFrame)
        {
            break; /* Break from "while(TRUE)" */
        }
    }
#endif

#if 1 /* Test JiKang debug. */
    static const uint8_t   s_abyTestData[] =
    {0x00, 0x00, 0x00, 0x12, 0x7F, 0x45, 0x36, 0x1B, 0xE8, 0x45, 0x5D, 0x44, 0xC2, 0x40, 0x39};

    while (1)
    {
        /* Waiting until received a wake frame from SINK. */
        s_bRxUARTFrame = FALSE;
        while (!s_bRxUARTFrame)
        {
            nop();
        }
        ClearCommFrame(); /* For RX next frame */

        /* Delay 3000ms */
        DelayMs(3000);

        for (chCnt = 0; chCnt < 10; ++chCnt)
        {
            /* MUST set FALSE before Send UART frame, otherwise incurred race condition! */
            s_bRxUARTFrame = FALSE;

            /* Send a test frame to NODE. */
            SendUARTFrame(s_abyTestData, sizeof(s_abyTestData), TYPE_TX_RF_DATA);
            ++s_lTxCnt;

            /* Waiting responsed of net settings. */
            tTimeEnd = RTIMER_NOW() + 50; /* wait 50ms */
            while ((!s_bRxUARTFrame) && RTIMER_CLOCK_LT(RTIMER_NOW(), tTimeEnd))
            {
                null();
            }

            /* Check whether received "TX OK" from NODE. */
            if (s_bRxUARTFrame)
            {
                ++s_lRxCnt;
                ClearCommFrame(); /* For RX next frame */
                break; /* Break from "for()" */
            }
            else
            {
                ++s_lFailedCnt;
            }
        }

        if (s_chMaxTry < chCnt)
        {
            s_chMaxTry = chCnt;
        }

        if (0 == s_lTxCnt % 1)
        {
            snprintf( s_achPrintBuf, 
                          sizeof(s_achPrintBuf), 
                          "Tx=%ld, Rx=%ld, Failed=%ld, MaxTry=%d\r\n", 
                          s_lTxCnt, s_lRxCnt, s_lFailedCnt, s_chMaxTry );
            dp_Tx(s_achPrintBuf, strlen(s_achPrintBuf));
        }
    }
#endif

}
#endif

/*-------------------------------------------------------------------------*/
static void TestLed(void)
{
    int8_t i;

    GPIO_ResetBits(LED1_IOPORT, LED1_PIN);
    GPIO_ResetBits(LED2_IOPORT, LED2_PIN);
    GPIO_ResetBits(LED3_IOPORT, LED3_PIN);
    GPIO_ResetBits(LED4_IOPORT, LED4_PIN);
    
    for(i = 0; i < 1; i++)
    {
        GPIO_SetBits(LED1_IOPORT, LED1_PIN);
        DelayMs(100);
        GPIO_ResetBits(LED1_IOPORT, LED1_PIN);

        GPIO_SetBits(LED2_IOPORT, LED2_PIN);
        DelayMs(100);
        GPIO_ResetBits(LED2_IOPORT, LED2_PIN);

        GPIO_SetBits(LED3_IOPORT, LED3_PIN);
        DelayMs(100);
        GPIO_ResetBits(LED3_IOPORT, LED3_PIN);

        GPIO_SetBits(LED4_IOPORT, LED4_PIN);
        DelayMs(100);
        GPIO_ResetBits(LED4_IOPORT, LED4_PIN);
    }
}

/*-------------------------------------------------------------------------*/
static bool AtomicTestClearBit(Atomic_t *p_tAtomic, uint8_t byEventBit)
{
    bool    bIsSet;
    halIntState_t    intState;

    ASSERT(p_tAtomic);

    HAL_ENTER_CRITICAL_SECTION(intState);
    if (*p_tAtomic & byEventBit)
    {
        *p_tAtomic &= ~byEventBit;
        bIsSet = TRUE;
    }
    else
    {
        bIsSet = FALSE;
    }
    HAL_EXIT_CRITICAL_SECTION(intState);

    return bIsSet;
}

/*-------------------------------------------------------------------------*/
static void AtomicSetBit(Atomic_t *p_tAtomic, uint8_t byEventBit)
{
    halIntState_t    intState;

    HAL_ENTER_CRITICAL_SECTION(intState);
    *p_tAtomic |= byEventBit;
    HAL_EXIT_CRITICAL_SECTION(intState);

    return;
}

/*-------------------------------------------------------------------------*/
void RTCIRQHandler(void)
{
    s_tSystEvent |= SYST_EVENT_RTC;

    return;
}

/*-------------------------------------------------------------------------*/
void KeySetEvent(void)
{
    s_tSystEvent |= SYST_EVENT_KEY;

    return;
}

/**
  * @brief  This is the 'standard' C startup entry point.
  * @note  None. 
  * @param  None.
  * @retval  main() should NEVER return
  */
void main(void)
{
    /* Initialize the MCU and peripherals. */
    InitMCUClock();
    GPIO_LowPower_Init();
    rtimer_arch_init();
    cpc_Init();
    key_Init();
    led_Init();
    dp_Init();

    /* EXPLAIN: enable INT before initialize process! */
    HAL_ENABLE_INTERRUPTS();

#if EN_TEST_NODE_UART
    GetNetSettings();
#endif

    /* Initialize the SHT1x */
    sht7x_Init();

    INIT_RESET_NODE_PIN();

    /* Wait forever until the Node at work by got version from it. */
    GetLoRaNodeVersion();

    /* Initialize RTC and enable wake up per 1 second. */
    InitRTCEnableWakeUpInt();

    while (1)
    {
        /* Enter low power mode until wake up by events. */
        wfi();

        /* Process all events. */
        while (s_tSystEvent)
        {
            if (AtomicTestClearBit(&s_tSystEvent, SYST_EVENT_RTC)) /* By RTC */
            {
                if(0 == GetHumiTemp())
                {
                    SendHumiTemp(TYPE_TX_RF_DATA);
                }
            }
            else if (AtomicTestClearBit(&s_tSystEvent, SYST_EVENT_KEY)) /* By KEY */
            {
                GPIO_SetBits(KEY_LED_IOPORT, KEY_LED_PIN);
                if(0 == GetHumiTemp())
                {
                    SendHumiTemp(TYPE_TX_RF_DATA);
                }
                DelayMs(100); /* for LED on more long */
                GPIO_ResetBits(KEY_LED_IOPORT, KEY_LED_PIN);
            }
            else if (AtomicTestClearBit(&s_tSystEvent, SYST_EVENT_GATEWAY)) /* By GATEWAY */
            {
                TestLed();
                if(0 == GetHumiTemp())
                {
                    SendHumiTemp(TYPE_WAKE_ACK);
                }
                ClearCommFrame(); /* For RX next frame */
            }
            else /* Catch error */
            {
                GPIO_SetBits(ALARM_LED_IOPORT, ALARM_LED_PIN);
                AtomicSetBit(&s_tSystEvent, 0); /* Avoid dead cycle. */
                DelayMs(100); /* for LED on more long */
                GPIO_ResetBits(ALARM_LED_IOPORT, ALARM_LED_PIN);
                ASSERT(!"Error: MCU been waked up but event is none!\r\n");
            }
        }
    }
}


/*--------------------------------------------------------------------------
                                                                       0ooo
                                                           ooo0       (   )
                                                            (   )      ) /
                                                             \ (      (_/
                                                              \_)
----------------------------------------------------------------------------*/

