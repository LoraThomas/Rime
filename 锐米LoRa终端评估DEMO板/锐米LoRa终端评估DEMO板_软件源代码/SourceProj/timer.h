/**
 * \file
 *         timer.h
 * \description
 *         timer of STM8L151C8T6
 * \author
 *         XXXXXXXXXXXXXXX
 * \date
 *         2016-01-22 12:05
 * \copyright
 *         (c) RimeLink (www.rimelink.com) All Rights Reserved.
 */

 
#ifndef __RTIMER_ARCH_H__
#define __RTIMER_ARCH_H__


#define RTIMER_ARCH_SECOND    1000 /* 1kHz */

typedef unsigned short rtimer_clock_t;
#define RTIMER_CLOCK_LT(a,b)     ((signed short)((a)-(b)) < 0) /* TimeOut<=32768 */
#define RTIMER_NOW() rtimer_arch_now()


/**
 * \brief    Initialize the real timer
 * \return  none
 */
extern void rtimer_arch_init(void);

/**
 * \brief    Get the current clock time
 * \return  The current time
 *
 *            This function returns what the real-time module thinks
 *            is the current time. The current time is used to set
 *            the timeouts for real-time tasks.
 *
 * \hideinitializer
 */
extern rtimer_clock_t rtimer_arch_now(void);

/**
 * \brief      Set an interrupt to timeout event.
 * \param    rtimer_clock_t t    the quantity of timeout, unit is ms.
 *
 *              This function schedules a real-time task at a specified
 *              time in the future.
 *
 */
extern void rtimer_arch_schedule(rtimer_clock_t t);

/**
 * \brief    Disable interrupt of rtimer timeout.
 *
 *            This function disable interrupt of real timer for removing timer.
 */
extern void rtimer_arch_disable_irq(void);

/**
 * \brief    Enable interrupt of rtimer timeout.
 *
 *            This function enable interrupt of real timer for restarting timer.
 */
extern void rtimer_arch_enable_irq(void);

/**
 * \brief      turn on the real timer.
 * \param    none
 *
 *              This function would turn on the real timer.
 */
extern void rtimer_arch_TurnOn(void);

/**
 * \brief      turn off the real timer.
 * \param    none
 *
 *              This function would turn off the real timer for saved energy.
 */
extern void rtimer_arch_TurnOff(void);


#endif

/*--------------------------------------------------------------------------------------------------------
                   									     0ooo
                   								ooo0     (   )
                								(   )     ) /
                								 \ (     (_/
                								  \_)
----------------------------------------------------------------------------------------------------------*/
