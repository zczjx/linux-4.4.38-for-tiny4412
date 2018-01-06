/*******************************************************************************
* Copyright (C), 2000-2017,  Electronic Technology Co., Ltd.
*                
* @filename: exynos4412_pwm_timer_regs.h 
*                
* @author: Clarence.Zhou <zhou_chenz@163.com> 
*                
* @version:
*                
* @date: 2017-11-26    
*                
* @brief:          
*                  
*                  
* @details:        
*                 
*    
*    
* @comment           
*******************************************************************************/
#ifndef _EXYNOS4412_PWM_TIMER_REGS_H_
#define _EXYNOS4412_PWM_TIMER_REGS_H_

#define TIMER_SUB_REG(tmr,reg) ((reg)+0x0c+((tmr)*0x0c))

#define REG_TCFG0	      (0x00)
#define REG_TCFG1	      (0x04)
#define REG_TCON	      (0x08)

#define REG_TINT_CSTAT    (0x44)

#define TCFG_PRESCALER0_MASK (255<<0)
#define TCFG_PRESCALER1_MASK (255<<8)
#define TCFG_PRESCALER1_SHIFT (8)
#define TCFG_DEADZONE_MASK   (255<<16)
#define TCFG_DEADZONE_SHIFT  (16)

#define TCFG1_MUX4_DIV2	  (0<<16)
#define TCFG1_MUX4_DIV4	  (1<<16)
#define TCFG1_MUX4_DIV8	  (2<<16)
#define TCFG1_MUX4_DIV16  (3<<16)
#define TCFG1_MUX4_TCLK1  (4<<16)
#define TCFG1_MUX4_MASK	  (15<<16)
#define TCFG1_MUX4_SHIFT  (16)

#define TCFG1_MUX3_DIV2	  (0<<12)
#define TCFG1_MUX3_DIV4	  (1<<12)
#define TCFG1_MUX3_DIV8	  (2<<12)
#define TCFG1_MUX3_DIV16  (3<<12)
#define TCFG1_MUX3_TCLK1  (4<<12)
#define TCFG1_MUX3_MASK	  (15<<12)


#define TCFG1_MUX2_DIV2	  (0<<8)
#define TCFG1_MUX2_DIV4	  (1<<8)
#define TCFG1_MUX2_DIV8	  (2<<8)
#define TCFG1_MUX2_DIV16  (3<<8)
#define TCFG1_MUX2_TCLK1  (4<<8)
#define TCFG1_MUX2_MASK	  (15<<8)


#define TCFG1_MUX1_DIV2	  (0<<4)
#define TCFG1_MUX1_DIV4	  (1<<4)
#define TCFG1_MUX1_DIV8	  (2<<4)
#define TCFG1_MUX1_DIV16  (3<<4)
#define TCFG1_MUX1_TCLK0  (4<<4)
#define TCFG1_MUX1_MASK	  (15<<4)

#define TCFG1_MUX0_DIV2	  (0<<0)
#define TCFG1_MUX0_DIV4	  (1<<0)
#define TCFG1_MUX0_DIV8	  (2<<0)
#define TCFG1_MUX0_DIV16  (3<<0)
#define TCFG1_MUX0_TCLK0  (4<<0)
#define TCFG1_MUX0_MASK	  (15<<0)

#define TCFG1_MUX_DIV2	  (0<<0)
#define TCFG1_MUX_DIV4	  (1<<0)
#define TCFG1_MUX_DIV8	  (2<<0)
#define TCFG1_MUX_DIV16   (3<<0)
#define TCFG1_MUX_TCLK    (4<<0)
#define TCFG1_MUX_MASK	  (15<<0)

#define TCFG1_MUX_DIV1	  (0<<0)
#define TCFG1_MUX_DIV2	  (1<<0)
#define TCFG1_MUX_DIV4	  (2<<0)
#define TCFG1_MUX_DIV8    (3<<0)
#define TCFG1_MUX_DIV16   (4<<0)
#define TCFG1_MUX_TCLK    (5<<0)  /* 3 sets of TCLK */
#define TCFG1_MUX_MASK	  (15<<0)

#define TCFG1_SHIFT(x)	  ((x) * 4)

/* for each timer, we have an count buffer, an compare buffer and
 * an observation buffer
*/

/* WARNING - timer 4 has no buffer reg, and it's observation is at +4 */

#define REG_TCNTB(tmr)    TIMER_SUB_REG(tmr, 0x00)
#define REG_TCMPB(tmr)    TIMER_SUB_REG(tmr, 0x04)
#define REG_TCNTO(tmr)    TIMER_SUB_REG(tmr, (((tmr) == 4) ? 0x04 : 0x08))

// #define REG_TCNTB(chan)			(0x0c + 12 * (chan))
// #define REG_TCMPB(chan)			(0x10 + 12 * (chan))


#define TCON_T4RELOAD	  (1<<22)
#define TCON_T4MANUALUPD  (1<<21)
#define TCON_T4START	  (1<<20)

#define TCON_T3RELOAD	  (1<<19)
#define TCON_T3INVERT	  (1<<18)
#define TCON_T3MANUALUPD  (1<<17)
#define TCON_T3START	  (1<<16)

#define TCON_T2RELOAD	  (1<<15)
#define TCON_T2INVERT	  (1<<14)
#define TCON_T2MANUALUPD  (1<<13)
#define TCON_T2START	  (1<<12)

#define TCON_T1RELOAD	  (1<<11)
#define TCON_T1INVERT	  (1<<10)
#define TCON_T1MANUALUPD  (1<<9)
#define TCON_T1START	  (1<<8)

#define TCON_T0DEADZONE	  (1<<4)
#define TCON_T0RELOAD	  (1<<3)
#define TCON_T0INVERT	  (1<<2)
#define TCON_T0MANUALUPD  (1<<1)
#define TCON_T0START	  (1<<0)


#endif 


