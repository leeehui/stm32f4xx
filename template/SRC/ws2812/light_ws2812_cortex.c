/*
 * light weight WS2812 lib - ARM Cortex M0/M0+ version
 *
 * Created: 07.07.2013
 *  Author: Tim (cpldcpu@gmail.com)
 */

#include "light_ws2812_cortex.h"

/*
* The total length of each bit is 1.25µs (25 cycles @ 20Mhz)
* At 0µs the dataline is pulled high.
* To send a zero the dataline is pulled low after 0.375µs
* To send a one the dataline is pulled low after 0.625µs
*/

//delay 1 nanosecond needs (ws2812_cpuclk/1000)/1000000 cpu clcle
#define ws2812_ctot	(((ws2812_cpuclk/1000)*1250)/1000000) //1.25μs = 1250ns
#define ws2812_t1	(((ws2812_cpuclk/1000)*375 )/1000000)	  // floor
#define ws2812_t2	(((ws2812_cpuclk/1000)*625)/1000000)    // ceil

//ZERO
//  T0______T1       T2        T3
//  |        |                 |
//   |        |_________________|

//ONE
//  T0______T1_______T2        T3
//  |                 |        |
//  |                 |________|

//　T0 : set high, wait "w1" cpu cycles
//  T1 : set low if sending zero, keep unchanged if sending one, wait "w2" cycles
//  T2 : set low unconditionally, wait "w3" cycles
//  T3 : T0

//considering the time that occupied by instructions between T0 and T1(T1 and T2, T2 and T3(T0))
//Note: we assume that the performance is 1 cpu cycle per instruction.
//      according to spec, this cpu has 1.25 DMIPS/MHz (Dhrystone 2.1) performance at 0 wait state memory access.
//      so theoretically less than 1 cpu cycle per instruction.
//      when cpu runs faster, there will be less affect to the timing, and the assumption is acceptable for timing
#define w1 (ws2812_t1-2)
#define w2 (ws2812_t2-ws2812_t1-2)
#define w3 (ws2812_ctot-ws2812_t2-5)

#define ws2812_DEL1 nop;
#define ws2812_DEL2 nop; nop;
#define ws2812_DEL4 ws2812_DEL2; ws2812_DEL2
#define ws2812_DEL8 ws2812_DEL4; ws2812_DEL4
#define ws2812_DEL16 ws2812_DEL8; ws2812_DEL8
#define ws2812_DEL32 ws2812_DEL16; ws2812_DEL16
#define ws2812_DEL64 ws2812_DEL32; ws2812_DEL32
#define ws2812_DEL128 ws2812_DEL64; ws2812_DEL64


					

void ws2812_config_gpio(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;

  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
  
  /* Configure MCO1 pin(PA8) in alternate function */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
  GPIO_InitStructure.GPIO_Speed = GPIO_High_Speed;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;  
  GPIO_Init(GPIOB, &GPIO_InitStructure);
}

/*
 * Important Note:
 * 1. Do not use optimization (use default or O(0))
 * 2. lsl can not change C flag, use lsls instead, this is why "bcs" can not compiled when using lsl
 */
void ws2812_sendarray_armcc(uint8_t *data,int datlen)
{
	uint32_t maskhi = ws2812_mask_set;
	uint32_t masklo = ws2812_mask_clr;
	volatile uint32_t *set = ws2812_port_set;
	volatile uint32_t *clr = ws2812_port_clr;
	uint32_t i;
	uint32_t curbyte;

	//__set_PRIMASK(1);
	while (datlen--) {
		curbyte=*data++;
		__ASM {
			lsl curbyte, #24         //curbyte left shift 24bits
			movs i, #8               //set i to 8
			iloop:                   //main loop
			lsls curbyte, #1         //left shift curbyte by #1, subfix "s" tell processor changing the "C" flag
			str maskhi, [set]        //T0(T3) : set output high   
#if (w1&1)                     //nop delay
			ws2812_DEL1
#endif
#if (w1&2)
			ws2812_DEL2
#endif
#if (w1&4)
			ws2812_DEL4
#endif
#if (w1&8)
			ws2812_DEL8
#endif
#if (w1&16)
			ws2812_DEL16
#endif
#if (w1&32)
			ws2812_DEL32
#endif
#if (w1&64)
			ws2812_DEL64
#endif
#if (w1&128)
			ws2812_DEL128
#endif

			bcs aaaa                 //skip set output low if MSB is 1, see "lsls" in <<CortexM3ProgrammingManual>>
			str masklo, [clr]        //T1 : set output low    
			aaaa:		                 //nop delay
#if (w2&1)
			ws2812_DEL1
#endif
#if (w2&2)
			ws2812_DEL2
#endif
#if (w2&4)
			ws2812_DEL4
#endif
#if (w2&8)
			ws2812_DEL8
#endif
#if (w2&16)
			ws2812_DEL16
#endif
#if (w2&32)
			ws2812_DEL32
#endif
#if (w2&64)
			ws2812_DEL64
#endif
#if (w2&128)
			ws2812_DEL128
#endif

			subs i, #1               //sub i by 1
			str masklo, [clr]        //T2 : unconditionally set output low, Note: str do not change flags
#if (w3&1)
			ws2812_DEL1
#endif
#if (w3&2)
			ws2812_DEL2
#endif
#if (w3&4)
			ws2812_DEL4
#endif
#if (w3&8)
			ws2812_DEL8
#endif
#if (w3&16)
			ws2812_DEL16
#endif
#if (w3&32)
			ws2812_DEL32
#endif
#if (w3&64)
			ws2812_DEL64
#endif
#if (w3&128)
			ws2812_DEL128
#endif
			beq end                  //if i == 0 jump out iloop
			b iloop                  //if program runs here, unconditionall jump to iloop
			end:
		}
	}	
	//__set_PRIMASK(0);
}

//void ws2812_sendarray_gnucc(uint8_t *data,int datlen)
//{
//	uint32_t maskhi = ws2812_mask_set;
//	uint32_t masklo = ws2812_mask_clr;
//	volatile uint32_t *set = ws2812_port_set;
//	volatile uint32_t *clr = ws2812_port_clr;
//	uint32_t i; 
//	uint32_t curbyte;

//	while (datlen--) {
//		curbyte=*data++;

//	__asm volatile(
//			"		lsl %[dat],#24				\n\t"
//			"		movs %[ctr],#8				\n\t"
//			"ilop%=:							\n\t"
//			"		lsl %[dat], #1				\n\t"
//			"		str %[maskhi], [%[set]]		\n\t"
//#if (w1&1)
//			ws2812_DEL1
//#endif
//#if (w1&2)
//			ws2812_DEL2
//#endif
//#if (w1&4)
//			ws2812_DEL4
//#endif
//#if (w1&8)
//			ws2812_DEL8
//#endif
//#if (w1&16)
//			ws2812_DEL16
//#endif
//			"		bcs one%=					\n\t"
//			"		str %[masklo], [%[clr]]		\n\t"
//			"one%=:								\n\t"
//#if (w2&1)
//			ws2812_DEL1
//#endif
//#if (w2&2)
//			ws2812_DEL2
//#endif
//#if (w2&4)
//			ws2812_DEL4
//#endif
//#if (w2&8)
//			ws2812_DEL8
//#endif
//#if (w2&16)
//			ws2812_DEL16
//#endif
//			"		sub %[ctr], #1				\n\t"
//			"		str %[masklo], [%[clr]]		\n\t"
//			"		beq	end%=					\n\t"
//#if (w3&1)
//			ws2812_DEL1
//#endif
//#if (w3&2)
//			ws2812_DEL2
//#endif
//#if (w3&4)
//			ws2812_DEL4
//#endif
//#if (w3&8)
//			ws2812_DEL8
//#endif
//#if (w3&16)
//			ws2812_DEL16
//#endif

//			"		b 	ilop%=					\n\t"
//			"end%=:								\n\t"
//			:	[ctr] "+r" (i)
//			:	[dat] "r" (curbyte), [set] "r" (set), [clr] "r" (clr), [masklo] "r" (masklo), [maskhi] "r" (maskhi)
//			);
//	}
//}

