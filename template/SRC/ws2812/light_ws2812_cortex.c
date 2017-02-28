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
#define ws2812_ctot	(((ws2812_cpuclk/1000)*1250)/1000000) //1.25μs = 1250ns  ws2812_ctot = 168 * 1.25 = 210    
#define ws2812_t1	(((ws2812_cpuclk/1000)*375 )/1000000)	  // floor       ws2812_t1 = 168 * 0.375 = 63
#define ws2812_t2	(((ws2812_cpuclk/1000)*625)/1000000)    // ceil          ws2812_t2 = 168 * 0.625 = 105

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
#define w1_single (ws2812_t1-2)
#define w2_single (ws2812_t2-ws2812_t1-2)
#define w3_single (ws2812_ctot-ws2812_t2-5)

#define w1_multi (ws2812_t1-40) //23
#define w2_multi (ws2812_t2-ws2812_t1-30)//52
#define w3_multi (ws2812_ctot-ws2812_t2-95) //63


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
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5;
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
#if (w1_single&1)                     //nop delay
			ws2812_DEL1
#endif
#if (w1_single&2)
			ws2812_DEL2
#endif
#if (w1_single&4)
			ws2812_DEL4
#endif
#if (w1_single&8)
			ws2812_DEL8
#endif
#if (w1_single&16)
			ws2812_DEL16
#endif
#if (w1_single&32)
			ws2812_DEL32
#endif
#if (w1_single&64)
			ws2812_DEL64
#endif
#if (w1_single&128)
			ws2812_DEL128
#endif

			bcs aaaa                 //skip set output low if MSB is 1, see "lsls" in <<CortexM3ProgrammingManual>>
			str masklo, [clr]        //T1 : set output low    
			aaaa:		                 //nop delay
#if (w2_single&1)
			ws2812_DEL1
#endif
#if (w2_single&2)
			ws2812_DEL2
#endif
#if (w2_single&4)
			ws2812_DEL4
#endif
#if (w2_single&8)
			ws2812_DEL8
#endif
#if (w2_single&16)
			ws2812_DEL16
#endif
#if (w2_single&32)
			ws2812_DEL32
#endif
#if (w2_single&64)
			ws2812_DEL64
#endif
#if (w2_single&128)
			ws2812_DEL128
#endif

			subs i, #1               //sub i by 1
			str masklo, [clr]        //T2 : unconditionally set output low, Note: str do not change flags
#if (w3_single&1)
			ws2812_DEL1
#endif
#if (w3_single&2)
			ws2812_DEL2
#endif
#if (w3_single&4)
			ws2812_DEL4
#endif
#if (w3_single&8)
			ws2812_DEL8
#endif
#if (w3_single&16)
			ws2812_DEL16
#endif
#if (w3_single&32)
			ws2812_DEL32
#endif
#if (w3_single&64)
			ws2812_DEL64
#endif
#if (w3_single&128)
			ws2812_DEL128
#endif
			beq end                  //if i == 0 jump out iloop
			b iloop                  //if program runs here, unconditionall jump to iloop
			end:
		}
	}	
	//__set_PRIMASK(0);
}



/*
 * Important Note:
 * 1. Do not use optimization (use default or O(0))
 * 2. lsl can not change C flag, use lsls instead, this is why "bcs" can not compiled when using lsl
 */
void ws2812_send_multi_ch_armcc(uint8_t *data, uint8_t *data1,uint8_t *data2,uint8_t *data3,uint8_t *data4,uint8_t *data5,int datlen)
{
	uint32_t maskhi = ws2812_mask_set;
	uint32_t masklo = ws2812_mask_clr;
	uint32_t maskhi1 = ws2812_mask_set1;
	uint32_t masklo1 = ws2812_mask_clr1;
	uint32_t maskhi2 = ws2812_mask_set2;
	uint32_t masklo2 = ws2812_mask_clr2;
	uint32_t maskhi3 = ws2812_mask_set3;
	uint32_t masklo3 = ws2812_mask_clr3;
	uint32_t maskhi4 = ws2812_mask_set4;
	uint32_t masklo4 = ws2812_mask_clr4;
	uint32_t maskhi5 = ws2812_mask_set5;
	uint32_t masklo5 = ws2812_mask_clr5;

	volatile uint32_t *set = ws2812_port_set;
	volatile uint32_t *clr = ws2812_port_clr;
	volatile uint32_t *set1 = ws2812_port_set1;
	volatile uint32_t *clr1 = ws2812_port_clr1;
	volatile uint32_t *set2 = ws2812_port_set2;
	volatile uint32_t *clr2 = ws2812_port_clr2;
	volatile uint32_t *set3 = ws2812_port_set3;
	volatile uint32_t *clr3 = ws2812_port_clr3;
	volatile uint32_t *set4 = ws2812_port_set4;
	volatile uint32_t *clr4 = ws2812_port_clr4;
	volatile uint32_t *set5 = ws2812_port_set5;
	volatile uint32_t *clr5 = ws2812_port_clr5;


	uint32_t i;
	uint32_t flag;
	uint32_t curbyte,curbyte1,curbyte2,curbyte3,curbyte4,curbyte5;

	//__set_PRIMASK(1);
	while (datlen--) {
		curbyte=*data++;
		curbyte1=*data1++;
		curbyte2=*data2++;
		curbyte3=*data3++;
		curbyte4=*data4++;
		curbyte5=*data5++;
		__ASM {
			movs i, #0x80               //set i to 1000 0000
			iloop:                   //main loop
			
			movs flag, #0            //reset flag

			tst curbyte, i           //curbyte & i, update cpu flags
			it eq                    //if z=1(result is 0, which means bit is 0. need set low at T1)
			orreq flag, #0x01        //channel 1 save flag bit 
			tst curbyte1, i
			it eq
			orreq flag, #0x02        //channel 2
			tst curbyte2, i
			it eq
			orreq flag, #0x04        //channel 3
			tst curbyte3, i
			it eq
			orreq flag, #0x08        //channel 4
			tst curbyte4, i
			it eq
			orreq flag, #0x10        //channel 5
			tst curbyte5, i
			it eq
			orreq flag, #0x20        //channel 6

			str maskhi, [set]        //T0(T3) : set output high   
			str maskhi1, [set1]
			str maskhi2, [set2]
			str maskhi3, [set3]
			str maskhi4, [set4]
			str maskhi5, [set5]
#if (w1_multi&1)                     //nop delay
			ws2812_DEL1
#endif
#if (w1_multi&2)
			ws2812_DEL2
#endif
#if (w1_multi&4)
			ws2812_DEL4
#endif
#if (w1_multi&8)
			ws2812_DEL8
#endif
#if (w1_multi&16)
			ws2812_DEL16
#endif
#if (w1_multi&32)
			ws2812_DEL32
#endif
#if (w1_multi&64)
			ws2812_DEL64
#endif
#if (w1_multi&128)
			ws2812_DEL128
#endif

			tst flag, #0x01            //test flag bit 
			it ne                      //test if bit 1 is 1, which means result is not 0
			strne masklo, [clr]        //T1 : set output low  

			tst flag, #0x02
			it ne
			strne masklo1, [clr1]
			
			tst flag, #0x04
			it ne
			strne masklo2, [clr2]

			tst flag, #0x08
			it ne
			strne masklo3, [clr3]

			tst flag, #0x10
			it ne
			strne masklo4, [clr4]

			tst flag, #0x20
			it ne
			strne masklo5, [clr5]

        				             
#if (w2_multi&1)                            //nop delay
			ws2812_DEL1
#endif
#if (w2_multi&2)
			ws2812_DEL2
#endif
#if (w2_multi&4)
			ws2812_DEL4
#endif
#if (w2_multi&8)
			ws2812_DEL8
#endif
#if (w2_multi&16)
			ws2812_DEL16
#endif
#if (w2_multi&32)
			ws2812_DEL32
#endif
#if (w2_multi&64)
			ws2812_DEL64
#endif
#if (w2_multi&128)
			ws2812_DEL128
#endif

			lsrs i, #1               //right shift by 1,  sub i by 1
			str masklo, [clr]        //T2 : unconditionally set output low, Note: str do not change flags
			str masklo1, [clr1]
			str masklo2, [clr2]
			str masklo3, [clr3]
			str masklo4, [clr4]
			str masklo5, [clr5]
#if (w3_multi&1)
			ws2812_DEL1
#endif
#if (w3_multi&2)
			ws2812_DEL2
#endif
#if (w3_multi&4)
			ws2812_DEL4
#endif
#if (w3_multi&8)
			ws2812_DEL8
#endif
#if (w3_multi&16)
			ws2812_DEL16
#endif
#if (w3_multi&32)
			ws2812_DEL32
#endif
#if (w3_multi&64)
			ws2812_DEL64
#endif
#if (w3_multi&128)
			ws2812_DEL128
#endif
			beq end                  //if i == 0 jump out iloop
			b iloop                  //if program runs here, unconditionall jump to iloop
			end:
		}
	}	
	//__set_PRIMASK(0);
}



void led_demo(void)
{
	uint8_t rgb[6*3] = {
							0x33, 0x0, 0x0,
							0x0, 0x33, 0x0,
							0x0, 0x0, 0x33,
							0x33, 0x0, 0x0,
							0x0, 0x33, 0x0,
							0x0, 0x0, 0x33};
	uint8_t rgb1[6*3] = {
							0xFF, 0x0, 0x0,
							0x0, 0xFF, 0x0,
							0x0, 0x0, 0xFF,
							0xFF, 0x0, 0x0,
							0x0, 0xFF, 0x0,
							0x0, 0x0, 0xFF};

	uint8_t rgb2[6*3] = {
							0xFF, 0x0, 0x0,
							0x0, 0xFF, 0x0,
							0x0, 0x0, 0xFF,
							0xFF, 0x0, 0x0,
							0x0, 0xFF, 0x0,
							0x0, 0x0, 0xFF};

	uint8_t rgb3[6*3] = {
							0xFF, 0x0, 0x0,
							0x0, 0xFF, 0x0,
							0x0, 0x0, 0xFF,
							0xFF, 0x0, 0x0,
							0x0, 0xFF, 0x0,
							0x0, 0x0, 0xFF};
	uint8_t rgb4[6*3] = {
							0xFF, 0x0, 0x0,
							0x0, 0xFF, 0x0,
							0x0, 0x0, 0xFF,
							0xFF, 0x0, 0x0,
							0x0, 0xFF, 0x0,
							0x0, 0x0, 0xFF};

	uint8_t rgb5[6*3] = {
							0xFF, 0x0, 0x0,
							0x0, 0xFF, 0x0,
							0x0, 0x0, 0xFF,
							0xFF, 0x0, 0x0,
							0x0, 0xFF, 0x0,
							0x0, 0x0, 0xFF};



	while(1)
	{
		for(int i = 0; i < 255; i++)
		{
			rgb[0]=i;
			rgb1[0]=i;
			rgb2[0]=i;
			rgb3[0]=i;
			rgb4[0]=i;
			rgb5[0]=i;

			ws2812_send_multi_ch_armcc(rgb,rgb1,rgb2,rgb3,rgb4,rgb5,3*3);
				delay_ms(1);
		}
		for(int i = 255; i > 0; i--)
		{
			rgb[0]=i;
			rgb1[0]=i;
			rgb2[0]=i;
			rgb3[0]=i;
			rgb4[0]=i;
			rgb5[0]=i;
			ws2812_send_multi_ch_armcc(rgb,rgb1,rgb2,rgb3,rgb4,rgb5,3*3);
				delay_ms(1);
		}
    }
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

