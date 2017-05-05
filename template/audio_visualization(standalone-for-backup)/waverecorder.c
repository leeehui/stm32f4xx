/**
  ******************************************************************************
  * @file    Audio_playback_and_record/src/waverecorder.c 
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    28-October-2011
  * @brief   I2S audio program 
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */ 

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "pdm_filter.h"
#include "waverecorder.h" 

#include "light_ws2812_cortex.h"
#include "bsp_debug_usart.h"
#include "arm_math.h"

/** @addtogroup STM32F4-Discovery_Audio_Player_Recorder
* @{
*/ 

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* SPI Configuration defines */
#define SPI_SCK_PIN                       GPIO_Pin_10
#define SPI_SCK_GPIO_PORT                 GPIOB
#define SPI_SCK_GPIO_CLK                  RCC_AHB1Periph_GPIOB
#define SPI_SCK_SOURCE                    GPIO_PinSource10
#define SPI_SCK_AF                        GPIO_AF_SPI2

#define SPI_MOSI_PIN                      GPIO_Pin_3
#define SPI_MOSI_GPIO_PORT                GPIOC
#define SPI_MOSI_GPIO_CLK                 RCC_AHB1Periph_GPIOC
#define SPI_MOSI_SOURCE                   GPIO_PinSource3
#define SPI_MOSI_AF                       GPIO_AF_SPI2

#define AUDIO_REC_SPI_IRQHANDLER          SPI2_IRQHandler

/* Audio recording frequency in Hz */
#define REC_FREQ                          8000  

/* PDM buffer input size */
#define INTERNAL_BUFF_SIZE      64

/* PCM buffer output size */
#define PCM_OUT_SIZE            16

#define FFT_SIZE    PCM_OUT_SIZE
#define SAMPLE_SIZE         (FFT_SIZE*2)

float32_t input[SAMPLE_SIZE] ;
float32_t output[FFT_SIZE];
float32_t wave_max = 30000.0;
float32_t output_max = 200000.0;

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
extern __IO uint16_t Time_Rec_Base;
extern __IO uint8_t Command_index;
extern USB_OTG_CORE_HANDLE  USB_OTG_Core;
extern __IO uint32_t WaveCounter;
extern FIL file;
extern __IO uint8_t LED_Toggle;
uint16_t RAM_Buf[RAM_BUFFER_SIZE];
uint16_t RAM_Buf1 [RAM_BUFFER_SIZE];
uint16_t buf_idx = 0, buf_idx1 =0;
uint16_t *writebuffer;
uint16_t counter = 0;
uint8_t WaveRecStatus = 0;
/* Current state of the audio recorder interface intialization */
static uint32_t AudioRecInited = 0;
PDMFilter_InitStruct Filter;
/* Audio recording Samples format (from 8 to 16 bits) */
uint32_t AudioRecBitRes = 16; 
uint16_t RecBuf[PCM_OUT_SIZE], RecBuf1[PCM_OUT_SIZE];
uint8_t RecBufHeader[512], Switch = 0;
__IO uint32_t Data_Status =0;
/* Audio recording number of channels (1 for Mono or 2 for Stereo) */
uint32_t AudioRecChnlNbr = 1;
/* Main buffer pointer for the recorded data storing */
uint16_t* pAudioRecBuf;
/* Current size of the recorded buffer */
uint32_t AudioRecCurrSize = 0; 
uint16_t bytesWritten;
/* Temporary data sample */
static uint16_t InternalBuffer[INTERNAL_BUFF_SIZE];
static uint32_t InternalBufferSize = 0;

/* Private function prototypes -----------------------------------------------*/
static void WaveRecorder_GPIO_Init(void);
static void WaveRecorder_SPI_Init(uint32_t Freq);
static void WaveRecorder_NVIC_Init(void);

/* Private functions ---------------------------------------------------------*/

/*led configurations*/
typedef enum
{
	MODE_1 = 0,
	MODE_2,
	MODE_3,
	MODE_END
}mode_t;

static uint32_t is_led_configuration_updated = 0;

//
#define Q_ASSERT_COMPILE(test_) \
    extern int32_t Q_assert_compile[(test_) ? 1 : -1]

/*Important Note£º
	if all modules use same led num,  #define SAME_MODULE_SIZE 0	
	otherwise,                        #define SAME_MODULE_SIZE 1	
*/ 
#define SAME_MODULE_SIZE 0	
	
#define CH_NUM      6 /*total channel num of led controller*/
#define CH_MAX      (CH_NUM - 1) /*for buffer index protection*/

#define CH_LED_NUM  120/*total led num of one channel*/
#define CH_LED_MAX  (CH_LED_NUM - 1)/*for buffer index protection*/

#define MODULE_NUM       FFT_SIZE/*total led module num of led controller*/
#define MODULE_MAX      (MODULE_NUM - 1)/*for buffer index protection*/

	
#if SAME_MODULE_SIZE
	#define MODULE_LED_NUM(led_module)   30/*total led num of one led module*/
	#define MODULE_LED_MAX(led_module) (MODULE_LED_NUM(led_module) - 1)/*for buffer index protection*/

	//Note: all indexs start at 0, if led_module = 3, means the 4th module should be located in (3 * 30)/120 = 0(th) channel 
	#define GET_LED_CH(led_module) ((led_module * MODULE_LED_NUM)/CH_LED_NUM) 
	#define GET_LED_CH_POS(led_module, led_module_pos) (((led_module * MODULE_LED_NUM) % CH_LED_NUM) + led_module_pos)
	
	//led buffer size check
	Q_ASSERT_COMPILE((CH_LED_NUM % MODULE_LED_NUM) == 0);
	Q_ASSERT_COMPILE((CH_NUM * CH_LED_NUM) >= (MODULE_NUM * MODULE_LED_NUM));

#else
	#define MODULE_LED_NUM(led_module)   used_led_num_in_module[led_module]/*total led num of one led module*/
	#define MODULE_LED_MAX(led_module)  (MODULE_LED_NUM(led_module) - 1)/*for buffer index protection*/

	#define GET_LED_CH(led_module) ((sum_of_former_module[led_module])/CH_LED_NUM) 
	#define GET_LED_CH_POS(led_module, led_module_pos) (((sum_of_former_module[led_module]) % CH_LED_NUM) + led_module_pos)
	
	//security check will be done by calling update_led_configuration()
	
#endif
	
	
	
uint8_t led_buffer[CH_NUM][CH_LED_NUM*3] = {0} ;



uint32_t cur_target_pos[MODULE_NUM] = {0};
uint32_t cur_max_pos[MODULE_NUM] = {0};
uint32_t output_pos[MODULE_NUM] = {0};
uint32_t is_last_target_reached[MODULE_NUM] = {1};
uint32_t cur_pos[MODULE_NUM] = {0};
uint32_t last_pos[MODULE_NUM] = {0};
uint32_t max_pos[MODULE_NUM] = {0};

//used_led_num_in_module is defined because different module may have different size
//accurate pos is needed to calculated inside one channel because its serial property
static const uint32_t used_led_num_in_module[MODULE_NUM] = 
{
	30, //module 1
	30, //module 2
	30, //module 3
	30, //module 4
	30, //module 5
	
	30, //module 6
	30, //module 7
	30, //module 8
	30, //module 9
	30, //module 10
	
	30, //module 11
	30, //module 12
	30, //module 13
	30, //module 14
	30, //module 15
	
	30  //module 16
};

static uint32_t sum_of_former_module[MODULE_NUM];
static uint32_t max_led_num_of_modules;
static uint32_t min_led_num_of_modules;



static const uint32_t used_led_num_in_ch[CH_NUM] = 
{
	CH_LED_NUM - 0,
	CH_LED_NUM - 0,
	CH_LED_NUM - 0,
	CH_LED_NUM - 0,
	CH_LED_NUM - 0
};


typedef uint32_t color_t;

#define COLOR(r,g,b) ((color_t)((0xFF0000 & (r<<16)) | (0xFF00 & (g<<8)) | (0xFF & b) ))
#define GET_R(c) ((c>>16) & 0xFF)
#define GET_G(c) ((c>>8) & 0xFF)
#define GET_B(c) ((c) & 0xFF)


static const color_t  c_zero = COLOR(0,0,0);


void update_led(void);
void set_led_in_ch(uint8_t led_ch, uint32_t led_ch_pos, const color_t *c);
void set_led_in_module(uint8_t led_module, uint32_t led_module_pos,  const color_t *c);
void set_led_range_in_ch(uint8_t led_ch, uint32_t led_start, uint32_t led_end,  const color_t *c);
void set_led_range_in_module(uint8_t led_module, uint32_t led_start, uint32_t led_end,  const color_t *c);
void set_led_roll_in_ch(uint8_t led_ch, uint32_t roll_pos, uint32_t roll_led_num, const color_t *c_roll, const color_t *c_other);
void set_led_roll_in_module(uint8_t led_module, uint32_t roll_pos, uint32_t roll_led_num, const color_t *c_roll, const color_t *c_other);


void delay_ms(uint32_t delay);



void err_dispaly(void)
{
	
}

void update_sum_of_former_module(void)
{
	sum_of_former_module[0] = 0;
	for(int i = 1; i < MODULE_NUM; i++)
	{
		sum_of_former_module[i] = sum_of_former_module[i-1] + used_led_num_in_module[i];
	}
}
void update_max_led_num_of_modules(void)
{
	max_led_num_of_modules = used_led_num_in_module[0];
	for(int i = 1; i < MODULE_NUM; i++)
	{
		if(max_led_num_of_modules < used_led_num_in_module[i])
		{
			max_led_num_of_modules = used_led_num_in_module[i];
		}
	}
}
void update_min_led_num_of_modules(void)
{
	min_led_num_of_modules = used_led_num_in_module[0];
	for(int i = 1; i < MODULE_NUM; i++)
	{
		if(min_led_num_of_modules > used_led_num_in_module[i])
		{
			min_led_num_of_modules = used_led_num_in_module[i];
		}
	}
}

void update_max_pos(mode_t mode)
{
	switch((uint32_t)mode)
	{
		case MODE_1:
		case MODE_2:
			for(int i = 0; i < MODULE_NUM; i++)
			{
				max_pos[i] = MODULE_LED_NUM(i);
			}
			break;
		case MODE_3:
			for(int i = 0; i < MODULE_NUM; i++)
			{
				max_pos[i] = MODULE_LED_NUM(i) >> 1;
			}
			break;
		default:break;
	}
}


void update_led_configuration(void)
{
	update_sum_of_former_module();
	update_max_led_num_of_modules();
	update_min_led_num_of_modules();
	update_max_pos(MODE_1);
	//total led buffer nums =  CH_NUM * CH_LED_NUM
	//also = the (MODULE_NUM-1)(th) used_led_num_in_module + the (MODULE_NUM-1)(th) sum_of_former_module
	if((CH_NUM * CH_LED_NUM) >= (sum_of_former_module[MODULE_NUM - 2] + used_led_num_in_module[MODULE_NUM - 1]))
	{
		is_led_configuration_updated = 1;
	}
	else
	{
		is_led_configuration_updated = 0;
	}
}


void update_led(void)
{
	ws2812_send_multi_ch_armcc(led_buffer[0],led_buffer[1],led_buffer[2],led_buffer[3],
								led_buffer[4],led_buffer[5],CH_LED_NUM*3);
}

void prepare_data(void)
{
	for(int i,j=0; i<SAMPLE_SIZE; i+=2,j++)
	{
		int16_t sdata= pAudioRecBuf[j];
		input[i] = sdata;
		input[i+1] = 0.0;
	}
}

void do_fft(void)
{
	arm_cfft_radix4_instance_f32 S; 
	arm_cfft_radix4_init_f32(&S, FFT_SIZE, 0, 1); 
	arm_cfft_radix4_f32(&S, input);
	arm_cmplx_mag_f32(input, output, FFT_SIZE);
}


void update_music_col_len(void)
{
	for(int i = 0; i < MODULE_NUM; i++)
	{
		cur_pos[i] =  output[i] / 200000 * CH_LED_NUM;

		//do not exceed max value of MODULE_LED_NUM
		if(cur_pos[i] > ((max_pos[i])))
		{
			cur_pos[i] = ((max_pos[i]));
		}
		
		//save newest max value
		cur_max_pos[i] = (cur_pos[i] > last_pos[i])? cur_pos[i] : last_pos[i];	
		
		//update current target value if last target has been completed
		if(is_last_target_reached[i] == 1)
		{
			cur_target_pos[i] = cur_max_pos[i];
			if(cur_target_pos[i] < 1)
				cur_target_pos[i] = 1;
			is_last_target_reached[i] = 0;
		}
		
		//compare current target with respective output_pos which 
		//can be interpreted as pos of led roll or other mode specific meanings
		if(output_pos[i] < cur_target_pos[i])
		{
			output_pos[i] += 1;
		}
		else if (output_pos[i] > cur_target_pos[i])
		{
			if(output_pos[i] > 1)
				output_pos[i]--;			
		}
		else
		{
			is_last_target_reached[i] = 1;
		}		

		last_pos[i] = cur_pos[i];
	}
}

void led_loop_fft(void)
{
	color_t c_music_col = COLOR(0,100,0);
	color_t c_other = COLOR(5,5,0);

	
	if(!is_led_configuration_updated)
	{
		debug(info, "is_led_configuration_updated is false, please firstly call update_led_configuration()");
		return;
	}
	update_max_pos(MODE_1);

	while(1)
	{
		/*wait data ready*/
		while(Data_Status == 0);
		Data_Status =0;
		
		/*prepare input data, insert 0 between PCM vlaues inside pAudioRecBuf*/
		prepare_data();
		
		/*update fft value to output*/
		do_fft();
		
		/*visualization, convert freq to pos inside module*/ 
		update_music_col_len();
		
		//debug(info, "led_num_ch0 = %d ", output_pos[8]);	
		
		
		/**update mem space of led**/
		/*mode 1*/
//		for(int j=0; j<FFT_SIZE; j++)
//		{
//			set_led_range_in_module(j,0, output_pos[j], &c_music_col);
//			set_led_range_in_module(j,output_pos[j], used_led_num_in_module[j], &c_other);
//			set_led_range_in_module(j,used_led_num_in_module[j], MODULE_LED_NUM[j], &c_zero);
//		}

		/*mode 2*/
		for(int j=0; j<FFT_SIZE; j++)
		{
			//c_music_col = COLOR((cur_pos[0]+ 20), (cur_pos[7]+ 20), (cur_pos[MODULE_MAX]+ 20));
			set_led_roll_in_module(j, output_pos[j], 5, &c_music_col, &c_other);
		}	
		
		/*mode 3*/
		/*!!!!!Note: call update_max_pos(MODE_3); before this while(1) loop */
//		for(int j=0; j<MODULE_NUM; j++)
//		{
//			uint32_t temp = MODULE_LED_NUM(j) - output_pos[j];
//			set_led_range_in_module(j,0, output_pos[j], &c_music_col);
//			set_led_range_in_module(j,output_pos[j], temp, &c_other);
//			set_led_range_in_module(j,temp, MODULE_LED_NUM(j), &c_music_col);
//		}	

		/*send data to leds*/
		update_led();
		
		/*control the overall freqency*/
		delay_ms(5);
	}
}

void set_led_roll_in_ch(uint8_t led_ch, uint32_t roll_pos, uint32_t roll_led_num, 
							 const color_t *c_roll, const color_t *c_other)
{
	uint32_t roll_start = roll_pos;
	uint32_t roll_end = roll_pos + roll_led_num;
	set_led_range_in_ch(led_ch, 0, roll_start, c_other);
	set_led_range_in_ch(led_ch, roll_start, roll_end, c_roll);
	set_led_range_in_ch(led_ch, roll_end, used_led_num_in_ch[led_ch], c_other);
	set_led_range_in_ch(led_ch, used_led_num_in_ch[led_ch], CH_LED_NUM, &c_zero);
}

void led_roll_in_ch(uint8_t led_ch_num, uint32_t roll_led_num, const color_t *c_roll, const color_t *c_other, uint32_t delay)
{

	if(roll_led_num > CH_LED_NUM)
	{
		return;
	}
	int max = CH_LED_NUM - roll_led_num + 1;
	for(int i=0; i<max; i++)
	{
		for(int j=0; j<led_ch_num; j++)
		{
			set_led_roll_in_ch(j, i, roll_led_num, c_roll, c_other);
		}		
		update_led();
		delay_ms(delay);
	}	
}

void set_led_roll_in_module(uint8_t led_module, uint32_t roll_pos, uint32_t roll_led_num, 
							 const color_t *c_roll, const color_t *c_other)
{
	uint32_t roll_start = roll_pos;
	uint32_t roll_end = roll_pos + roll_led_num;
	set_led_range_in_module(led_module, 0, roll_start, c_other);
	set_led_range_in_module(led_module, roll_start, roll_end, c_roll);
	set_led_range_in_module(led_module, roll_end, MODULE_LED_NUM(led_module), c_other);
}


//demo of roll in module 
void led_roll_in_module(uint8_t led_module_num_to_roll, uint32_t roll_led_num, const color_t *c_roll, const color_t *c_other, uint32_t delay)
{
	if(roll_led_num > min_led_num_of_modules)
	{
		return;
	}
	int max = min_led_num_of_modules - roll_led_num + 1;
	for(int i=0; i<max; i++)
	{
		for(int j=0; j<led_module_num_to_roll; j++)
		{
			set_led_roll_in_module(j, i, roll_led_num, c_roll, c_other);
		}
		update_led();
		delay_ms(delay);
	}	
}

void led_roll_demo(void)
{
	color_t c_roll = COLOR(50,50,0);
	color_t c_other = COLOR(0,0,0);
	color_t c_init = COLOR(0,0,0);
	if(!is_led_configuration_updated)
	{
		debug(info, "is_led_configuration_updated is false, please firstly call update_led_configuration()");
		return;
	}
	set_led_range_in_ch(0, 0, CH_LED_NUM, &c_init);
	update_led();
	for(int i = 0; i <5 ; i++)
	{
		led_roll_in_module(1, i+1, &c_roll, &c_other, 5);
	}
	for(int i = 0; i <5 ; i++)
	{
		led_roll_in_module(2, i+1, &c_roll, &c_other, 5);
	}
	for(int i = 0; i <5 ; i++)
	{
		led_roll_in_module(8, i+1, &c_roll, &c_other, 5);
	}
	for(int i = 0; i <5 ; i++)
	{
		led_roll_in_ch(1, i+1, &c_roll, &c_other, 5);
	}
	for(int i = 0; i <5 ; i++)
	{
		led_roll_in_ch(2, i+1, &c_roll, &c_other, 5);
	}
}

void set_led_in_ch(uint8_t led_ch, uint32_t led_ch_pos, const color_t *c)
{
	if(led_ch > CH_MAX)
	{
		debug(err, "ch(%d) too large.", led_ch);
		return;
	}
	if(led_ch_pos > CH_LED_MAX)
	{
		debug(err, "led_ch_pos(%d) too large.", led_ch_pos);
		return;
	}
	uint32_t led_ch_buf_pos = led_ch_pos * 3;
	led_buffer[led_ch][led_ch_buf_pos] = GET_G(*c);
	led_buffer[led_ch][led_ch_buf_pos+1] = GET_R(*c);
	led_buffer[led_ch][led_ch_buf_pos+2] = GET_B(*c);
}

void set_led_in_module(uint8_t led_module, uint32_t led_module_pos, const color_t *c)
{
	if(led_module > MODULE_MAX)
	{
		debug(err, "module too large.");
		return;
	}
	if(led_module_pos > MODULE_LED_MAX(led_module))
	{
		debug(err, "led_module_pos too large.");
		return;
	}
	
	uint8_t led_ch = GET_LED_CH(led_module);
	uint8_t led_ch_pos = GET_LED_CH_POS(led_module, led_module_pos);
	set_led_in_ch(led_ch, led_ch_pos, c);
}

void set_led_range_in_ch(uint8_t led_ch, uint32_t led_start, uint32_t led_end,  const color_t *c)
{
	if(led_start > led_end)
	{
		debug(err, "led_start(%d) is larger than led_end(%d)." , led_start,led_end);
		return;
	}
	for(int i = led_start; i < led_end; i++)
	{
		set_led_in_ch(led_ch,i,c);
	}
}

void set_led_range_in_module(uint8_t led_module, uint32_t led_start, uint32_t led_end,  const color_t *c)
{
	if(led_start > led_end)
	{
		debug(err, "led_start(%d) is larger than led_end(%d)." , led_start,led_end);
		return;
	}
	for(int i = led_start; i < led_end; i++)
	{
		set_led_in_module(led_module,i,c);
	}
}

void wave_start(void)
{

	pAudioRecBuf = RecBuf;
	/* Enable the Rx buffer not empty interrupt */
    SPI_I2S_ITConfig(SPI2, SPI_I2S_IT_RXNE, ENABLE);
    /* The Data transfer is performed in the SPI interrupt routine */
    /* Enable the SPI peripheral */
    I2S_Cmd(SPI2, ENABLE); 

}

/**
  * @brief  Initialize wave recording
  * @param  AudioFreq: Sampling frequency
  *         BitRes: Audio recording Samples format (from 8 to 16 bits)
  *         ChnlNbr: Number of input microphone channel
  * @retval None
  */
uint32_t WaveRecorderInit(uint32_t AudioFreq, uint32_t BitRes, uint32_t ChnlNbr)
{ 
  /* Check if the interface is already initialized */
  if (AudioRecInited)
  {
    /* No need for initialization */
    return 0;
  }
  else
  {
    /* Enable CRC module */
    RCC->AHB1ENR |= RCC_AHB1ENR_CRCEN;
    
    /* Filter LP & HP Init */
    Filter.LP_HZ = 8000;
    Filter.HP_HZ = 10;
    Filter.Fs = 16000;
    Filter.Out_MicChannels = 1;
    Filter.In_MicChannels = 1;
    
    PDM_Filter_Init((PDMFilter_InitStruct *)&Filter);
    
    /* Configure the GPIOs */
    WaveRecorder_GPIO_Init();
    
    /* Configure the interrupts (for timer) */
    WaveRecorder_NVIC_Init();
    
    /* Configure the SPI */
    WaveRecorder_SPI_Init(AudioFreq);
    
    /* Set the local parameters */
    AudioRecBitRes = BitRes;
    AudioRecChnlNbr = ChnlNbr;
    
    /* Set state of the audio recorder to initialized */
    AudioRecInited = 1;
    
    /* Return 0 if all operations are OK */
    return 0;
  }  
}

/**
  * @brief  Start audio recording
  * @param  pbuf: pointer to a buffer
  *         size: Buffer size
  * @retval None
  */
uint8_t WaveRecorderStart(uint16_t* pbuf, uint32_t size)
{
/* Check if the interface has already been initialized */
  if (AudioRecInited)
  {
    /* Store the location and size of the audio buffer */
    pAudioRecBuf = pbuf;
    AudioRecCurrSize = size;
    
    /* Enable the Rx buffer not empty interrupt */
    SPI_I2S_ITConfig(SPI2, SPI_I2S_IT_RXNE, ENABLE);
    /* The Data transfer is performed in the SPI interrupt routine */
    /* Enable the SPI peripheral */
    I2S_Cmd(SPI2, ENABLE); 
   
    /* Return 0 if all operations are OK */
    return 0;
  }
  else
  {
    /* Cannot perform operation */
    return 1;
  }
}

/**
  * @brief  Stop audio recording
  * @param  None
  * @retval None
  */
uint32_t WaveRecorderStop(void)
{
  /* Check if the interface has already been initialized */
  if (AudioRecInited)
  {
    
    /* Stop conversion */
    I2S_Cmd(SPI2, DISABLE); 
    
    /* Return 0 if all operations are OK */
    return 0;
  }
  else
  {
    /* Cannot perform operation */
    return 1;
  }
}

/**
  * @brief  This function handles AUDIO_REC_SPI global interrupt request.
  * @param  None
  * @retval None
*/

	
void AUDIO_REC_SPI_IRQHANDLER(void)
{  
   u16 volume;
   u16 app;
	u16 data;

  /* Check if data are available in SPI Data register */
  if (SPI_GetITStatus(SPI2, SPI_I2S_IT_RXNE) != RESET)
  {
    app = SPI_I2S_ReceiveData(SPI2);
	  data = HTONS(app);
    InternalBuffer[InternalBufferSize++] = data;
    
    /* Check to prevent overflow condition */
    if (InternalBufferSize >= INTERNAL_BUFF_SIZE)
    {
      InternalBufferSize = 0;
     
      volume = 50;
      
      PDM_Filter_64_LSB((uint8_t *)InternalBuffer, (uint16_t *)pAudioRecBuf, volume , (PDMFilter_InitStruct *)&Filter);
		
      Data_Status = 1;       
    }
  }
}

void print_dcm_data(void)
{
	while(1)
	{
		while(Data_Status == 0);
		Data_Status =0;
      
		/* Switch the buffers*/
		if (Switch ==1)
		{
			pAudioRecBuf = RecBuf;
			writebuffer = RecBuf1;
			Switch = 0;
		}
		else
		{
			pAudioRecBuf = RecBuf1;
			writebuffer = RecBuf;
			Switch = 1;
		}
		
	}
}

/**
  * @brief  Initialize the wave header file
  * @param  pHeadBuf:Pointer to a buffer
  * @retval None
  */
uint32_t WavaRecorderHeaderInit(uint8_t* pHeadBuf)
{
  uint16_t count = 0;

  /* write chunkID, must be 'RIFF'  ------------------------------------------*/
  pHeadBuf[0] = 'R';
  pHeadBuf[1] = 'I';
  pHeadBuf[2] = 'F';
  pHeadBuf[3] = 'F';

  /* Write the file length */
  /* The sampling time 8000 Hz
   To recorde 10s we need 8000 x 10 x 2 (16-Bit data) */
  pHeadBuf[4] = 0x00;
  pHeadBuf[5] = 0xE2;
  pHeadBuf[6] = 0x04;
  pHeadBuf[7] = 0x00;

  
  /* Write the file format, must be 'WAVE' */
  pHeadBuf[8]  = 'W';
  pHeadBuf[9]  = 'A';
  pHeadBuf[10] = 'V';
  pHeadBuf[11] = 'E';

  /* Write the format chunk, must be'fmt ' */
  pHeadBuf[12]  = 'f';
  pHeadBuf[13]  = 'm';
  pHeadBuf[14]  = 't';
  pHeadBuf[15]  = ' ';

  /* Write the length of the 'fmt' data, must be 0x10 */
  pHeadBuf[16]  = 0x10;
  pHeadBuf[17]  = 0x00;
  pHeadBuf[18]  = 0x00;
  pHeadBuf[19]  = 0x00;

  /* Write the audio format, must be 0x01 (PCM) */
  pHeadBuf[20]  = 0x01;
  pHeadBuf[21]  = 0x00;

  /* Write the number of channels, must be 0x01 (Mono) or 0x02 (Stereo) */
  pHeadBuf[22]  = 0x02;
  pHeadBuf[23]  = 0x00;

  /* Write the Sample Rate 8000 Hz */
  pHeadBuf[24]  = (uint8_t)((REC_FREQ & 0xFF));
  pHeadBuf[25]  = (uint8_t)((REC_FREQ >> 8) & 0xFF);
  pHeadBuf[26]  = (uint8_t)((REC_FREQ >> 16) & 0xFF);
  pHeadBuf[27]  = (uint8_t)((REC_FREQ >> 24) & 0xFF);

  /* Write the Byte Rate */
  pHeadBuf[28]  = (uint8_t)((REC_FREQ & 0xFF));
  pHeadBuf[29]  = (uint8_t)((REC_FREQ >> 8) & 0xFF);
  pHeadBuf[30]  = (uint8_t)((REC_FREQ >> 16) & 0xFF);
  pHeadBuf[31]  = (uint8_t)((REC_FREQ >> 24) & 0xFF);

  /* Write the block alignment */
  pHeadBuf[32]  = 0x02;/*0x02*/
  pHeadBuf[33]  = 0x00;

  /* Write the number of bits per sample */
  pHeadBuf[34]  = 0x10; /*0x08*/
  pHeadBuf[35]  = 0x00;

  /* Write the Data chunk, must be 'data' */
  pHeadBuf[36]  = 'd';
  pHeadBuf[37]  = 'a';
  pHeadBuf[38]  = 't';
  pHeadBuf[39]  = 'a';

  /* Write the number of sample data */
  pHeadBuf[40] = 0x00;
  pHeadBuf[41] = 0xE2;
  pHeadBuf[42] = 0x04;
  pHeadBuf[43] = 0x00;

  /* Fill the missing bytes in Buffer with 0x80 */
  for (count = 44; count < 512 ; count ++)
  {
    pHeadBuf[count] = 0x80;
  }
  
  /* Return 0 if all operations are OK */
  return 0;
}

/**
  * @brief  Update the recorded data 
  * @param  None
  * @retval None
  */
void WaveRecorderUpdate(void)
{     
  WaveRecorderInit(32000,16, 1);
  WaveCounter = 0;
  LED_Toggle = 7;
  
  /* Remove Wave file if exist on flash disk */
  f_unlink (REC_WAVE_NAME);
     
  /* Open the file to write on it */
  if ((HCD_IsDeviceConnected(&USB_OTG_Core) != 1) || (f_open(&file, REC_WAVE_NAME, FA_CREATE_ALWAYS | FA_WRITE) != FR_OK))
  {
    /* Set ON Red LED */ 
    while(1)
    {
      STM_EVAL_LEDToggle(LED5); 
    }
  }
  else
  {
    WaveRecStatus = 1;
  }
  /* Initialize the Header Wave */
  WavaRecorderHeaderInit(RecBufHeader);
  
  /* Write the Header wave */
  f_write (&file, RecBufHeader, 512, (void *)&bytesWritten);
  
  /* Increment tne wave counter */  
  WaveCounter += 512;

  /* Start the record */
  WaveRecorderStart(RecBuf, PCM_OUT_SIZE);
  
  /* Reset the time base variable */
  Time_Rec_Base = 0;
  Switch = 0;
     
  while(HCD_IsDeviceConnected(&USB_OTG_Core))
  { 
    /* Wait for the recording time */  
    if (Time_Rec_Base <= TIME_REC)
    {
      /* Wait for the data to be ready with PCM form */
      while((Data_Status == 0)&& HCD_IsDeviceConnected(&USB_OTG_Core));
      Data_Status =0;
      
      /* Switch the buffers*/
      if (Switch ==1)
     {
       pAudioRecBuf = RecBuf;
       writebuffer = RecBuf1;
       WaveCounter += 32;
       Switch = 0;
     }
      else
      {
        pAudioRecBuf = RecBuf1;
        writebuffer = RecBuf;
        WaveCounter += 32;
        Switch = 1;
      }
      
      for (counter=0; counter<16; counter++)
      {
        LED_Toggle = 3;
        if (buf_idx< RAM_BUFFER_SIZE)
        {
          /* Store Data in RAM buffer */
          RAM_Buf[buf_idx++]= *(writebuffer + counter);
          if (buf_idx1 == RAM_BUFFER_SIZE)
          {
            buf_idx1 = 0;
            /* Write the stored data in the RAm to the USB Key */
            f_write (&file, (uint16_t*)RAM_Buf1, RAM_BUFFER_SIZE*2 , (void *)&bytesWritten);
          }
        }
        else if (buf_idx1< RAM_BUFFER_SIZE)
        {
          /* Store Data in RAM buffer */
          RAM_Buf1[buf_idx1++]= *(writebuffer + counter);
          if (buf_idx == RAM_BUFFER_SIZE)
          {
            buf_idx = 0;
            /* Write the stored data in the RAM to the USB Key */
            f_write (&file, (uint16_t*)RAM_Buf, RAM_BUFFER_SIZE*2 , (void *)&bytesWritten);
          }
        }
      }   
 
      /* User button pressed */
      if ( Command_index != 1)
      {
        /* Stop recording */
        WaveRecorderStop();  
        Command_index = 0;
        LED_Toggle = 6;
        break;
      }
    }
    else /* End of Recording time  */
    {
      WaveRecorderStop();
      LED_Toggle = 4;
      Command_index = 2;
      Data_Status =0;
      break;
    }
  }
   
  /* Update the data length in the header of the recorded wave */    
  f_lseek(&file, 0);
    
  RecBufHeader[4] = (uint8_t)(WaveCounter + 512) ;
  RecBufHeader[5] = (uint8_t)(((WaveCounter+512)  >> 8) & 0xFF);
  RecBufHeader[6] = (uint8_t)(((WaveCounter+512)  >> 16) & 0xFF);
  RecBufHeader[7] = (uint8_t)(((WaveCounter+512)  >> 24) & 0xFF);
  
  RecBufHeader[40] = (uint8_t)(WaveCounter);
  RecBufHeader[41] = (uint8_t)((WaveCounter >> 8) & 0xFF);
  RecBufHeader[42] = (uint8_t)((WaveCounter >> 16) & 0xFF);
  RecBufHeader[43] = (uint8_t)((WaveCounter >> 24) & 0xFF);
    
  /* Write the updated header wave */
  f_write (&file, RecBufHeader, 512, (void *)&bytesWritten);
  
  /* Close file and filesystem */
  f_close (&file);
  f_mount(0, NULL);
  
}

/**
  * @brief  Initialize GPIO for wave recorder.
  * @param  None
  * @retval None
  */
static void WaveRecorder_GPIO_Init(void)
{  
  GPIO_InitTypeDef GPIO_InitStructure;

  /* Enable GPIO clocks */
  RCC_AHB1PeriphClockCmd(SPI_SCK_GPIO_CLK | SPI_MOSI_GPIO_CLK, ENABLE);

  /* Enable GPIO clocks */
  RCC_AHB1PeriphClockCmd(SPI_SCK_GPIO_CLK | SPI_MOSI_GPIO_CLK, ENABLE);

  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

  /* SPI SCK pin configuration */
  GPIO_InitStructure.GPIO_Pin = SPI_SCK_PIN;
  GPIO_Init(SPI_SCK_GPIO_PORT, &GPIO_InitStructure);
  
  /* Connect SPI pins to AF5 */  
  GPIO_PinAFConfig(SPI_SCK_GPIO_PORT, SPI_SCK_SOURCE, SPI_SCK_AF);
  
  /* SPI MOSI pin configuration */
  GPIO_InitStructure.GPIO_Pin =  SPI_MOSI_PIN;
  GPIO_Init(SPI_MOSI_GPIO_PORT, &GPIO_InitStructure);
  GPIO_PinAFConfig(SPI_MOSI_GPIO_PORT, SPI_MOSI_SOURCE, SPI_MOSI_AF);
}

/**
  * @brief  Initialize SPI peripheral.
  * @param  Freq :Audio frequency
  * @retval None
  */
static void WaveRecorder_SPI_Init(uint32_t Freq)
{
  I2S_InitTypeDef I2S_InitStructure;

  /* Enable the SPI clock */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2,ENABLE);
  
  /* SPI configuration */
  SPI_I2S_DeInit(SPI2);
  I2S_InitStructure.I2S_AudioFreq = 32000;
  I2S_InitStructure.I2S_Standard = I2S_Standard_LSB;
  I2S_InitStructure.I2S_DataFormat = I2S_DataFormat_16b;
  I2S_InitStructure.I2S_CPOL = I2S_CPOL_High;
  I2S_InitStructure.I2S_Mode = I2S_Mode_MasterRx;
  I2S_InitStructure.I2S_MCLKOutput = I2S_MCLKOutput_Disable;
  /* Initialize the I2S peripheral with the structure above */
  I2S_Init(SPI2, &I2S_InitStructure);

  /* Enable the Rx buffer not empty interrupt */
  SPI_I2S_ITConfig(SPI2, SPI_I2S_IT_RXNE, ENABLE);
}


/**
  * @brief  Initialize the NVIC.
  * @param  None
  * @retval None
  */
static void WaveRecorder_NVIC_Init(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;

  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_3); 
  /* Configure the SPI interrupt priority */
  NVIC_InitStructure.NVIC_IRQChannel = SPI2_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}


#ifdef  USE_FULL_ASSERT

/**
* @brief  Reports the name of the source file and the source line number
*   where the assert_param error has occurred.
* @param  file: pointer to the source file name
* @param  line: assert_param error line source number
* @retval None
*/
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
  ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  
  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/**
* @}
*/ 


/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
