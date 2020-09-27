//*****************************************************************************
//
//! @file pdm_fft.c
//!
//! @brief An example to show basic PDM operation.
//!
//! Purpose: This example enables the PDM interface to record audio signals from an
//! external microphone. The required pin connections are:
//!
//! Printing takes place over the ITM at 1M Baud.
//!
//! GPIO 11 - PDM DATA
//! GPIO 12 - PDM CLK
//
//*****************************************************************************

#define ARM_MATH_CM4
#include <arm_math.h>
#include "am_mcu_apollo.h"
#include "am_bsp.h"
#include "am_util.h"
#include <stdio.h>
#include <string.h>

//*****************************************************************************
//
// 
//
//*****************************************************************************
#define PRINT_PDM_DATA              1
#define PRINT_FFT_DATA              0 
#define AUDIO_FRAME_MS 							1000                          //單位:ms 
#define g_ui32SampleFreq        ((16000*AUDIO_FRAME_MS)/1000)     //default 16khz
#define AUDIO_FRAME_SIZE_MONO_BYTES     (g_ui32SampleFreq*2)      //單聲道
#define AUDIO_FRAME_SIZE_STEREO_BYTES   (g_ui32SampleFreq*4)      //雙聲道
#define TIMERNUM    3

//*****************************************************************************
//
// Macros 
//
//*****************************************************************************
#define     FASTGPIO_PIN_A      0
#define     FASTGPIO_PIN_B      1
#define     FASTGPIO_PIN_C      2
#define     FASTGPIO_PIN_D      3
#define     FASTGPIO_PIN_E      4
#define     FASTGPIO_PIN_F      5
#define     FASTGPIO_PIN_G      6
#define     FASTGPIO_PIN_H      7
//*****************************************************************************
//
// Global variables.
//
//*****************************************************************************
int value_led = 0;
int counter = 0;
//int PDM_regCount = 0;
volatile bool g_bPDMDataReady = false;
uint32_t g_ui32PDMDataBuffer[AUDIO_FRAME_SIZE_MONO_BYTES/4];  //存放已轉完的PCM data
volatile unsigned int *PDMData = (volatile unsigned int *)0x10040000;       //測試存放資料用記憶體位置
uint32_t portValue;

/*const am_hal_gpio_pincfg_t gpio_clock_in =
{
		.uFuncSel            = 3,
		//.eDriveStrength      = AM_HAL_GPIO_PIN_DRIVESTRENGTH_2MA,
		.eGPInput            = AM_HAL_GPIO_PIN_INPUT_ENABLE,
		//.eIntDir 						 = AM_HAL_GPIO_PIN_INTDIR_LO2HI,
};
*/
const am_hal_gpio_pincfg_t gpio_clock_out =
{
		.uFuncSel            = 3,
		//.eDriveStrength      = AM_HAL_GPIO_PIN_DRIVESTRENGTH_2MA,
		.eGPOutcfg            = AM_HAL_GPIO_PIN_OUTCFG_PUSHPULL,
		.eGPRdZero					 = AM_HAL_GPIO_PIN_RDZERO_READPIN,
};

const am_hal_gpio_pincfg_t gpio_data = 
{
	 	.uFuncSel            = 3,   
		.eGPOutcfg            = AM_HAL_GPIO_PIN_OUTCFG_DISABLE,
    .eGPInput            = AM_HAL_GPIO_PIN_INPUT_ENABLE,
		.eGPRdZero					 = AM_HAL_GPIO_PIN_RDZERO_READPIN,
};
	
// Timer configurations.
am_hal_ctimer_config_t g_sTimer3 =
{
    // do not link A and B together to make a long 32-bit counter.
    0,
    // Set up timer 3A to drive the ADC
    (AM_HAL_CTIMER_FN_PWM_REPEAT |
     AM_HAL_CTIMER_HFRC_12MHZ | 
		 AM_HAL_CTIMER_INT_ENABLE),
    // Timer 3B is not used in this example.
    0,
};

//*****************************************************************************
//
// HANDLE configuration information.
//
//*****************************************************************************
void *PDMHandle;
void *GPIOHandle;

am_hal_pdm_config_t g_sPdmConfig =
{
    .eClkDivider = AM_HAL_PDM_MCLKDIV_1,
    .eLeftGain = AM_HAL_PDM_GAIN_P105DB,
    .eRightGain = AM_HAL_PDM_GAIN_P105DB,
    .ui32DecimationRate = 48,
    .bHighPassEnable = 0,                           
    .ui32HighPassCutoff = 0xB,
    .ePDMClkSpeed = AM_HAL_PDM_CLK_1_5MHZ,
    .bInvertI2SBCLK = 0,
    .ePDMClkSource = AM_HAL_PDM_INTERNAL_CLK,
    .bPDMSampleDelay = 0,
    .bDataPacking = 1,
    .ePCMChannels = AM_HAL_PDM_CHANNEL_RIGHT,
    .ui32GainChangeDelay = 1,
    .bI2SEnable = 0, 
    .bSoftMute = 0,
    .bLRSwap = 0,
};

//*****************************************************************************
//
// PDM initialization.
//
//*****************************************************************************
void pdm_init(void)
{
    //
    // Initialize, power-up, and configure the PDM.
    //
    am_hal_pdm_initialize(0, &PDMHandle);
    am_hal_pdm_power_control(PDMHandle, AM_HAL_PDM_POWER_ON, false);
    am_hal_pdm_configure(PDMHandle, &g_sPdmConfig);
    am_hal_pdm_enable(PDMHandle);

    //am_util_stdio_printf("pdm init1111.\n\n");
    //
    // Configure the necessary pins.
    //
    am_hal_gpio_pincfg_t sPinCfg = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

    sPinCfg.uFuncSel = AM_HAL_PIN_11_PDMDATA;
    am_hal_gpio_pinconfig(11, sPinCfg);

    //am_util_stdio_printf("pdm init111122.\n\n");
    sPinCfg.uFuncSel = AM_HAL_PIN_12_GPIO;
    am_hal_gpio_pinconfig(12, gpio_clock_out);
    //am_util_stdio_printf("pdm init111112222.\n\n");

    am_hal_gpio_state_write(14, AM_HAL_GPIO_OUTPUT_CLEAR);
    am_hal_gpio_pinconfig(14, g_AM_HAL_GPIO_OUTPUT);

    //am_util_stdio_printf("pdm init2222.\n\n");
    //
    // Configure and enable PDM interrupts (set up to trigger on DMA
    // completion).
    //
    am_hal_pdm_interrupt_enable(PDMHandle, (AM_HAL_PDM_INT_DERR
                                            | AM_HAL_PDM_INT_DCMP
                                            | AM_HAL_PDM_INT_UNDFL
                                            | AM_HAL_PDM_INT_OVF));

    //am_util_stdio_printf("pdm init3333.\n\n");
    NVIC_EnableIRQ(PDM_IRQn);
}

//*****************************************************************************
//
// Print PDM configuration data.
//
//*****************************************************************************
void pdm_config_print(void)
{
    uint32_t ui32PDMClk;

    switch (g_sPdmConfig.ePDMClkSpeed)
    {
        case AM_HAL_PDM_CLK_12MHZ:  ui32PDMClk = 12000000; break;
        case AM_HAL_PDM_CLK_6MHZ:   ui32PDMClk =  6000000; break;
        case AM_HAL_PDM_CLK_3MHZ:   ui32PDMClk =  3000000; break;
        case AM_HAL_PDM_CLK_1_5MHZ: ui32PDMClk =  1500000; break;
        case AM_HAL_PDM_CLK_750KHZ: ui32PDMClk =   750000; break;
        case AM_HAL_PDM_CLK_375KHZ: ui32PDMClk =   375000; break;
        case AM_HAL_PDM_CLK_187KHZ: ui32PDMClk =   187000; break;

        default:
            ui32PDMClk = 0;
    }
}

//啟動PDM DMA傳輸
void pdm_data_get(void)
{
    //
    // Configure DMA and target address.
    //
    am_hal_pdm_transfer_t sTransfer;
    sTransfer.ui32TargetAddr = (uint32_t ) g_ui32PDMDataBuffer;
		//am_util_stdio_printf("%d",g_ui32PDMDataBuffer);
    sTransfer.ui32TotalCount = AUDIO_FRAME_SIZE_MONO_BYTES;

    //
    // Start the data transfer.
    //
    am_hal_pdm_enable(PDMHandle);
    am_util_delay_ms(100);
    am_hal_pdm_fifo_flush(PDMHandle);
    am_hal_pdm_dma_start(PDMHandle, &sTransfer);
}

//*****************************************************************************
//
// PDM interrupt handler.
//
//*****************************************************************************
void am_pdm0_isr(void)
{
    uint32_t ui32Status;

    //
    // Read the interrupt status.
    //
    am_hal_pdm_interrupt_status_get(PDMHandle, &ui32Status, true);
    am_hal_pdm_interrupt_clear(PDMHandle, ui32Status);

	  
    //am_util_stdio_printf("PDM Interrupt.  ui32Status = %d \n\n" , ui32Status);
    //
    // Once our DMA transaction completes, we will disable the PDM and send a
    // flag back down to the main routine. Disabling the PDM is only necessary
    // because this example only implemented a single buffer for storing FFT
    // data. More complex programs could use a system of multiple buffers to
    // allow the CPU to run the FFT in one buffer while the DMA pulls PCM data
    // into another buffer.
    //
    //DMA 傳輸完成
    //am_util_stdio_printf("PDM Interrupt.  counter = %d \n\n" , counter);
    if (ui32Status & AM_HAL_PDM_INT_DCMP)
    {
        am_hal_pdm_disable(PDMHandle);
        g_bPDMDataReady = true;
				am_util_stdio_printf("PDM Interrupt.  counter = %d \n\n" , counter);
				am_util_stdio_printf("PDM Interrupt.  Complete \n\n");
    }
}

//GPIO Interrupt
void am_gpio_isr(void)
{
  uint64_t gpioStatus;
	counter++;
  //am_hal_gpio_interrupt_status_get(true,&gpioStatus);//取得gpio狀態
  //AM_HAL_GPIO_MASKCREATE(GpioIntMask);
  //am_hal_gpio_interrupt_clear(AM_HAL_GPIO_MASKBIT(pGpioIntMask, 48));
	//am_hal_gpio_interrupt_enable(AM_HAL_GPIO_MASKBIT(pGpioIntMask, 48));
  
  //am_util_stdio_printf("GPIO ISR Status = %d \n", gpioStatus);
  //uint32_t res = am_hal_gpio_state_read(49,AM_HAL_GPIO_INPUT_READ,&portValue);
	//am_util_stdio_printf("status : %d\n",res);
	//*(PDM_Data + 1) = *Data;
	//am_util_stdio_printf("%x\n",*Data);
	if(counter == 1500000)
		{
			am_util_stdio_printf("disable interrupt!\n");
			AM_HAL_GPIO_MASKCREATE(sFastGpioMask);
			am_hal_gpio_interrupt_disable(AM_HAL_GPIO_MASKBIT(psFastGpioMask, 48));
		}
}


void am_ctimer_isr(void)
{
  // Clear TimerA0 Interrupt.
 	//PDM_regCount ++ ;
  uint32_t status = am_hal_ctimer_int_status_get(true);
	uint32_t res = am_hal_gpio_state_read(49,AM_HAL_GPIO_INPUT_READ,&portValue);
	//*(PDMData + PDM_regCount) =  portValue; 
	am_util_stdio_printf("%d ",portValue);
	//am_util_stdio_printf("%d ",res);
  am_hal_ctimer_int_clear(status);
  am_hal_ctimer_int_service(status);
	
  //am_util_stdio_printf("AM_HAL_CTIMER_INT_TIMERA3C0 = %x\n",status);
}

//設定PWM Clock
void timer_init(void)
{
//
// Only CTIMER 3 supports the ADC.
//
    uint32_t ui32Period ; // Set for 2 second (2000ms) period
		
    // LFRC has to be turned on for this example because we are running this
    // timer off of the LFRC.
    am_hal_clkgen_control( AM_HAL_CLKGEN_CONTROL_XTAL_START, 0);
		
    // Set up timer 3A so start by clearing it.
    am_hal_ctimer_clear(TIMERNUM, AM_HAL_CTIMER_TIMERA);

    // Configure the timer to count 32Hz LFRC clocks but don't start it yet.
    am_hal_ctimer_config(TIMERNUM, &g_sTimer3);

    // Compute CMPR value needed for desired period based on a 32HZ clock.
		ui32Period = 8;
    am_hal_ctimer_period_set(TIMERNUM, AM_HAL_CTIMER_TIMERA,
                             ui32Period, (ui32Period >> 1));
		am_hal_ctimer_output_config(TIMERNUM, AM_HAL_CTIMER_TIMERA, 12, AM_HAL_CTIMER_OUTPUT_NORMAL, AM_HAL_GPIO_PIN_DRIVESTRENGTH_12MA);
    // Set up timer 3A as the trigger source for the ADC.
    //am_hal_ctimer_adc_trigger_enable();	
		 am_hal_ctimer_int_enable(AM_HAL_CTIMER_INT_TIMERA3C0);
		 NVIC_EnableIRQ(CTIMER_IRQn);
		 am_hal_interrupt_master_enable();
    // Start timer 3A.
    am_hal_ctimer_start(TIMERNUM, AM_HAL_CTIMER_TIMERA);//................................
		am_util_stdio_printf("timer initial finished.\n\n");
} 

//*****************************************************************************
//
// Main
//
//*****************************************************************************
int main(void)
{
    uint32_t ui32Ret;
	  am_hal_burst_mode_e     eBurstMode;
    am_hal_burst_avail_e    eBurstModeAvailable;
    //
    // Perform the standard initialzation for clocks, cache settings, and
    // board-level low-power operation.
    //
    am_hal_clkgen_control(AM_HAL_CLKGEN_CONTROL_SYSCLK_MAX, 0);
    am_hal_cachectrl_config(&am_hal_cachectrl_defaults);
    am_hal_cachectrl_enable();
    //am_bsp_low_power_init();
		//am_hal_clkgen_control(AM_HAL_CLKGEN_CONTROL_HFADJ_ENABLE, 0);
		//am_hal_gpio_interrupt_register(48, GPIOHandle);
	
		//Initialize LED 
    am_devices_led_array_init(am_bsp_psLEDs, AM_BSP_NUM_LEDS);	//------------------
		am_devices_led_array_out (am_bsp_psLEDs, AM_BSP_NUM_LEDS , value_led);//--------
		//am_hal_gpio_pinconfig( 48 ,  gpio_clock_out);		//設置第48pin的clock為Lo2Hi interrupt
		am_hal_gpio_pinconfig( 49 ,  gpio_data);
		//am_devices_button_array_init(am_bsp_psButton0,AM_BSP_NUM_BUTTONS);
    //
    // Initialize the printf interface for ITM output
    //
    am_bsp_itm_printf_enable();
    //
    // Print the banner.
    //
    am_util_stdio_terminal_clear();
    //am_util_stdio_printf("PDM example.\n\n");

    // 設置快速GPIO		
    /*am_hal_gpio_fastgpio_disable(FASTGPIO_PIN_B);
    am_hal_gpio_fastgpio_clr(FASTGPIO_PIN_B);
		am_hal_gpio_fastgpio_enable(FASTGPIO_PIN_B);
    AM_HAL_GPIO_MASKCREATE(sFastGpioMask);
    ui32Ret = am_hal_gpio_fast_pinconfig(AM_HAL_GPIO_MASKBIT(psFastGpioMask, 48),
                                          gpio_clock_out, 0);*/
																				 															
		am_devices_led_on(am_bsp_psLEDs, 4);
    am_util_delay_ms(300);
    			 
	  // Clear the GPIO Interrupt (write to clear).
    //AM_HAL_GPIO_MASKCREATE(sFastGpioMask);
    //am_hal_gpio_interrupt_clear(AM_HAL_GPIO_MASKBIT(psFastGpioMask, 48));

    //am_util_stdio_printf("11111.\n\n");
    // Enable the GPIO/button interrupt.
    //am_hal_gpio_interrupt_enable(AM_HAL_GPIO_MASKBIT(psFastGpioMask, 48));

    //am_util_stdio_printf("22222.\n\n");
    // Enable the timer interrupt in the NVIC.
		
		//NVIC_EnableIRQ(GPIO_IRQn);
    //am_util_stdio_printf("33333.\n\n");
    //am_hal_interrupt_master_enable();
    //
    // Turn on the PDM, set it up for our chosen recording settings, and start
    // the first DMA transaction.
    //
		timer_init();
    am_devices_led_on(am_bsp_psLEDs, 1);
    

    //超頻模式
    /*
    if ( am_hal_burst_mode_initialize(&eBurstModeAvailable) == AM_HAL_STATUS_SUCCESS )
    {
        if ( eBurstModeAvailable == AM_HAL_BURST_AVAIL )
        {
            am_util_stdio_printf("\nTurboSPOT mode is Available\n");
						
            //進入超頻模式
            if ( am_hal_burst_mode_enable(&eBurstMode) == AM_HAL_STATUS_SUCCESS )
            {
                if ( eBurstMode == AM_HAL_BURST_MODE )
                {
                    am_util_stdio_printf("Operating in TurboSPOT mode (%dMHz)\n",
                                         AM_HAL_CLKGEN_FREQ_MAX_MHZ * 2);
                }
            }
            else
                am_util_stdio_printf("Failed to Enable TurboSPOT mode operation\n");
        }
        else
            am_util_stdio_printf("TurboSPOT mode is Not Available\n");
    }
    else
        am_util_stdio_printf("Failed to Initialize for TurboSPOT mode operation\n");
    */
		
    //pdm_init();
    //am_util_stdio_printf("44444.\n\n");
    //pdm_config_print();
    //am_hal_pdm_fifo_flush(PDMHandle);
		

    //pdm_data_get();

		am_devices_led_off(am_bsp_psLEDs, 4);

    // while loop 持續錄音
    while (1)
    {
        //am_hal_interrupt_master_disable();  //取消interrupt

       /* if (g_bPDMDataReady)
        {
            g_bPDMDataReady = false;

            //pcm_fft_print();

            while (PRINT_PDM_DATA || PRINT_FFT_DATA);

						//am_devices_led_on(am_bsp_psLEDs, 4);
            //pdm_data_get();
						//am_devices_led_off(am_bsp_psLEDs, 4);
        }*/

        // 進入睡眠模式
        am_hal_sysctrl_sleep(AM_HAL_SYSCTRL_SLEEP_DEEP);

        //am_hal_interrupt_master_enable();
    }
}
