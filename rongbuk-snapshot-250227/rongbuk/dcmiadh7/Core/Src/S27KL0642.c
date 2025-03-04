/*
 * S27KL0642.c
 *
 *  Created on: Feb 22, 2025
 *      Author: matia
 */


#include "S27KL0642.h"
#include "octospi.h"

static void OCTOSPI2_Init();
static void DelayBlock_Calibration();

void ExtRAM_Init()
{
    OCTOSPI2_Init();
}

/**
  * @brief OCTOSPI2 Initialization Function
  * @param None
  * @retval None
  */
static void OCTOSPI2_Init(void)
{

  /* USER CODE BEGIN OCTOSPI2_Init 0 */

  /* USER CODE END OCTOSPI2_Init 0 */

  OSPIM_CfgTypeDef sOspiManagerCfg = {0};
  OSPI_HyperbusCfgTypeDef sHyperBusCfg = {0};

  /* USER CODE BEGIN OCTOSPI2_Init 1 */
  OSPI_HyperbusCmdTypeDef sCommand;
   OSPI_MemoryMappedTypeDef sMemMappedCfg;
  /* USER CODE END OCTOSPI2_Init 1 */
  /* OCTOSPI2 parameter configuration*/
  hospi2.Instance = OCTOSPI2;
  hospi2.Init.FifoThreshold = 4;
  hospi2.Init.DualQuad = HAL_OSPI_DUALQUAD_DISABLE;
  hospi2.Init.MemoryType = HAL_OSPI_MEMTYPE_HYPERBUS;
  hospi2.Init.DeviceSize = 23;
  hospi2.Init.ChipSelectHighTime = 2;
  hospi2.Init.FreeRunningClock = HAL_OSPI_FREERUNCLK_DISABLE;
  hospi2.Init.ClockMode = HAL_OSPI_CLOCK_MODE_0;
  hospi2.Init.WrapSize = HAL_OSPI_WRAP_32_BYTES;
  hospi2.Init.ClockPrescaler = 2;
  hospi2.Init.SampleShifting = HAL_OSPI_SAMPLE_SHIFTING_NONE;
  hospi2.Init.DelayHoldQuarterCycle = HAL_OSPI_DHQC_ENABLE;
  hospi2.Init.ChipSelectBoundary = 23;
  hospi2.Init.DelayBlockBypass = HAL_OSPI_DELAY_BLOCK_USED;
  hospi2.Init.MaxTran = 0;
  hospi2.Init.Refresh = 400; //4us @100 Mhz
  if (HAL_OSPI_Init(&hospi2) != HAL_OK)
  {
    Error_Handler();
  }
  sOspiManagerCfg.ClkPort = 1;
  sOspiManagerCfg.DQSPort = 1;
  sOspiManagerCfg.NCSPort = 2;
  sOspiManagerCfg.IOLowPort = HAL_OSPIM_IOPORT_1_LOW;
  sOspiManagerCfg.IOHighPort = HAL_OSPIM_IOPORT_1_HIGH;
  sOspiManagerCfg.Req2AckTime = 1;
  if (HAL_OSPIM_Config(&hospi2, &sOspiManagerCfg, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    Error_Handler();
  }
  sHyperBusCfg.RWRecoveryTime = 4;
  sHyperBusCfg.AccessTime = 7;
  sHyperBusCfg.WriteZeroLatency = HAL_OSPI_LATENCY_ON_WRITE;
  sHyperBusCfg.LatencyMode = HAL_OSPI_FIXED_LATENCY;
  if (HAL_OSPI_HyperbusCfg(&hospi2, &sHyperBusCfg, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN OCTOSPI2_Init 2 */
    /* DELAY block */
	/* disable the output clock and enable the access to the phase selection SEL[3:0] */
	DLYB_OCTOSPI2->CR 	= DLYB_CR_SEN;
	/* set delay */
	DLYB_OCTOSPI2->CFGR = (0 & DLYB_CFGR_SEL_Msk);
	/* disable the access to the phase selection, enable output */
	DLYB_OCTOSPI2->CR 	= DLYB_CR_DEN;

  /* Memory-mapped mode configuration --------------------------------------- */
  sCommand.AddressSpace = HAL_OSPI_MEMORY_ADDRESS_SPACE;
  sCommand.AddressSize  = HAL_OSPI_ADDRESS_32_BITS;
  sCommand.DQSMode      = HAL_OSPI_DQS_ENABLE;
  sCommand.Address      = 0;
  sCommand.NbData       = 1;

  if (HAL_OSPI_HyperbusCmd(&hospi2, &sCommand, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    Error_Handler();
  }

  sMemMappedCfg.TimeOutActivation = HAL_OSPI_TIMEOUT_COUNTER_DISABLE;

  if (HAL_OSPI_MemoryMapped(&hospi2, &sMemMappedCfg) != HAL_OK)
  {
    Error_Handler();
  }

  //DelayBlock_Calibration();
  /* USER CODE END OCTOSPI2_Init 2 */
}

#define DLYB_BUFFERSIZE (COUNTOF(Cal_buffer) - 1)
#define COUNTOF(__BUFFER__) (sizeof(__BUFFER__) /sizeof(*(__BUFFER__)))

/*This function is used to calibrate the Delayblock before initiating USER's application read/write transactions*/
void DelayBlock_Calibration()
{
    /*buffer used for calibration*/
    uint8_t Cal_buffer[] = " ****Delay Block Calibration Buffer**** ****Delay Block Calibration Buffer**** ****Delay Block Calibration Buffer**** ****Delay Block Calibration Buffer**** ****Delay Block Calibration Buffer**** ****Delay Block Calibration Buffer**** ";
    uint16_t index;
    __IO uint8_t *mem_addr;
    uint8_t test_failed;
    uint8_t delay = 0x0;
    uint8_t Min_found = 0;
    uint8_t Max_found = 0;
    uint8_t Min_Window = 0x0;
    uint8_t Max_Window = 0xF;
    uint8_t Mid_window = 0;
    uint8_t calibration_ongoing = 1;

    /* Write the Cal_buffer to the memory*/
    mem_addr = (__IO uint8_t *)(OCTOSPI2_BASE);

    for (index = 0; index < DLYB_BUFFERSIZE; index++)
    {
        *mem_addr = Cal_buffer[index];
        mem_addr++;
    }

    while (calibration_ongoing)
    {/* update the Delayblock calibration */
        //HAL_RCCEx_OCTOSPIDelayConfig(delay, 0);
        test_failed = 0;
        mem_addr = (__IO uint8_t *)(OCTOSPI2_BASE);

        for (index = 0; index < DLYB_BUFFERSIZE; index++)
        {/* Read the Cal_buffer from the memory*/
            if (*mem_addr != Cal_buffer[index])
            {
                /*incorrect data read*/
                test_failed = 1;
            }
            mem_addr++;
        }

        if (Min_found != 1)
        {/* search for the Min window */
            if (test_failed == 1)
            {
                if (delay < 15)
                {
                    delay++;
                }
                else
                {/* If delay set to maximum and error still detected: can't use external Memory*/
                    Error_Handler();
                }
            }
            else
            {
                Min_Window = delay;
                Min_found = 1;
                delay = 0xF;
            }
        }
        else if (Max_found != 1)
        {/* search for the Max window */
            if (test_failed == 1)
            {
                if (delay > 0)
                {
                    delay--;
                }
                else
                {/* If delay set to minimum and error still detected: can't use external Memory */
                    Error_Handler();
                }
            }
            else
            {
                Max_Window = delay;
                Max_found = 1;
            }
        }
        else
        {/* min and max delay window found , configure the delay block with the middle window value and exit calibration */
            Mid_window = (Max_Window + Min_Window) / 2;
           // HAL_RCCEx_OCTOSPIDelayConfig(Mid_window, 0);
            /* Exit calibration */
            calibration_ongoing = 0;
        }
    }
}
