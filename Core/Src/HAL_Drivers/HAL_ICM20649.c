/*
 * HAL_ICM20689.c
 *
 *  Created on: Mar 3, 2022
 *      Author: Michael Noyce
 */

//======================== 1. Includes ==============================================================

#include <HAL_ICM20649.h>

//======================== 2. Private Variables =====================================================

uint8_t I2C_TX_CPLT;	//Flag signaling completion of I2C DMA transfer

//======================== 3. Static Functions Prototypes ===========================================

static void MX_DMA_Init(void);

static HAL_StatusTypeDef  MX_I2C1_Init(void);

static void MX_GPIO_Init(void);

//======================== 3. Static Functions Definition ===========================================

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static HAL_StatusTypeDef MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */
	//printmsg("i2c Function Init \r\n");
  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 	0x00100002;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    //Do nothing
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    //Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
	  //Do nothing
  }
  /* USER CODE BEGIN I2C1_Init 2 */
  //printmsg("i2c init successful! \r\n");
  /* USER CODE END I2C1_Init 2 */
  return HAL_OK;

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  HAL_PWREx_EnableVddIO2();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin : PD14 */
  GPIO_InitStruct.Pin = GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pins : PB14 PB7 */
  GPIO_InitStruct.Pin = GPIO_PIN_14|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}


/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMAMUX1_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);

}

//======================== 5. MSP Function Definitions ==================================
/* MSP FUNCTIONS:
 *
 *  Functions designed to replace the _weak MSP peripheral initialization
 *  function definitions in the HAL Library files.
 *
 *  NB!!!! before running the code do the following:
 *
 *  1. Uncomment the desired MSP Function
 *
 *  2. Cut the function and paste it in the stm32l4xx_hal_msp.c file
 *
 *  3. In the stm32l4xx_hal_msp.c file, include the header "HAL_ICM20649.h"
 */



//======================= 6. Initializaiton Function Definitions ========================================

imu_status_t ICM20649_Init_IMU(uint8_t g_fsr, uint8_t a_fsr, uint8_t dlpf_acc_coeff, uint8_t dlpf_gyro_coeff)
{

	//Select Register Bank
	ICM20649_Register_Bank_Select(&hi2c1, 0);

	//Initialise Peripherals
	MX_DMA_Init();
	if(MX_I2C1_Init() != HAL_OK)
	{
		printmsg("I2C init error \r\n");
		return IMU_PERIPHERAL_INIT_ERROR;
	}
	MX_GPIO_Init();

	//reset buffers and flags
	//imu_sample_count = 0;
	I2C_TX_CPLT = 0;
	//IMU_Log_On = 0;
	//IMU_Log_On = 1;

	//memset(IMU_Buffer,0,IMU_BUFFER_SIZE);

	//get ID
	uint8_t whoami = 0;
	if(ICM20649_Get_ID(&hi2c1,&whoami) != IMU_OK)
	{
		printmsg("IMU offline \r\n");
		return IMU_I2C_DEVICE_OFFLINE;
	}



	if(whoami != WHO_AM_I_VALUE)
	{
		return IMU_I2C_ID_ERROR;
	}

	//Initialise registers
	ICM20649_reset(&hi2c1);
	HAL_Delay(100);
	ICM20649_Set_Wake(&hi2c1); //wake up device
	ICM20649_Set_PLLSrc(&hi2c1, PWR_MGMT_1_CLK_SEL_3); // set clock source to PLL gyro x axis

	//Configure sensor settings
	ICM20649_Set_Acc_FSR(&hi2c1,a_fsr);
	ICM20649_Set_Gyro_FSR(&hi2c1,g_fsr);
	ICM20649_Set_Acc_DLPF(&hi2c1,1, ENABLE);
	ICM20649_Set_Gyro_DLPF(&hi2c1, 1, ENABLE);
	//ICM20649_Disable_LPF(&hi2c1, ACC_FCHOICE_DIS, GYRO_FCHOICE_DIS);
	ICM20649_Set_Sample_Rate(&hi2c1);

	//Enable Temperature Sensor
	ICM20649_Init_TempSensor(&hi2c1, DISABLE);

	//Enable and config FIFO
	//ICM20649_Init_FIFO(&hi2c1, IMU_RESET);
	ICM20649_Init_FIFO(&hi2c1, DISABLE);
	ICM20649_Config_FIFO(&hi2c1, DISABLE, DISABLE, DISABLE, DISABLE);
	//ICM20649_Config_FIFO(&hi2c1, ENABLE, ENABLE, DISABLE, ENABLE);

	//configure interrupt pin
	ICM20649_Disable_Interrupts(&hi2c1);
	//ICM20649_Config_Interrupt_Pin(INT_PIN_CFG_LEVEL_HIGH, INT_PIN_CFG_PIN_PUSH_PULL,INT_PIN_CFG_LATCH_INT_DIS);
	//ICM20649_Enable_Interrupt(&hi2c1, RAW_DATA_0_RDY_INT, INT_ENABLE_1);

	Reset_FIFO(&hi2c1);

	return IMU_OK;
}

imu_status_t ICM20649_Deinit_IMU(void)
{
	 ICM20649_Disable_Interrupts(&hi2c1);
	 //ICM20649_Get_Interrupt_Status(&hi2c1,DATA_READY,(uint8_t*)0xFF);
	//reset Device and place in sleep mode
	ICM20649_Set_PLLSrc(&hi2c1, PWR_MGMT_1_CLK_SEL_0);
	ICM20649_reset(&hi2c1);
	HAL_Delay(100);
	//Deinit I2C peripheral
	if(HAL_I2C_DeInit(&hi2c1) != HAL_OK)
	{
	  return IMU_CONFIG_ERROR;
	}
	if(HAL_DMA_DeInit(&hdma_i2c1_rx) != HAL_OK)
	{
		return IMU_CONFIG_ERROR;
	}
	//imu_sample_count = 0;
	//memset(IMU_Buffer,0,IMU_BUFFER_SIZE);
	return IMU_OK;
}

imu_status_t ICM20649_Init_FIFO(I2C_HandleTypeDef *hi2c, uint8_t enable)
{
	ICM20649_Register_Bank_Select(&hi2c1, 0);

	//get USER_CTRL register Data
	uint8_t uc_byte;
	uint8_t fifo_rst_byte = 0;
	if(HAL_I2C_Mem_Read(hi2c,IMU_Device_Address,USER_CTRL,1,&uc_byte,1,100) != HAL_OK)
	{
		return IMU_I2C_ERROR;
	}
	//set relevent bits
	if(enable == ENABLE)
	{
		uc_byte |= USER_CTRL_FIFO_EN;
	}else if(enable == DISABLE)
	{
		uc_byte &= ~USER_CTRL_FIFO_EN;
	}else if(enable == IMU_RESET)
	{
		fifo_rst_byte |= FIFO_RESET;

	}

	//write values to data register
	if(HAL_I2C_Mem_Write(hi2c,IMU_Device_Address, USER_CTRL,1,&uc_byte,1,100) != HAL_OK)
	{
		return IMU_I2C_ERROR;
	}



	//write values to data register
	if(HAL_I2C_Mem_Write(hi2c,IMU_Device_Address,FIFO_RST,1,&fifo_rst_byte,1,100) != HAL_OK)
	{
		return IMU_I2C_ERROR;
	}
	else
	{
		//Deassert Reset (need to assert and deassert for full reset
		fifo_rst_byte &= ~FIFO_RESET;
		if(HAL_I2C_Mem_Write(hi2c,IMU_Device_Address,FIFO_RST,1,&fifo_rst_byte,1,100)!= HAL_OK)
		{
			return IMU_I2C_ERROR;
		}
	}

	return IMU_OK;

}

imu_status_t ICM20649_Init_TempSensor(I2C_HandleTypeDef *hi2c, uint8_t cmd)
 {
	ICM20649_Register_Bank_Select(&hi2c1, 0);

 	//get register data
 	uint8_t pwrmgmtbyte = 0;
 	if(HAL_I2C_Mem_Read(hi2c,IMU_Device_Address,PWR_MGMT_1,1,&pwrmgmtbyte,1,100) != HAL_OK)
 	{
 		return IMU_I2C_ERROR;
 	}

 	if(cmd == ENABLE)
 	{
 		pwrmgmtbyte &= ~(PWR_MGMT_1_TEMP_DIS);
 	}else if(cmd == DISABLE)
 	{
 		pwrmgmtbyte |= PWR_MGMT_1_TEMP_DIS;
 	}else
 	{
 		return IMU_INIT_CMD_ERROR;
 	}
 	if(HAL_I2C_Mem_Write(hi2c,IMU_Device_Address,PWR_MGMT_1,1,&pwrmgmtbyte,1,100) != HAL_OK)
 	{
 		return IMU_I2C_ERROR;
 	}
 	return IMU_OK;
 }

imu_status_t ICM20649_Enable_Interrupt(I2C_HandleTypeDef *hi2c, uint8_t interrupts, uint8_t intAddress)
{
	ICM20649_Register_Bank_Select(&hi2c1, 0);


	uint8_t iconfigbyte = 0;

	if(HAL_I2C_Mem_Read(hi2c, IMU_Device_Address, intAddress, 1, &iconfigbyte, 1, 100) != HAL_OK)
	{
		return IMU_I2C_ERROR;
	}

	//set enable bit
	iconfigbyte |= interrupts;
	//write to register
	if(HAL_I2C_Mem_Write(hi2c,IMU_Device_Address, intAddress,1,&iconfigbyte,1,100) != HAL_OK)
	{
		return IMU_I2C_ERROR;
	}
	return IMU_OK;
}

/**
 * @brief Clear all interrupt registers
 *
 * @param hi2c
 * @param interrupts
 * @return
 */
imu_status_t ICM20649_Disable_Interrupts(I2C_HandleTypeDef *hi2c)
{
	uint8_t iconfigbyte = 0;
	if(HAL_I2C_Mem_Read(hi2c,IMU_Device_Address,INT_ENABLE,1,&iconfigbyte,1,100) != HAL_OK)
	{
		return IMU_I2C_ERROR;
	}
	//set enable bit
	iconfigbyte = 0;
	//write to register

	//Clear INT_ENABLE -INT_ENABLE_3
	if(HAL_I2C_Mem_Write(hi2c,IMU_Device_Address,INT_ENABLE,1,&iconfigbyte,1,100) != HAL_OK)
	{
		return IMU_I2C_ERROR;
	}

	if(HAL_I2C_Mem_Write(hi2c,IMU_Device_Address,INT_ENABLE_1,1,&iconfigbyte,1,100) != HAL_OK)
	{
		return IMU_I2C_ERROR;
	}

	if(HAL_I2C_Mem_Write(hi2c,IMU_Device_Address,INT_ENABLE_2,1,&iconfigbyte,1,100) != HAL_OK)
	{
		return IMU_I2C_ERROR;
	}

	if(HAL_I2C_Mem_Write(hi2c,IMU_Device_Address,INT_ENABLE_3,1,&iconfigbyte,1,100) != HAL_OK)
	{
		return IMU_I2C_ERROR;
	}

	return IMU_OK;
}

//======================= 7. Sensor Configuration Function Definitions ==================================

imu_status_t ICM20649_Set_Gyro_FSR(I2C_HandleTypeDef *hi2c,uint8_t FSR)
{
	ICM20649_Register_Bank_Select(&hi2c1, 2);
	  uint8_t byte =0;
	  byte |= FSR;
	if(HAL_I2C_Mem_Write(hi2c,IMU_Device_Address,GYRO_CONFIG_1,1,&byte,1,100)!= HAL_OK)
	{
		return IMU_I2C_ERROR;
	}
	 return IMU_OK;
}

imu_status_t ICM20649_Set_Acc_FSR(I2C_HandleTypeDef *hi2c,uint8_t FSR)
{
	ICM20649_Register_Bank_Select(&hi2c1, 2);
	uint8_t byte = 0;
	byte |= FSR ;
	if(HAL_I2C_Mem_Write(hi2c,IMU_Device_Address,ACCEL_CONFIG,1,&byte,1,100)!= HAL_OK)
	{
		return IMU_I2C_ERROR;
	}
	 return IMU_OK;
}

imu_status_t ICM20649_Set_Sample_Rate(I2C_HandleTypeDef *hi2c)
{
	ICM20649_Register_Bank_Select(&hi2c1, 2);

	//check if DLPF is enabled
	uint8_t configbyte = 0;
	uint8_t smplrtdiv= 0;
	if(HAL_I2C_Mem_Read(hi2c, IMU_Device_Address, GYRO_CONFIG_1, 1, &configbyte, 1, 100) != HAL_OK)
	{
		return IMU_I2C_ERROR;
	}

	//digital low pass value is last 3 bits of CONFIG register
	if((configbyte &0b1) > 0)
	{
		smplrtdiv= (uint8_t)(GYRO_OUTPUT_RATE_DLPF_EN/SAMPLE_RATE - 1);
		printmsg("Gyro DPLF Enabled! \r\n");
	}else
	{
		smplrtdiv = (uint8_t)(GYRO_OUTPUT_RATE_DLPF_DIS/SAMPLE_RATE -1);
	}


	if(HAL_I2C_Mem_Write(hi2c, IMU_Device_Address, GYRO_SMPLRT_DIV, 1, &smplrtdiv, 1, 100) != HAL_OK)
	{
		return IMU_I2C_ERROR;
	}

	printmsg("Gyro smplrt: %d \r\n", smplrtdiv);

	configbyte = 0;
	smplrtdiv = 0;

	if(HAL_I2C_Mem_Read(hi2c, IMU_Device_Address, ACCEL_CONFIG, 1, &configbyte, 1, 100) != HAL_OK)
	{
		return IMU_I2C_ERROR;
	}

	if((configbyte &0b1) > 0)
	{
		printmsg("Accel DPLF Enabled! \r\n");
		smplrtdiv= (uint16_t)(ACC_OUTPUT_RATE_DLPF_EN/SAMPLE_RATE - 1);
	}else
	{
		smplrtdiv = (uint16_t)(ACC_OUTPUT_RATE_DLPF_DIS/SAMPLE_RATE -1);
	}

	uint8_t accSmplrt1;
	uint8_t accSmplrt2;

	accSmplrt1  = (uint8_t) (smplrtdiv >> 8);
	accSmplrt2 =  (uint8_t) (smplrtdiv & 0xFF);

	if(HAL_I2C_Mem_Write(hi2c, IMU_Device_Address, ACCEL_SMPLRT_DIV_1, 1, &accSmplrt1, 1, 100) != HAL_OK)
	{
		return IMU_I2C_ERROR;
	}

	if(HAL_I2C_Mem_Write(hi2c, IMU_Device_Address, ACCEL_SMPLRT_DIV_2, 1, &accSmplrt2, 1, 100) != HAL_OK)
	{
		return IMU_I2C_ERROR;
	}

	printmsg("Accel smplrt: %d \r\n", smplrtdiv);

	uint8_t Align_EN = 1;

	if(HAL_I2C_Mem_Write(hi2c, IMU_Device_Address, ODR_ALIGN_EN, 1, &Align_EN, 1, 100) != HAL_OK)
	{
			return IMU_I2C_ERROR;
	}



	return IMU_OK;
}

imu_status_t ICM20649_Set_FSync(I2C_HandleTypeDef *hi2c, uint8_t Fsync)
{
	//get CONFIG Register
	uint8_t configbyte = 0;
	if(HAL_I2C_Mem_Read(hi2c, IMU_Device_Address, FSYNC_CONFIG, 1, &configbyte, 1, 100) != HAL_OK)
	{
		return IMU_I2C_ERROR;
	}

	configbyte|= (Fsync&0b111000);
	if(HAL_I2C_Mem_Write(hi2c,IMU_Device_Address,FSYNC_CONFIG,1,&configbyte,1,100) != HAL_OK)
	{
		return IMU_I2C_ERROR;
	}
	return IMU_OK;
}

imu_status_t ICM20649_Set_Acc_DLPF(I2C_HandleTypeDef *hi2c, uint8_t DLPF, uint8_t cmd)
{

	ICM20649_Register_Bank_Select(&hi2c1, 2);

	//mask unwanted bits
	uint8_t configbyte = 0;
	if(HAL_I2C_Mem_Read(hi2c, IMU_Device_Address, ACCEL_CONFIG, 1, &configbyte, 1, 100) != HAL_OK)
	{
		return IMU_I2C_ERROR;
	}

	if((cmd = DISABLE))
	{
		configbyte &= ~(ACC_FCHOICE_EN);
	}
	if((cmd = ENABLE))
	{
		configbyte |= (ACC_FCHOICE_EN);
	}else
 	{
 		return IMU_INIT_CMD_ERROR;
 	}

	configbyte |= (DLPF&(0b111<<3));
	if(HAL_I2C_Mem_Write(hi2c,IMU_Device_Address,ACCEL_CONFIG,1,&configbyte,1,100) != HAL_OK)
	{
		return IMU_I2C_ERROR;
	}
	return IMU_OK;
}

imu_status_t ICM20649_Set_Gyro_DLPF(I2C_HandleTypeDef *hi2c, uint8_t DLPF, uint8_t cmd)
{
	ICM20649_Register_Bank_Select(&hi2c1, 2);

	//mask unwanted bits
	uint8_t configbyte = 0;
	if(HAL_I2C_Mem_Read(hi2c, IMU_Device_Address, GYRO_CONFIG_1, 1, &configbyte, 1, 100) != HAL_OK)
	{
		return IMU_I2C_ERROR;
	}


	if((cmd = DISABLE))
	{
		configbyte &= ~(GYRO_FCHOICE_EN);
	}
	if((cmd = ENABLE))
	{
		configbyte |=  (GYRO_FCHOICE_EN);
	}else
 	{
 		return IMU_INIT_CMD_ERROR;
 	}

    configbyte |= (DLPF&(0b111<<3));
	if(HAL_I2C_Mem_Write(hi2c,IMU_Device_Address, GYRO_CONFIG_1,1,&configbyte,1,100) != HAL_OK)
	{
		return IMU_I2C_ERROR;
	}
	return IMU_OK;
}



imu_status_t ICM20649_Set_PLLSrc(I2C_HandleTypeDef *hi2c, uint8_t PLL)
{

	ICM20649_Register_Bank_Select(&hi2c1, 0);

	//get PWR_MGMT_1 reg data
	uint8_t pwrmgmtbyte = 0;
	if(HAL_I2C_Mem_Read(hi2c,IMU_Device_Address,PWR_MGMT_1,1,&pwrmgmtbyte,1,100) != HAL_OK)
	{
		return IMU_I2C_ERROR;
	}
	//configure PLL source
	pwrmgmtbyte = (pwrmgmtbyte&0b11111000)|PLL;

	//write byte to register
	if(HAL_I2C_Mem_Write(hi2c,IMU_Device_Address,PWR_MGMT_1,1,&pwrmgmtbyte,1,100) != HAL_OK)
	{
		return IMU_I2C_ERROR;
	}
	return IMU_OK;
}

imu_status_t ICM20649_Config_Interrupt_Pin( uint8_t level,uint8_t open,  uint8_t latch)
{
	ICM20649_Register_Bank_Select(&hi2c1, 0);

	//get configbyte
	uint8_t byte = 0;
	if(HAL_I2C_Mem_Read(&hi2c1,IMU_Device_Address,INT_PIN_CFG,1,&byte,1,100) != HAL_OK)
	{
		return IMU_I2C_ERROR;
	}
	//clear config params
	byte &= 0x0;
	byte |= (open | level | latch);

	if(HAL_I2C_Mem_Write(&hi2c1,IMU_Device_Address,INT_PIN_CFG,1,&byte,1,100)!= HAL_OK)
	{
		return IMU_I2C_ERROR;
	}
	return IMU_OK;
}

imu_status_t  ICM20649_Config_FIFO(I2C_HandleTypeDef *hi2c, uint8_t accEnable,  uint8_t gyroEnable, uint8_t tempEnable, uint8_t cmd )
{
	ICM20649_Register_Bank_Select(&hi2c1, 0);

	uint8_t fifobyte = 0;

	if(HAL_I2C_Mem_Read(hi2c,IMU_Device_Address, FIFO_EN_2, 1 , &fifobyte, 1, 100) != HAL_OK)
	{
		return IMU_I2C_ERROR;
	}

	if(HAL_I2C_Mem_Write(hi2c,IMU_Device_Address,FIFO_EN_2, 1,&fifobyte,1,100) != HAL_OK)
	{
		return IMU_I2C_ERROR;
	}

	 if(cmd == ENABLE)
	{
		if(accEnable == ENABLE)
		{
			fifobyte |= ACCEL_FIFO_EN;
		}
		if(gyroEnable == ENABLE)
		{
			fifobyte |= GYRO_X_FIFO_EN|GYRO_Y_FIFO_EN|GYRO_Z_FIFO_EN;
		}
		if(tempEnable == ENABLE)
		{
			fifobyte |= TEMP_FIFO_EN;
		}

	}
	else if(cmd == DISABLE)
	{
		fifobyte &= 0;
	}
	if(HAL_I2C_Mem_Write(hi2c,IMU_Device_Address,FIFO_EN_2, 1,&fifobyte,1,100) != HAL_OK)
	{
		return IMU_I2C_ERROR;
	}



	return IMU_OK;
}

imu_status_t ICM20649_FIFO_CMD(I2C_HandleTypeDef *hi2c,uint8_t cmd)
{
	uint8_t fifobyte;
		if(HAL_I2C_Mem_Read(hi2c,IMU_Device_Address,USER_CTRL,1,&fifobyte,1,100) != HAL_OK)
		{
			return IMU_I2C_ERROR;
		}

		if(cmd == ENABLE)
		{
			fifobyte |= USER_CTRL_FIFO_EN;
		}else if(cmd == DISABLE)
		{
			fifobyte &= ~(USER_CTRL_FIFO_EN);
		}else
		{
			//invalid cmd sent
			return IMU_INIT_CMD_ERROR;
		}
		//write byte to user control
		if(HAL_I2C_Mem_Write(hi2c,IMU_Device_Address,USER_CTRL,1,&fifobyte,1,100) != HAL_OK)
		{
			return IMU_I2C_ERROR;
		}
		return IMU_OK;
}

imu_status_t ICM20649_Register_Bank_Select(I2C_HandleTypeDef *hi2c,uint8_t reg)
{
	uint8_t byte = 0;
	byte = reg<<4;

	if(HAL_I2C_Mem_Write(hi2c, IMU_Device_Address, REG_BANK_SEL, 1, &byte, 1, 100)!= HAL_OK)
	{
		return IMU_I2C_ERROR;
	}
	 return IMU_OK;
}

imu_status_t ICM20649_Disable_LPF(I2C_HandleTypeDef *hi2c, uint8_t accel_dis_lpf, uint8_t gyro_dis_lpf)
{
	ICM20649_Register_Bank_Select(&hi2c1, 2);
	uint8_t iconfigbyte = 0;
	if(HAL_I2C_Mem_Read(hi2c, IMU_Device_Address, ACCEL_CONFIG, 1, &iconfigbyte, 1, 100) != HAL_OK)
	{
		return IMU_I2C_ERROR;
	}
	//set enable bit
	iconfigbyte |= accel_dis_lpf;
	//write to register
	if(HAL_I2C_Mem_Write(hi2c,IMU_Device_Address, ACCEL_CONFIG,1,&iconfigbyte,1,100) != HAL_OK)
	{
		return IMU_I2C_ERROR;
	}

	iconfigbyte = 0;
	if(HAL_I2C_Mem_Read(hi2c, IMU_Device_Address, gyro_dis_lpf, 1, &iconfigbyte, 1, 100) != HAL_OK)
	{
		return IMU_I2C_ERROR;
	}

	//set enable bit
	iconfigbyte |= gyro_dis_lpf;
	//write to register
	if(HAL_I2C_Mem_Write(hi2c,IMU_Device_Address, GYRO_CONFIG_1,1,&iconfigbyte,1,100) != HAL_OK)
	{
		return IMU_I2C_ERROR;
	}

	return IMU_OK;
}

//======================= 8. Sensor Read Functions Definitions =============================================

 imu_status_t ICM20649_Get_ID(I2C_HandleTypeDef *hi2c,uint8_t* ID)
 {

	 ICM20649_Register_Bank_Select(&hi2c1, 0);

	 uint8_t whoami;
	  //check if device is I2C ready
	  if(HAL_I2C_IsDeviceReady(&hi2c1,IMU_Device_Address,10,100)== HAL_OK)
	  {
		  	  //read 1 byte of WHO_AM_I register into variable
			  HAL_StatusTypeDef flag = HAL_I2C_Mem_Read(&hi2c1,IMU_Device_Address,WHO_AM_I,1,&whoami,1,100);
			  //verify read was successful
			  if(flag != HAL_OK)
			  {
				 if(flag == HAL_BUSY)
				 {
					 return IMU_I2C_DEVICE_BUSY;
				 }

				 return IMU_I2C_ERROR;
			  }
			  *ID = whoami;
	  }
	  else
	  {
		  //unable to connect to device
		  return IMU_I2C_DEVICE_OFFLINE;
	  }
	  return IMU_OK;
 }


imu_status_t ICM20649_Get_SelfTestResponse_Values(I2C_HandleTypeDef *hi2c,IMU_SelfTest_t *mpu)
 {
	 uint8_t temp[4]={0};
	 if(HAL_I2C_Mem_Read(hi2c,IMU_Device_Address,SELF_TEST_X_GYRO,1,temp,4,100)!= HAL_OK)
	 {
		 return IMU_I2C_ERROR;
	 }
	 mpu->A_x = (temp[0]&0b1110000)>>2 | ((temp[3]&0b00110000)>>4);
	 mpu->G_x =  temp[0]&0b0001111;
	 mpu->A_y = (temp[1]&0b1110000)>>2 | ((temp[3]&0b00001100)>>2);
	 mpu->G_y =  temp[1]&0b0001111;
	 mpu->A_z = (temp[2]&0b1110000) |	(temp[3]&0b11);
	 mpu->G_z =  temp[2]&0b0001111;
	 return IMU_OK;
 }

imu_status_t ICM20649_Get_Interrupt_Status(I2C_HandleTypeDef *hi2c, Interrupt_source_t interrupt_src,uint8_t* res)
{
	ICM20649_Register_Bank_Select(&hi2c1, 0);

	uint8_t istatus = 0;


	 switch (interrupt_src)
	{
		case FIFO_OVERFLOW:
				if(HAL_I2C_Mem_Read(hi2c,IMU_Device_Address,INT_STATUS_1,1,&istatus,1,100) != HAL_OK)
				{
					return IMU_I2C_ERROR;
				}
			*res = istatus;
			break;
		case DATA_READY:
			if(HAL_I2C_Mem_Read(hi2c,IMU_Device_Address,INT_STATUS_2,1,&istatus,1,100) != HAL_OK)
			{
				return IMU_I2C_ERROR;
			}
			*res = istatus;
		case FIFO_WATERMARK:
			if(HAL_I2C_Mem_Read(hi2c,IMU_Device_Address,INT_STATUS_3,1,&istatus,1,100) != HAL_OK)
			{
				return IMU_I2C_ERROR;
			}
			*res = istatus;
		default:
			break;

	}

	return IMU_OK;
}


imu_status_t ICM20649_Is_Data_Ready(I2C_HandleTypeDef *hi2c, uint8_t *status)
{
	uint8_t data_status = 0;

	ICM20649_Register_Bank_Select(&hi2c1, 0);

	if(HAL_I2C_Mem_Read(&hi2c1,IMU_Device_Address, INT_STATUS_1, I2C_MEMADD_SIZE_8BIT, &data_status, 1, 10) != HAL_OK)
	{
		return IMU_I2C_ERROR;
	}

    *status = data_status&RAW_DATA_0_RDY_INT;

    return IMU_OK;
}


imu_status_t ICM20649_Get_IMU_RawData(I2C_HandleTypeDef *hi2c, uint8_t *imu)
{
	if(HAL_I2C_Mem_Read(&hi2c1, IMU_Device_Address, ACCEL_XOUT_H, 1, imu, 12, 10) != HAL_OK)
	{
		return IMU_I2C_ERROR;
	}

	return IMU_OK;
}


imu_status_t ICM20649_Get_FIFO_Count(I2C_HandleTypeDef *hi2c, uint16_t* count)
{

	uint8_t fifo_count[2] = {0};

	if(HAL_I2C_Mem_Read(hi2c, IMU_Device_Address,FIFO_COUNTH,1, fifo_count,2,100) != HAL_OK)
	{
		return IMU_I2C_ERROR;
	}

	*count = (int16_t)fifo_count[0]<<8 | (int16_t)fifo_count[1];

	return IMU_OK;

}

imu_status_t ICM20649_Get_IMU_RawData_FIFO(I2C_HandleTypeDef *hi2c, uint8_t* imuBuf, uint8_t count)
{



	/*if (HAL_I2C_GetState(hi2c) != HAL_I2C_STATE_READY)
	{
			HAL_Delay(1);
	}*/

	//ICM20649_Register_Bank_Select(&hi2c1, 0);

	if(HAL_I2C_Mem_Read_DMA(hi2c, IMU_Device_Address, FIFO_R_W, 1, FIFO_Buffer, 4080)!= HAL_OK)
	{
		HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_14);
	}


	return IMU_OK;

}


imu_status_t Reset_FIFO(I2C_HandleTypeDef *hi2c)
{

	ICM20649_Register_Bank_Select(&hi2c1, 0);

	uint8_t fifo_rst_byte = 0;
	fifo_rst_byte = 0x0F;

		if(HAL_I2C_Mem_Write(hi2c,IMU_Device_Address,FIFO_RST,1,&fifo_rst_byte,1,5) != HAL_OK)
		{
			return IMU_I2C_ERROR;
		}

		//Deassert Reset (need to assert and deassert for full reset
		fifo_rst_byte = 0x0;
		if(HAL_I2C_Mem_Write(hi2c,IMU_Device_Address,FIFO_RST,1,&fifo_rst_byte,1,5)!= HAL_OK)
		{
			return IMU_I2C_ERROR;
		}

		return IMU_OK;

}



//======================= 9. Power Mode Config Function Definitions =======================================

imu_status_t ICM20649_Signal_conditioned_Reset(I2C_HandleTypeDef *hi2c)
{
	//write reset condition to USER_CTRL
	uint8_t byte = USER_CTRL_SIG_COND_RESET;
	if(HAL_I2C_Mem_Write(hi2c,IMU_Device_Address,USER_CTRL,1,&byte,1,100) != HAL_OK)
	{
		return IMU_I2C_ERROR;
	}
	(void)byte;
	//wait for sig cond bit to reset
	uint8_t sig_reset_complete = 1;
	while(sig_reset_complete)
	{
		if(HAL_I2C_Mem_Read(hi2c,IMU_Device_Address,USER_CTRL,1,&sig_reset_complete,1,100) != HAL_OK)
		{
			return IMU_I2C_ERROR;
		}
		sig_reset_complete &= 0b1;
	}
	return IMU_OK;
}

imu_status_t ICM20649_Set_Low_Power_Mode_Acc(I2C_HandleTypeDef *hi2c,uint8_t Cycles)
{
	uint8_t byte[2] = {0};
	byte[0]  = (PWR_MGMT_1_LP_EN | PWR_MGMT_1_TEMP_DIS);
	byte[1]  = (Cycles|ACCEL_CYCLE|GYRO_CYCLE);
	 if(HAL_I2C_Mem_Write(hi2c,IMU_Device_Address,PWR_MGMT_1,1,&byte[0],1,100)!= HAL_OK)
	 {
			return IMU_I2C_ERROR;
	 }
	 if(HAL_I2C_Mem_Write(hi2c,IMU_Device_Address,LP_CONFIG,1,&byte[1],1,100)!= HAL_OK)
	{
			return IMU_I2C_ERROR;
	}
	 return IMU_OK;

}

imu_status_t ICM20649_Set_Wake(I2C_HandleTypeDef *hi2c)
 {
	ICM20649_Register_Bank_Select(&hi2c1, 0);

	 uint8_t byte[2] = {0b00000000,0b00000000};

	 if(HAL_I2C_Mem_Write(hi2c, IMU_Device_Address, PWR_MGMT_1, 1, &byte[0], 1, 100)!= HAL_OK)
	{
		return IMU_I2C_ERROR;
	}
	if(HAL_I2C_Mem_Write(hi2c, IMU_Device_Address, PWR_MGMT_2, 1, &byte[1], 1, 100)!= HAL_OK)
	{
	 	return IMU_I2C_ERROR;
	}
	 return IMU_OK;
 }

imu_status_t ICM20649_Set_Sleep_Power_Mode(I2C_HandleTypeDef *hi2c)
 {
	 uint8_t byte = PWR_MGMT_1_SLEEP_EN;

	 if(HAL_I2C_Mem_Write(hi2c,IMU_Device_Address,PWR_MGMT_1,1,&byte,1,100)!= HAL_OK)
	{
		return IMU_I2C_ERROR;
	}

	 return IMU_OK;
 }

imu_status_t ICM20649_reset(I2C_HandleTypeDef *hi2c)
{
	//Select Register Bank
	ICM20649_Register_Bank_Select(&hi2c1, 0);

	//set bit in register
	uint8_t pwrmgmtbyte = (uint8_t)PWR_MGMT_1_DEVICE_RESET;
	if(HAL_I2C_Mem_Write(hi2c,IMU_Device_Address,PWR_MGMT_1,1,&pwrmgmtbyte,1,2) != HAL_OK)
	{
		return IMU_I2C_ERROR;
	}
	//wait for bit to clear
	uint8_t retries = 0;
	while((pwrmgmtbyte &PWR_MGMT_1_DEVICE_RESET)!=0)
	{
		//poll data register untill bit is cleared
		if(HAL_I2C_Mem_Read(&hi2c1,IMU_Device_Address,PWR_MGMT_1,1,&pwrmgmtbyte,1,2)!= HAL_OK)
		{
			//failiure to read register
				retries++;
		}
		if(retries >= 100)
		{
			return IMU_RESET_FAIL;
		}
	}

	return IMU_OK;
}

//======================= 10. Data Processing Function Definitions =======================================
/*
 * @brief:
 *
 * This function is used to perform a self test on the device. This function
 * should be run during a power on/ first time start.
 *
 * @description:
 * Self Test is applied to all 6 axes of imu. Individual axis can be enabled for self test
 * by setting the nA_ST bit in the Acceleration Config Register (n is the x,y,z axis)
 *
 * Once, activated, the sensors are electronically actuated over a set distance simulating an
 * external force. Once complete, a corresponding output signal is produced which is then
 * subtracted from the values in the self test register resulting in a self test response.
 *
 *@set up:
 *@set GYRO FSR to 250 dps
 *@set ACCEL range to 8+-g
 *@set PLL clock source to x-axis gyro ref
 *
 *@param: hi2c - i2c handler
 *@param: test_res - an array to hold the results of the self test for each axis. Note: must be an array with a length >= 6
 *
 *@return imu_status_t - value to show the status of the function
 *
 */
imu_status_t ICM20649_SelfTest(I2C_HandleTypeDef *hi2c,float* test_res)
{
	// set clock source to PLL gyro x axis
	ICM20649_Set_PLLSrc(&hi2c1, PWR_MGMT_1_CLK_SEL_1);
	//set Gyro FSR to 250 dps
	ICM20649_Set_Gyro_FSR(&hi2c1, GYRO_CONFIG_FSSEL_500DPS);
	//set Acc FSR to +-8g
	ICM20649_Set_Acc_FSR(&hi2c1,ACC_CONFIG_AFSSEL_8G);
	//enable selftest
	uint8_t stbyte[2];
	 if(HAL_I2C_Mem_Read(hi2c,IMU_Device_Address,GYRO_CONFIG_1, 1, stbyte, 2, 100)!= HAL_OK)
	{
		return IMU_I2C_ERROR;
	}
	stbyte[0] |= GYRO_CONFIG_ST_EN_X | GYRO_CONFIG_ST_EN_Y | GYRO_CONFIG_ST_EN_Z;
	stbyte[1] |= ACC_CONFIG_ST_EN_X | ACC_CONFIG_ST_EN_Y | ACC_CONFIG_ST_EN_Z;
	 if(HAL_I2C_Mem_Write(hi2c,IMU_Device_Address,GYRO_CONFIG_2,1,stbyte,2,100)!= HAL_OK)
	{
		return IMU_I2C_ERROR;
	}
	 HAL_I2C_Mem_Read(hi2c,IMU_Device_Address,GYRO_CONFIG_2,1,stbyte,2,100);
	HAL_Delay(250); //TODO: Replace with a wait till self test updated function. Value taken from https://github.com/kriswiner/ICM20649/blob/master/ICM20649BasicExample.ino line 662

	//read Self Test register
	IMU_SelfTest_t mpu_str;
	IMU_FT_t mpu_ft;
	if(ICM20649_Get_SelfTestResponse_Values(&hi2c1,&mpu_str) != IMU_OK)
	{
		return IMU_STR_READ_ERROR;
	}
	//calculate Factory Trim for gyro
	mpu_ft.G_x = 25*131*pow(1.046,(float)mpu_str.G_x -1);
	mpu_ft.G_y =-25*131*pow(1.046,(float)mpu_str.G_y -1);
	mpu_ft.G_z = 25*131*pow(1.046,(float)mpu_str.G_z -1);

	//calculate Factory Trim for Acc
	mpu_ft.A_x = 4096*0.34*pow(0.92/0.34,(((float)mpu_str.A_x-1)/30.0));
	mpu_ft.A_y = 4096*0.34*pow(0.92/0.34,(((float)mpu_str.A_y-1)/30.0));
	mpu_ft.A_z = 4096*0.34*pow(0.92/0.34,(((float)mpu_str.A_z-1)/30.0));

	//calculate factory trim change
	test_res[0] = 100+ 100*((float)mpu_str.A_x - mpu_ft.A_x)/mpu_ft.A_x;
	test_res[1] = 100+ 100*((float)mpu_str.A_y - mpu_ft.A_y)/mpu_ft.A_y;
	test_res[2] = 100+ 100*((float)mpu_str.A_z - mpu_ft.A_z)/mpu_ft.A_z;
	test_res[3] = 100+ 100*((float)mpu_str.G_x - mpu_ft.G_x)/mpu_ft.G_x;
	test_res[4] = 100+ 100*((float)mpu_str.G_y - mpu_ft.G_y)/mpu_ft.G_y;
	test_res[5] = 100+ 100*((float)mpu_str.G_z - mpu_ft.G_z)/mpu_ft.G_z;
	return IMU_OK;


}

imu_status_t ICM20649_Calibrate_Acc(I2C_HandleTypeDef *hi2c,float* accel_bias)
{
	//Initialise registers
	ICM20649_reset(&hi2c1);
	ICM20649_Set_Wake(&hi2c1); //wake up device
	ICM20649_Set_PLLSrc(&hi2c1, PWR_MGMT_1_CLK_SEL_1); // set clock source to PLL gyro x axis
	HAL_Delay(400);
	ICM20649_Init_FIFO(&hi2c1,DISABLE);
	ICM20649_Init_FIFO(&hi2c1,IMU_RESET);

	//configure Gyro and Accelerometer
	ICM20649_Set_Gyro_FSR(&hi2c1,GYRO_CONFIG_FSSEL_500DPS); //set Gyro FSR to 500 dps
	ICM20649_Set_Acc_FSR(&hi2c1,ACC_CONFIG_AFSSEL_8G);		//set Acc FSR to +-8g
	int32_t acc_res =ACC_8G_WORD_LENGTH;
	ICM20649_Set_Gyro_DLPF(&hi2c1, GYRO_DPLFCFG_1, ENABLE);					//set dlpf to 188 Hz
	ICM20649_Set_Sample_Rate(&hi2c1);						// set sample rate to user defined value in HAL_ICM20649.h
	ICM20649_Init_FIFO(&hi2c1,ENABLE);						//enable FIFO
	//enable acc and gyro in FIFO buffer
	ICM20649_Enable_Interrupt(&hi2c1, INT_ENABLE_FIFO_OFLOW_EN, 2);
	//ICM20649_Config_FIFO(&hi2c1,FIFO_EN_ACC,ENABLE);
	//wait until fifo is completed
	uint8_t interrupt_status = 0;
	while(!interrupt_status)
	{
		 ICM20649_Get_Interrupt_Status(&hi2c1,FIFO_OVERFLOW,&interrupt_status);
	}
	//ICM20649_Config_FIFO(&hi2c1,FIFO_EN_ACC,DISABLE);
	uint16_t count = 0;
	ICM20649_Get_FIFO_Count(&hi2c1,&count);
	uint8_t buffer[1024];
	uint8_t addr = FIFO_R_W;

	if(HAL_I2C_Master_Transmit(&hi2c1,IMU_Device_Address,&addr,1,100)== HAL_OK)
	{
		if(HAL_I2C_Master_Seq_Receive_DMA(&hi2c1,IMU_Device_Address,buffer,count,I2C_FIRST_FRAME) != HAL_OK)
		{
			return IMU_FIFO_READ_ERROR;
		}
		//wait for read to finish
		while(I2C_TX_CPLT != 1);

	}
	//Processing algorithm:
	int32_t n_s = count/6; //3 axes, 2 bytes per axis
	int32_t imu_bias[3] = {0};	//data array for storing the
	int16_t acc_temp[3] ={0};// gyro_temp[3] = {0};
	for (int i = 0; i < n_s; ++i)
	{
		//get set of imu data
		acc_temp[0]= ((int16_t)buffer[6*i])<<8 | ((int16_t)buffer[6*i+1]&0xFF);
		acc_temp[1]= ((int16_t)buffer[6*i+2])<<8 | ((int16_t)buffer[6*i+3]&0xFF);
		acc_temp[2]= ((int16_t)buffer[6*4])<<8 | ((int16_t)buffer[6*i+5]&0xFF);
		int j = 0;
		for (j = 0; j < 3; ++j)
		{
			imu_bias[j] += (int32_t)acc_temp[j];
		}
	}
	//divide total by number of samples to get offset
	imu_bias[0] /= n_s;
	imu_bias[1] /=n_s;
	imu_bias[2] = (imu_bias[2] - acc_res)/n_s;
	accel_bias[0] = (float)imu_bias[0]/(float)acc_res;
	accel_bias[1] = (float)imu_bias[1]/(float)acc_res;
	accel_bias[2] = (float)imu_bias[2]/(float)acc_res;
	return IMU_CAL_SUCCESS;
}


//======================= 11. IRQ Handler Functions =======================================

void ICM20649_IntPin_IRQ()
{

	//HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_7);
	//ICM20649_Get_IMU_RawData(&hi2c1, IMU_Buffer);
	fifo_sample_complete = 1;
	__HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_14);

}


void ICM20649_FIFO_Interrupt_IRQ(void)
{

	HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_7);

	ICM20649_Get_IMU_RawData_FIFO(&hi2c1, FIFO_Buffer, 0);

	__HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_14);

}

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *I2cHandle)
{
  /* Turn LED1 on: Transfer in transmission process is correct */
	Reset_FIFO(&hi2c1);
	fifo_sample_count = 1;
	//memcpy(IMU_Buffer, FIFO_Buffer, 4080);
	//printmsg("I2C Transfer complete!");
}


 /*
 * NB!!! In order to use the functions, you need to call them in the respective IRQ Handlers as per the vector table
 * in the startup.s file. This can also be accomplished by uncommenting the following code and placing it in the
 * stm32l4xx_it.c file:
 *
 * note: if the project does not include this file, you can make your own or declare the IRQhanlder in another location
 *
 */

///**
//  * @brief This function handles DMA1 channel7 global interrupt.
//  */
//void DMA1_Channel7_IRQHandler(void)
//{
//  /* USER CODE BEGIN DMA1_Channel7_IRQn */
//  HAL_DMA_IRQHandler(&hdma_i2c1_rx);
//  imu6050_DMA_PeriphIRQHandler();
//
//}
