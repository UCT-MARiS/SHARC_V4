/*
 * HAL_SD.c
 *
 *  Created on: Apr 20, 2022
 *      Author: Michael Noyce
 */

//======================== 1. Includes ==============================================================

#include "HAL_SD.h"
#include "string.h"
#include "main.h"
//======================== 2. Global Variables =====================================================

extern SD_HandleTypeDef hsd1;
FATFS SDFatFs; /* File system object for User logical drive */
FIL File; /* File object */
DIR Dir; /*Directory Object*/
FILINFO fno;
uint32_t waveLogNo;
uint32_t waveDirNo;
uint32_t gpsLogNo;
uint32_t gpsDirNo;
uint32_t envLogNo;
uint32_t envDirNo;
uint32_t pwrLogNo;
uint32_t pwrDirNo;

//======================== 3. Static Functions Prototypes ===========================================
static void MX_GPIO_Init(void);
static void MX_SDMMC1_SD_Init(void);
static void MX_CRC_Init(void);
static void MX_RTC_Init(void);
void MX_FATFS_Init();

//======================== 4. Static Functions Definition ===========================================

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  HAL_PWREx_EnableVddIO2();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);

  /*Configure GPIO pin : SD_Detect_Pin */
  GPIO_InitStruct.Pin = SD_Detect_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(SD_Detect_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD3_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/**
  * @brief SDMMC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SDMMC1_SD_Init(void)
{

  /* USER CODE BEGIN SDMMC1_Init 0 */

  /* USER CODE END SDMMC1_Init 0 */

  /* USER CODE BEGIN SDMMC1_Init 1 */

  /* USER CODE END SDMMC1_Init 1 */
  hsd1.Instance = SDMMC1;
  hsd1.Init.ClockEdge = SDMMC_CLOCK_EDGE_RISING;
  hsd1.Init.ClockPowerSave = SDMMC_CLOCK_POWER_SAVE_DISABLE;
  hsd1.Init.BusWide = SDMMC_BUS_WIDE_1B;
  hsd1.Init.HardwareFlowControl = SDMMC_HARDWARE_FLOW_CONTROL_DISABLE;
  hsd1.Init.ClockDiv = 1;
  hsd1.Init.Transceiver = SDMMC_TRANSCEIVER_DISABLE;
  /* USER CODE BEGIN SDMMC1_Init 2 */

  /* USER CODE END SDMMC1_Init 2 */

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
 *  3. In the stm32l4xx_hal_msp.c file, include the header "HAL_BMP280.h"
 */



//======================== 6. SD Configuration Functions =========================================


sd_status_t SD_Init(void)
{
	MX_FATFS_Init();
	MX_GPIO_Init();
	MX_SDMMC1_SD_Init();

	return SD_OK;
}

sd_status_t SD_Mount(FATFS SDFatFs, char diskPath[])
{

	if(f_mount(&SDFatFS, (TCHAR const*)diskPath, 0) != FR_OK)
	{
	  return SD_MOUNT_ERROR;
	}

	return SD_OK;
}


sd_status_t SD_Unmount(FATFS SDFatFs)
{
	char diskPath[10];

	if(f_mount(0, (TCHAR const*)diskPath, 0) != FR_OK)
	{
	  printmsg("SD unmount error \r\n");
	  return SD_MOUNT_ERROR;
	}
	else
	{
		//printmsg("Dismount successful!");
	}

	return SD_OK;
}


sd_status_t SD_File_Open(FIL *myFile, uint32_t fileNumber)
{

	  char diskPath[10]; /* User logical drive path */
	  sd_status_t errorCode;
	  FRESULT res;
	  uint32_t byteswritten = 0;

	  errorCode = SD_Init();

	  if(SD_Mount(SDFatFs, diskPath) == SD_OK)
	  {
		  sprintf(diskPath, "Log%ld.txt", fileNumber);
		   if(f_open(myFile, (const TCHAR*)diskPath, FA_CREATE_ALWAYS | FA_WRITE) == FR_OK)
		    {
			  //printmsg("Opened file!");
		    }
		  else
		  {
			return SD_OPEN_ERROR;
			printmsg("Open failed!");
		  }
	  }
	  else
	  {
		  printmsg("Mount failed");
		  return SD_MOUNT_ERROR;
	  }

	  return SD_OK;

}

sd_status_t SD_File_Write(FIL *myFile, uint8_t *dataBuffer)
{

	uint32_t byteswritten = 0 ;  /* File write counts */

	if(f_write(myFile, dataBuffer, strlen((char *)dataBuffer), (void *)&byteswritten)!= FR_OK)
	{
		printmsg("Write failed!");
		return SD_WRITE_ERROR;
	}

	if(byteswritten == 0)
    {
	  printmsg("Write failed!");
  	  return SD_WRITE_ERROR;
    }



	return SD_OK;
}

sd_status_t SD_File_Close(FIL *myFile)
{
	if(f_close(myFile) != FR_OK)
	{
		printmsg("Close Failed!");
		return SD_CLOSE_ERROR;
	}


	return SD_OK;
}

/**
 * @brief Closes Current Directory
 *
 * @param myDir
 * @return
 */
sd_status_t SD_Dir_Close(DIR *myDir)
{
	if(f_closedir(myDir) != FR_OK)
	{
		printmsg("CloseDir Failed!");
		return SD_CLOSE_ERROR;
	}

	return SD_OK;
}

/**
 * @brief Manually timestmap file if get_fattime() in fatfs.c is not used
 *
 * @param myFile
 * @return
 */
/*
sd_status_t SD_File_TimeStamp(FIL *myFile)
{
	RTC_DateTypeDef gDate;
	RTC_TimeTypeDef gTime;

	HAL_RTC_GetTime(&hrtc, &gTime, RTC_FORMAT_BIN);

	HAL_RTC_GetDate(&hrtc, &gDate, RTC_FORMAT_BIN);

	FILINFO fno;

	fno.fdate = (WORD)(((gDate.Year - 1980) * 512U) | gDate.Month * 32U | gDate.Date);
	fno.ftime = (WORD)(gTime.Hours * 2048U | gTime.Minutes * 32U | gTime.Seconds / 2U);


	if(f_utime(myFile, &fno) != FR_OK)
		{
			printmsg("Timestamp Failed!");
			return SD_TIME_ERROR;
		}

		return SD_OK;

} */





/**
 * @brief Function to open a wave log file for writing local data buffers to.
 *
 * Description: The function opens a wavLog file in a waveDir directory to be written to
 *
 *
 * @param myFile
 * @param waveBufferSegment
 * @return
 */
sd_status_t SD_Wave_Open(FIL *myFile, DIR *myDir, FILINFO* fno, uint32_t waveDirNo, uint32_t waveLogNo)
{

     char diskPath[14]; /* User logical drive path */
     char dirPath[14]; //max path length is 14 without long path names enabled

	  if(SD_Mount(SDFatFs, diskPath) == SD_OK)
	  {

		  sprintf(dirPath, "WD");

		  if(f_stat((const TCHAR*)dirPath, fno) == FR_NO_FILE)
		  {
		  	 if(f_mkdir((const TCHAR*)dirPath) != FR_OK)
		  	 {
		  		printmsg("Main Wave Directory not created \r\n");
		  	 }
		  }


		  sprintf(dirPath, "WD/WS%d", waveDirNo);

		  if(f_stat((const TCHAR*)dirPath, fno) == FR_NO_FILE)
		  {

			  FRESULT status = f_mkdir((const TCHAR*)dirPath);

			  if(status != FR_OK)
			  {
			  	printmsg("Directory not created, error %d\r\n", status);
			  }
		  }

			  sprintf(diskPath, "WD/WS%d/W%d.txt", waveDirNo, waveLogNo);
			  FRESULT status = f_open(myFile, (const TCHAR*)diskPath, FA_OPEN_APPEND | FA_WRITE);
			  if(status == FR_OK)
			  {
				  printmsg("Opened file! \r\n");
			  }
			  else
			  {
				  printmsg("File not opened, error %d\r\n", status);
				  return SD_OPEN_ERROR;
			  }
	  }
	  else
	  {
		   printmsg("Mount failed");
		   return SD_MOUNT_ERROR;
	  }


	  return SD_OK;


}


sd_status_t SD_GPS_Open(FIL *myFile, DIR *myDir, FILINFO* fno, uint32_t GPSDirNo, uint8_t GPSLogNo)
{

	 //printmsg("SDMMC Interface Starting! \r\n");
     char diskPath[16]; /* User logical drive path */
     char dirPath[16]; //max path length is 14 without long path names enabled

     if(SD_Mount(SDFatFs, diskPath) == SD_OK)
     {

     	sprintf(dirPath, "GD");

		 if(f_stat((const TCHAR*)dirPath, fno) == FR_NO_FILE)
		 {
			if(f_mkdir((const TCHAR*)dirPath) != FR_OK)
			{
				printmsg("Directory not created");
			}
		 }


		 	 sprintf(dirPath, "GD/GS%d", GPSDirNo);

			 if(f_stat((const TCHAR*)dirPath, fno) == FR_NO_FILE)
			 {

				 if(f_mkdir((const TCHAR*)dirPath) != FR_OK)
				 {
					 printmsg("Directory not created \r\n");
				 }
			 }

			 	 sprintf(diskPath, "GD/GS%d/G%d.txt", GPSDirNo, GPSLogNo);

				  if(f_open(myFile, (const TCHAR*)diskPath, FA_CREATE_ALWAYS | FA_WRITE) == FR_OK)
				  {
					  //printmsg("Opened file! \r\n");
				  }
				  else
				  {
					  printmsg("Open failed! \r\n");
					  return SD_OPEN_ERROR;
				  }
	  }
	  else
	  {
		   printmsg("Mount failed \r\n");
		   return SD_MOUNT_ERROR;
	  }


	  return SD_OK;


}

sd_status_t SD_ENV_Open(FIL *myFile, DIR *myDir, FILINFO* fno, uint32_t ENVDirNo, uint8_t ENVLogNo)
{

	 //printmsg("SDMMC Interface Starting! \r\n");
     char diskPath[16]; /* User logical drive path */
     char dirPath[16]; //max path length is 14 without long path names enabled

     if(SD_Mount(SDFatFs, diskPath) == SD_OK)
     {

     	sprintf(dirPath, "ED");

		 if(f_stat((const TCHAR*)dirPath, fno) == FR_NO_FILE)
		 {
			if(f_mkdir((const TCHAR*)dirPath) != FR_OK)
			{
				printmsg("Directory not created");
			}
		 }


		 	 sprintf(dirPath, "ED/ES%d", ENVDirNo);

			 if(f_stat((const TCHAR*)dirPath, fno) == FR_NO_FILE)
			 {

				 if(f_mkdir((const TCHAR*)dirPath) != FR_OK)
				 {
					 printmsg("Directory not created \r\n");
				 }
			 }

			 	 sprintf(diskPath, "ED/ES%d/E%d.txt", ENVDirNo, ENVLogNo);

				  if(f_open(myFile, (const TCHAR*)diskPath, FA_CREATE_ALWAYS | FA_WRITE) == FR_OK)
				  {
					  //printmsg("Opened file! \r\n");
				  }
				  else
				  {
					  printmsg("Open failed! \r\n");
					  return SD_OPEN_ERROR;
				  }
	  }
	  else
	  {
		   printmsg("Mount failed \r\n");
		   return SD_MOUNT_ERROR;
	  }


	  return SD_OK;


}

sd_status_t SD_PWR_Open(FIL *myFile, DIR *myDir, FILINFO* fno, uint32_t PWRDirNo, uint8_t PWRLogNo)
{

	 //printmsg("SDMMC Interface Starting! \r\n");
     char diskPath[16]; /* User logical drive path */
     char dirPath[16]; //max path length is 14 without long path names enabled

     if(SD_Mount(SDFatFs, diskPath) == SD_OK)
     {

     	sprintf(dirPath, "PD");

		 if(f_stat((const TCHAR*)dirPath, fno) == FR_NO_FILE)
		 {
			if(f_mkdir((const TCHAR*)dirPath) != FR_OK)
			{
				printmsg("Directory not created");
			}
		 }


		 	 sprintf(dirPath, "PD/PS%d", PWRDirNo);

			 if(f_stat((const TCHAR*)dirPath, fno) == FR_NO_FILE)
			 {

				 if(f_mkdir((const TCHAR*)dirPath) != FR_OK)
				 {
					 printmsg("Directory not created \r\n");
				 }
			 }

			 	 sprintf(diskPath, "PD/PS%d/P%d.txt", PWRDirNo, PWRLogNo);

				  if(f_open(myFile, (const TCHAR*)diskPath, FA_CREATE_ALWAYS | FA_WRITE) == FR_OK)
				  {
					  //printmsg("Opened file! \r\n");
				  }
				  else
				  {
					  printmsg("Open failed! \r\n");
					  return SD_OPEN_ERROR;
				  }
	  }
	  else
	  {
		   printmsg("Mount failed \r\n");
		   return SD_MOUNT_ERROR;
	  }


	  return SD_OK;


}

sd_status_t SD_Wave_Read(FIL *myFile, int32_t *IMUArray,  uint32_t WaveDirNo, uint32_t WaveLogNo, IMU_Data_SD_t inertialDataType, uint32_t *fpointer)
{

	uint8_t dataBuf[60];
	int maxLen = 60;
	char diskPath[16]; /* User logical drive path */
	FSIZE_t fp = *fpointer;

	char *p; //pointer for strtok functions
	const char delim[] = ", ";
	char tempString[60];


	sprintf(diskPath, "WD/WS%d/W%d.txt", waveDirNo, waveLogNo);


	  if(SD_Mount(SDFatFs, diskPath) == SD_OK)
	     {

			 sprintf(diskPath, "WD/WS%d/W%d.txt", WaveDirNo, WaveLogNo);

			  if(f_open(myFile, (const TCHAR*)diskPath, FA_READ) == FR_OK)
			  {

				  //Open file to correct location
				  fp = *fpointer;
				  printmsg("Initial file pointer location %ld \r\n", *fpointer);
				  f_lseek(myFile, fp);

				  for(int i = 0; i<=1024; i++)
				  {
					  f_gets(dataBuf, maxLen, myFile);

					  p = strtok(dataBuf, delim);

					  //Value in string is determined by IMU_data_t enum
					  for(int i=0; i<inertialDataType; i++)
					  {
						  p = strtok(NULL, delim);
					  }

					  IMUArray[i] = atoi(p);

				  }

				  //store current file pointer location
				  *fpointer = f_tell(myFile);

			  }
			  else
			  {
				  printmsg("Open failed! \r\n");
				  return SD_OPEN_ERROR;
			  }


		  }
		  else
		  {
			   printmsg("Mount failed \r\n");
			   return SD_MOUNT_ERROR;
		  }

	  return SD_OK;

}

sd_status_t SD_GPS_Read(FIL *myFile, float *GPSArray,  uint32_t GPSDirNo, uint32_t GPSLogNo, uint32_t *fpointer)
{

	uint8_t dataBuf[60];
	int maxLen = 500;
	char diskPath[16]; /* User logical drive path */
	FSIZE_t fp = *fpointer;

	char *p; //pointer for strtok functions
	const char delim[] = ", ";
	char tempString[60];


	sprintf(diskPath, "GD/GS%d/G%d.txt", GPSDirNo, GPSLogNo);


	  if(SD_Mount(SDFatFs, diskPath) == SD_OK)
	     {

			 sprintf(diskPath, "GD/GS%d/G%d.txt", GPSDirNo, GPSLogNo);

			  if(f_open(myFile, (const TCHAR*)diskPath, FA_READ) == FR_OK)
			  {

				  //Open file to correct location
				  fp = *fpointer;
				  printmsg("Initial GPS file pointer location %ld \r\n", *fpointer);
				  f_lseek(myFile, fp);

				  f_gets(dataBuf, maxLen, myFile);

				  uint8_t i = 0;

				  p = strtok(dataBuf, delim);

				  GPSArray[i] = atof(p);

				  while( (p != NULL) && (i < 11))
				  {
				 		p = strtok(NULL, delim);
				 		i++;
				 		GPSArray[i] = atof(p);
				  }

				  //store current file pointer location
				  *fpointer = f_tell(myFile);

			  }
			  else
			  {
				  printmsg("Open failed! \r\n");
				  return SD_OPEN_ERROR;
			  }


		  }
		  else
		  {
			   printmsg("Mount failed \r\n");
			   return SD_MOUNT_ERROR;
		  }

	  return SD_OK;

}

sd_status_t SD_ENV_Read(FIL *myFile, float *ENVArray,  uint32_t ENVDirNo, uint32_t ENVLogNo, uint32_t *fpointer)
{

	uint8_t dataBuf[60];
	int maxLen = 500;
	char diskPath[16]; /* User logical drive path */
	FSIZE_t fp = *fpointer;

	char *p; //pointer for strtok functions
	const char delim[] = ", ";
	char tempString[60];


	sprintf(diskPath, "ED/ES%d/E%d.txt", ENVDirNo, ENVLogNo);


	  if(SD_Mount(SDFatFs, diskPath) == SD_OK)
	     {

			 sprintf(diskPath, "ED/ES%d/E%d.txt", ENVDirNo, ENVLogNo);

			  if(f_open(myFile, (const TCHAR*)diskPath, FA_READ) == FR_OK)
			  {

				  //Open file to correct location
				  fp = 0;
				  printmsg("Initial ENV file pointer location %ld \r\n", *fpointer);
				  f_lseek(myFile, fp);

				  f_gets(dataBuf, maxLen, myFile);

				  uint8_t i = 0;

				  p = strtok(dataBuf, delim);

				  ENVArray[i] = atof(p);

				  while( (p != NULL) && (i < 3))
				  {
				 		p = strtok(NULL, delim);
				 		i++;
				 		ENVArray[i] = atof(p);
				  }

				  //store current file pointer location
				  *fpointer = f_tell(myFile);

			  }
			  else
			  {
				  printmsg("Open failed! \r\n");
				  return SD_OPEN_ERROR;
			  }


		  }
		  else
		  {
			   printmsg("Mount failed \r\n");
			   return SD_MOUNT_ERROR;
		  }

	  return SD_OK;

}

sd_status_t SD_PWR_Read(FIL *myFile, float *PWRArray,  uint32_t PWRDirNo, uint32_t PWRLogNo, uint32_t *fpointer)
{

	uint8_t dataBuf[60];
	int maxLen = 500;
	char diskPath[16]; /* User logical drive path */
	FSIZE_t fp = *fpointer;

	char *p; //pointer for strtok functions
	const char delim[] = ", ";
	char tempString[60];


	sprintf(diskPath, "PD/PS%d/P%d.txt", PWRDirNo, PWRLogNo);


	  if(SD_Mount(SDFatFs, diskPath) == SD_OK)
	     {

			 sprintf(diskPath, "PD/PS%d/P%d.txt", PWRDirNo, PWRLogNo);

			  if(f_open(myFile, (const TCHAR*)diskPath, FA_READ) == FR_OK)
			  {

				  //Open file to correct location
				  fp = *fpointer;
				  printmsg("Initial PWR file pointer location %ld \r\n", *fpointer);
				  f_lseek(myFile, fp);

				  f_gets(dataBuf, maxLen, myFile);

				  uint8_t i = 0;

				  p = strtok(dataBuf, delim);

				  PWRArray[i] = atof(p);

				  while(p != NULL)
				  {
				 		p = strtok(NULL, delim);
				 		i++;
				 		PWRArray[i] = atof(p);
				  }

				  //store current file pointer location
				  *fpointer = f_tell(myFile);

			  }
			  else
			  {
				  printmsg("Open failed! \r\n");
				  return SD_OPEN_ERROR;
			  }


		  }
		  else
		  {
			   printmsg("Mount failed \r\n");
			   return SD_MOUNT_ERROR;
		  }

	  return SD_OK;

}

