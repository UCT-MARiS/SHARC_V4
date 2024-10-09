/*
 * HAL_SD.h
 *
 *  Created on: Apr 20, 2022
 *      Author: Michael Noyce
 *      Student No: NYCMIC002
 *      For: The University of Cape Town
 *
 *  This Library is designed to be used with the STM32 HAL Driver files version 1.15.1
 *  Version 1.14.x is also supported
 *
 *  This library is designed to interface with SD card external memory using the
 *  onboard SDMMC driver on the STM32
 *
 *  This library contains all the functions and definitions to interface with and configure the sensor.
 *  This includes functions that
 *
 *  1. Configure the SD card and FATFS
 *  2. Open files
 *  3. Read files
 *  4. Create Files
 *
 * NB: get_fattime must be updated in the fatfs.c file to use f_utime and timestamp functions
 */

#ifndef HAL_SD_H_
#define HAL_SD_H_

#ifdef __cplusplus
extern "C" {
#endif


//============================= 1. Includes ==============================================

#include "stm32l4xx_hal.h"	//HAL library includes
#include "math.h"			//Math Functions
#include "stdint.h"			//integers
#include "string.h"			//Mem functions
#include "fatfs.h"
#include "stdio.h"
//========================== 2. Structs & Enums ===========================================

/*
 * sd_status_t
 *
 * @brief:	Used to represent numeric statuses returned as a result of
 * 		    running SDMMC interface
 */
typedef enum
{
	SD_SDMMC_ERROR,
	SD_INIT_ERROR,
	SD_MOUNT_ERROR,
	SD_CLOSE_ERROR,
	SD_OPEN_ERROR,
	SD_WRITE_ERROR,
	SD_CRC_ERROR,
	SD_DMA_ERROR,
	SD_OK,
	SD_TIME_ERROR,
}sd_status_t;



typedef enum
{
	TIMESTAMP_DATA,
	X_ACC,
	Y_ACC,
	Z_ACC,
	X_GYRO,
	Y_GYRO,
	Z_GYRO,
	TEMP,
}IMU_Data_SD_t;


//========================== 4. Global Variables ==========================================

extern FATFS SDFatFs; /* File system object for User logical drive */
extern FIL File; /* File object */
extern DIR Dir; /*Directory Object*/
extern FILINFO fno;
extern uint32_t waveLogNo;
extern uint32_t waveDirNo;
extern uint32_t gpsLogNo;
extern uint32_t gpsDirNo;
extern uint32_t envLogNo;
extern uint32_t envDirNo;
extern uint32_t pwrLogNo;
extern uint32_t pwrDirNo;

//============================= 5. Defines ===============================================

#define SD_Detect_Pin GPIO_PIN_5
#define SD_Detect_GPIO_Port GPIOF

//============================= 6. Handlers ===============================================

//extern UART_HandleTypeDef hlpuart1;

extern SD_HandleTypeDef hsd1;

//RTC_HandleTypeDef hrtc; // this is only included for the standalone module - remove if using code with own RTC


//======================== 7. SD Configuration Functions =========================================


/*Function Name sd_status_t SD_Init(void);
 *
 * @brief: Initialise SD Interface
 *
 */

sd_status_t SD_Init(void);

/*Function Name sd_status_t SD_Mount(uint8_t mount);
 *
 * @brief: Mount SD Card
 *
 */
sd_status_t SD_Mount(FATFS SDFatFs, char myPath[]);

/*Function Name sd_status_t SD_File_Open(FIL myfile);
 *
 * @brief: Open file on SD Card
 *
 */

/**
 * @brief
 *
 * @param SDFatFs
 * @param diskPath
 * @return
 */
sd_status_t SD_Unmount(FATFS SDFatFs);

//======================== 6. SD Read/Write Functions ============================================

sd_status_t SD_File_Open(FIL *myFile, uint32_t fileNo);

/*Function Name sd_status_t SD_File_Write(FIL myfile, uint8_t dataBuffer[]);
 *
 * @brief: Write to SD Card
 *
 */
sd_status_t SD_File_Write(FIL *myFile, uint8_t *dataBuffer);

/*Function Name sd_status_t SD_File_Close(FIL myfile)
 *
 * @brief: Close File on SD Card
 *
 */
sd_status_t SD_File_Close(FIL *myFile);

/**
 * @brief Closes current file directory
 *
 * @param myFile
 * @return
 */
sd_status_t SD_Dir_Close(DIR *myDir);


/**
 * @brief Time-stamps myFile with current time from RTC
 *
 * @param myFile
 * @return
 */
sd_status_t SD_File_TimeStamp(FIL *myFile);



/**
 * @brief Opens file in wave file directory
 *
 * Description: The function takes a individual IMU sample segment from local memory and appends it to a numbered wave log file.
 * After a determined number of buffers have been added, close file and start new file.
 *
 * @param myFile
 * @param waveLogNo
 * @param waveLogNo
 * @return
 */
sd_status_t SD_Wave_Open(FIL *myFile, DIR *myDir, FILINFO* fno, uint32_t wavDirNo, uint32_t waveLogNo);


/**
 * @brief
 *
 * @param myFile
 * @param LineNo
 * @param dataBuf
 * @return
 */
sd_status_t SD_Wave_Read(FIL *myFile, int32_t *IMUArray,  uint32_t WaveDirNo, uint32_t WaveLogNo,  IMU_Data_SD_t inertialDataType, uint32_t *fpointer);




/**
 * @brief Opens file in GPS file directory
 *
 * @param myFile
 * @param myDir
 * @param fno
 * @param
 * @return
 */
sd_status_t SD_GPS_Open(FIL *myFile, DIR *myDir, FILINFO* fno, uint32_t GPSDirNo, uint8_t GPSLogNo);


/**
 * @brief
 *
 * @param myFile
 * @param GPSArray
 * @param GPSDirNo
 * @param GPSLogNo
 * @param GPSDataType
 * @param fpointer
 * @return
 */
sd_status_t SD_GPS_Read(FIL *myFile, float *GPSArray,  uint32_t GPSDirNo, uint32_t GPSLogNo, uint32_t *fpointer);


/**
 * @brief
 *
 * @param myFile
 * @param ENVArray
 * @param ENVDirNo
 * @param ENVLogNo
 * @param fpointer
 * @return
 */
sd_status_t SD_ENV_Read(FIL *myFile, float *ENVArray,  uint32_t ENVDirNo, uint32_t ENVLogNo, uint32_t *fpointer);


/**
 * @brief
 *
 * @param myFile
 * @param PWRArray
 * @param PWRDirNo
 * @param PWRLogNo
 * @param fpointer
 * @return
 */
sd_status_t SD_PWR_Read(FIL *myFile, float *PWRArray,  uint32_t PWRDirNo, uint32_t PWRLogNo, uint32_t *fpointer);


/**
 * @brief Opens file in environmental directory
 *
 * @param myFile
 * @param myDir
 * @param fno
 * @param ENVDirNo
 * @param ENVLogNo
 * @return
 */
sd_status_t SD_ENV_Open(FIL *myFile, DIR *myDir, FILINFO* fno, uint32_t ENVDirNo, uint8_t ENVLogNo);


/**
 * @brief Opens file in power directory
 *
 * @param myFile
 * @param myDir
 * @param fno
 * @param PWRDirNo
 * @param PWRLogNo
 * @return
 */
sd_status_t SD_PWR_Open(FIL *myFile, DIR *myDir, FILINFO* fno, uint32_t PWRDirNo, uint8_t PWRLogNo);

#ifdef __cplusplus
}
#endif

#endif /* HAL_SD_H_ */
