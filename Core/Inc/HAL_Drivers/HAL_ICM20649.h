/*
 * HAL_ICM20649.h
 *
 *  Created on: Mar 3, 2021
 *      Author: Michael Noyce
 *      Student No: NYCMIC002
 *      For: The University of Cape Town
 *
 *  This Library is designed to be used with the STM32 HAL Driver files version 1.15.1
 *  Version 1.14.x is also supported
 *
 *  This library is designed to interface with the ICM20649 Inertial Measurement Unit (IMU). This
 *  device is a 6 degree of Freedom MEMs-based IMU that communicates via I2C. The device measures
 *  3-axis acceleration, 3-axis rotation and contains an on-board temperature sensor. In addition,
 *  The chip contains a digital motion processor that fuses accelerometer and gyroscope readings to
 *  get roll,pitch,heave measurements.
 *
 *  This library contains all the functions and definitions to interface with and configure the sensor.
 *  This includes functions that
 *
 *  1. Set the ACC and GYRO Full Scale Resolution
 *  2. Set the Power Mode
 *  3. Set The Sampling Rate
 *  4. Configure the Digital Low Pass Filter
 *  5. Enable/Disable and Configure Interrupts
 *  6. Initialise a FIFO buffer.
 *  7. Read from and write to the on-board register
 *
 *
 */

#ifndef HAL_ICM20649_H_
#define HAL_ICM20649_H_

//============================= 1. Includes ==============================================

#include "stm32l4xx_hal.h"	//HAL library includes
#include "math.h"			//Math Functions
#include "stdint.h"			//integers
#include "string.h"			//Mem functions

//========================== 2. Structs & Enums ===========================================

/*
 * ICM_20689_Register_t
 *
 * Complete 8-bit Register Map of the ICM20649 chip.
 */
typedef enum
{
	//Register Bank Select
	REG_BANK_SEL = 0x7F,
	REG_0 = 0,
	REG_1 = 1,
	REG_2 = 2,
	REG_3 = 3,

	//User Bank 0 Registers (REG_0)
	WHO_AM_I = 0x0,
	USER_CTRL = 0x03,
	LP_CONFIG = 0x05,
	PWR_MGMT_1 = 0x06,
	PWR_MGMT_2 = 0x07,
	INT_PIN_CFG = 0x0F,
	INT_ENABLE = 0x10,
	INT_ENABLE_1 = 0x11,
	INT_ENABLE_2 = 0X12,
	INT_ENABLE_3 = 0X13,
	I2C_MST_STATUS = 0X17,
	INT_STATUS = 0X19,
	INT_STATUS_1 = 0X1A,
	INT_STATUS_2 = 0X1B,
	INT_STATUS_3 = 0X1C,
	ACCEL_XOUT_H = 0X2D,
	ACCEL_XOUT_L = 0X2E,
	ACCEL_YOUT_H = 0X2F,
	ACCEL_YOUT_L = 0X30,
	ACCEL_ZOUT_H = 0X31,
	ACCEL_ZOUT_L = 0X32,
	GYRO_XOUT_H = 0X33,
	GYRO_XOUT_L = 0X34,
	GYRO_YOUT_H = 0X35,
	GYRO_YOUT_L = 0X36,
	GYRO_ZOUT_H = 0X37,
	GYRO_ZOUT_L = 0X38,
	TEMP_OUT_H = 0X39,
	TEMP_OUT_L = 0X3A,
	FIFO_EN_1 = 0X66,
	FIFO_EN_2 = 0X67,
	FIFO_RST = 0X68,
	FIFO_MODE = 0X69,
	FIFO_COUNTH = 0X70,
	FIFO_COUNTL = 0X71,
	FIFO_R_W = 0X72,
	DATA_RDY_STATUS = 0X74,
	FIFO_CFG = 0X76,

	//User Bank 1 Registers (REG_1)
	SELF_TEST_X_GYRO = 0X02,
	SELF_TEST_Y_GYRO = 0x03,
	SELF_TEST_Z_GYRO = 0x04,
	SELF_TEST_X_ACCEL = 0X0E,
	SELF_TEST_Y_ACCEL = 0x0F,
	SELF_TEST_Z_ACCEL = 0x10,
	XA_OFFS_H = 0X14,
	XA_OFFS_L = 0X15,
	YA_OFFS_H = 0X17,
	YA_OFFS_L = 0X18,
	ZA_OFFS_H = 0X1A,
	ZA_OFFS_L = 0X1B,
	TIMEBASE_CORRECTION_PLL = 0X28,

	//User Bank 2 Registers (REG_2)
	GYRO_SMPLRT_DIV = 0X0,
	GYRO_CONFIG_1 = 0X01,
	GYRO_CONFIG_2 = 0X02,
	XG_OFFS_USRH = 0X03,
	XG_OFFS_USRL = 0X04,
	YG_OFFS_USRH = 0X05,
	YG_OFFS_USRL = 0X06,
	ZG_OFFS_USRH = 0X07,
	ZG_OFFS_USRL = 0X08,
	ODR_ALIGN_EN = 0X09,
	ACCEL_SMPLRT_DIV_1 = 0X10,
	ACCEL_SMPLRT_DIV_2 = 0X11,
	ACCEL_INTEL_CTRL = 0X12,
	ACCEL_WOM_THR = 0X13,
	ACCEL_CONFIG = 0X14,
	ACCEL_CONFIG_2 = 0X15,
	FSYNC_CONFIG = 0x52,
	TEMP_CONFIG = 0X53,
	MOD_CTRL_USR = 0X54,

}ICM_20689_Register_t;

/*
 * IMU_Status_t
 *
 * @brief:	Used to represent numeric statuses returned as a result of
 * 		    running Sensor-register level functions e.g. using i2c to write
 * 		    a value for Power mode. Successful return
 */
typedef enum
{
	IMU_I2C_ERROR,
	IMU_PERIPHERAL_INIT_ERROR,
	IMU_I2C_DEVICE_BUSY,
	IMU_I2C_ACK_NACK,
	IMU_CONFIG_ERROR,
	IMU_I2C_DEVICE_OFFLINE,
	IMU_I2C_DEVICE_ONLINE,
	IMU_I2C_ID_ERROR,
	IMU_CONFIG_OUT_OF_RANGE,
	IMU_INIT_CMD_ERROR,
	IMU_SELF_TEST_PASS,
	IMU_SELF_TEST_FAIL,
	IMU_STR_READ_ERROR,
	IMU_RESET_FAIL,
	IMU_RESET_SUCCESS,
	IMU_FIFO_READ_ERROR,
	IMU_CAL_SUCCESS,
	IMU_OK

}imu_status_t;

/*
 * IMU_PowerMode
 *
 * @brief: Represents the possible operating modes the sensor can be placed in
 */
typedef enum
{
	IMU_STANDBY,
	IMU_SLEEP,
	IMU_WAKE,
	IMU_LP,
	IMU_Cycle,
	IMU_RESET,
}IMU_PowerMode;

/*
 * Interrupt_source_t
 *
 * Representation of the interrupt sources available on the chip
 */
typedef enum
{
	FIFO_OVERFLOW,
	FIFO_WATERMARK,
	I2C_MST_INT,
	DATA_READY
}Interrupt_source_t;

/*
 * IMU_SelfTest_t
 *
 * Struct to store the values located in the self-test register
 */
typedef struct
{
	uint8_t A_x;
	uint8_t A_y;
	uint8_t A_z;
	uint8_t G_x;
	uint8_t G_y;
	uint8_t G_z;
}IMU_SelfTest_t;

/*
 * IMU_FT_t
 *
 * struct to store IMU data as a float representation
 */
typedef struct
{
	float A_x;
	float A_y;
	float A_z;
	float G_x;
	float G_y;
	float G_z;
}IMU_FT_t;

/*
 * Stores Raw IMU data as an unsigned 16-bit integer
 */
typedef struct
{
	uint16_t Accel[3];
	uint16_t Gyro[3];
	uint16_t Temp;
}IMU_data_t;


//======================== 3. Macro Definitions =========================================
//--------------------------------------------------------------------------------------
//User Bank 0 Registers
//--------------------------------------------------------------------------------------

//LP_CONFIG
#define ACCEL_CYCLE 0b1<<5
#define GYRO_CYCLE 0b1<<4



//PWR_MGMT_1
/*
	An internal 8MHz oscillator, gyroscope based clock, or external sources can be selected as the
	ICM20649 clock source. When the internal 8 MHz oscillator or an external source is chosen as the
	clock source, the IMU20689 can operate in low power modes with the gyroscopes disabled.
	Upon power up, the IMU20689 clock source defaults to the internal oscillator. However, it is highly
	recommended that the device be configured to use one of the gyroscopes (or an external clock
	source) as the clock reference for improved stability. The clock source can be selected according to
	the following table.
	CLKSEL Clock Source
	0 Internal 8MHz oscillator
	1 PLL with X axis gyroscope reference
	2 PLL with Y axis gyroscope reference
	3 PLL with Z axis gyroscope reference
	4 PLL with external 32.768kHz reference
	5 PLL with external 19.2MHz reference
	6 Reserved
	7 Stops the clock and keeps the timing generator in reset

 */

#define PWR_MGMT_1_CLK_SEL_MSK  0b111
#define PWR_MGMT_1_CLK_SEL_0 	~PWR_MGMT_1_CLK_SEL_MSK
#define PWR_MGMT_1_CLK_SEL_1 	1
#define PWR_MGMT_1_CLK_SEL_2 	2
#define PWR_MGMT_1_CLK_SEL_3 	3
#define PWR_MGMT_1_CLK_SEL_4 	4
#define PWR_MGMT_1_CLK_SEL_5 	5
#define PWR_MGMT_1_CLK_SEL_6 	6
#define PWR_MGMT_1_CLK_SEL_7 	PWR_MGMT_1_CLK_SEL_MSK

/*
	By setting SLEEP to 1, the ICM20649 can be put into low power sleep mode. When CYCLE is set to
	1 while SLEEP is disabled, the ICM20649 will be put into Cycle Mode. In Cycle Mode, the device
	cycles between sleep mode and waking up to take a single sample of data from accelerometer at a
	rate determined by LP_WAKE_CTRL (register 108). To configure the wake frequency, use
	LP_WAKE_CTRL within the Power Management 2 register (Register 108).
 */

#define PWR_MGMT_1_DEVICE_RESET 0b1<<7
#define PWR_MGMT_1_SLEEP_EN 	0b1<<6
#define PWR_MGMT_1_LP_EN 	0b1<<5
#define PWR_MGMT_1_TEMP_DIS 0b1<<3

//PWR_MGMT_2

/* Description:
 * The user can put individual accelerometer and gyroscopes axes into standby mode by using this
   register. If the device is using a gyroscope axis as the clock source and this axis is put into standby
   mode, the clock source will automatically be changed to the internal 8MHz oscillator.
*/


//INT_PIN_CFG

#define INT_PIN_CFG_LEVEL_LOW 0b1<<7 //active logic level for pin
#define INT_PIN_CFG_LEVEL_HIGH 0b0<<7
#define INT_PIN_CFG_PIN_OPEN_DRAIN 0b1<<6
#define INT_PIN_CFG_PIN_PUSH_PULL  0b0<<6
#define INT_PIN_CFG_LATCH_INT_EN 0b1<<5 //pin held high untill interrupt is cleared (if set to 0, pin emits a 50 us pulse)
#define INT_PIN_CFG_LATCH_INT_DIS 0b0<<5
#define INT_PIN_CFG_INT_RD_CLEAR 0b1<<4
#define INT_PIN_CFG_FSYNC_INT_LEVEL_LOW 0b1<<3
#define INT_PIN_CFG_FSYNC_INT_LEVEL_HIGH ~INT_PIN_FSYNC_INT_LEVEL_LOW
#define INT_PIN_CFG_FSYNC_INT_EN 0b1<<2


//INT_ENABLE
#define REG_WOM_EN 0b1<<7
#define WOM_INT_EN 0b1<<3
#define PLL_RDY_EN 0b1<<2


//INT_ENABLE_2
#define RAW_DATA_0_RDY_EN 0b1<<0

//INT_ENABLE_2
#define INT_ENABLE_FIFO_OFLOW_EN 0b11111

//INT_STATUS_1
#define RAW_DATA_0_RDY_INT 0b1

//INT_STATUS_2
#define FIFO_OVERFLOW_INT 0b11111

//INT_STATUS 3
#define FIFO_WM_INT 0b1

//SIGNAL_PATH_RESET
#define SIGNAL_PATH_ACC_RESET  0b1<<1
#define SIGNAL_PATH_TEMP_RESET 0b1

//USER_CTRL
#define USER_CTRL_FIFO_EN 0b1<<6
#define USER_CTRL_I2C_IF_DIS 0b1<<4//disabling interface allows for SPI mode
#define USER_CTRL_FIFO_RESET 0b1<<2
#define USER_CTRL_SIG_COND_RESET 0b1


//FIFO EN
//Data stored inside the sensor data registers (Registers 59 to 96) will be loaded into the FIFO buffer if
//a sensor�s respective FIFO_EN bit is set to 1 in this register.

//FIFO_EN_2 (FIFO_EN_1 is for an external sensor i2c)
#define ACCEL_FIFO_EN   0b1<<4
#define GYRO_Z_FIFO_EN	0b1<<3
#define GYRO_Y_FIFO_EN	0b1<<2
#define GYRO_X_FIFO_EN	0b1<<1
#define TEMP_FIFO_EN	0b1<<0

//FIFO_RST
#define FIFO_RESET 0b1<<4


//--------------------------------------------------------------------------------------
//User Bank 2 Registers
//--------------------------------------------------------------------------------------

//GYRO_CONFIG_1 Macros
#define GYRO_DPLFCFG_0 0<<3
#define GYRO_DPLFCFG_1 1<<3
#define GYRO_DPLFCFG_2 2<<3
#define GYRO_DPLFCFG_3 3<<3
#define GYRO_DPLFCFG_4 4<<3
#define GYRO_DPLFCFG_5 5<<3
#define GYRO_DPLFCFG_6 6<<3
#define GYRO_DPLFCFG_7 7<<3

#define GYRO_CONFIG_FSSEL_MSK		0b11
#define GYRO_CONFIG_FSSEL_500DPS	0<<1
#define GYRO_CONFIG_FSSEL_1000DPS	1<<1
#define GYRO_CONFIG_FSSEL_2000DPS	2<<1
#define GYRO_CONFIG_FSSEL_4000DPS	GYRO_CONFIG_FSSEL_MSK<<1

#define GYRO_FCHOICE_DIS 0 //DLPF disabled
#define GYRO_FCHOICE_EN 1 //DLPF Enabled

//GYRO_CONFIG_2 Macros
#define GYRO_CONFIG_ST_EN_X	0b1<<5
#define GYRO_CONFIG_ST_EN_Y	0b1<<4
#define GYRO_CONFIG_ST_EN_Z	0b1<<3
#define GYRO_CONFIG_ST_EN 0b111<<3
#define GYRO_AVGCFG_1x 0
#define GYRO_AVGCFG_2x 1
#define GYRO_AVGCFG_4x 2
#define GYRO_AVGCFG_8x 3
#define GYRO_AVGCFG_16x 4
#define GYRO_AVGCFG_32x 5
#define GYRO_AVGCFG_64x 6
#define GYRO_AVGCFG_128x 7



//ACC_CONFIG Macros
#define ACC_CONFIG_AFSSEL_MSK 0b11
#define ACC_CONFIG_AFSSEL_4G	0<<3
#define ACC_CONFIG_AFSSEL_8G	1<<3
#define ACC_CONFIG_AFSSEL_16G	2<<3
#define ACC_CONFIG_AFSSEL_30G	ACC_CONFIG_AFSSEL_MSK<<3

#define ACCEL_DPLFCFG_1 0<<3
#define ACCEL_DPLFCFG_2 1<<3
#define ACCEL_DPLFCFG_3 2<<3
#define ACCEL_DPLFCFG_4 3<<3
#define ACCEL_DPLFCFG_5 4<<3
#define ACCEL_DPLFCFG_6 5<<3

#define ACC_CONFIG_ST_EN_X	0b1<<7
#define ACC_CONFIG_ST_EN_Y	0b1<<6
#define ACC_CONFIG_ST_EN_Z	0b1<<5

#define ACC_FCHOICE_DIS 0 //DLPF disabled
#define ACC_FCHOICE_EN 1 //DLPF Enabled
//Sample Rate Value

#define ACC_OUTPUT_RATE_DLPF_EN 1125 //Hz
#define ACC_OUTPUT_RATE_DLPF_DIS 4500 //Hz


/* I2C Definitions:
 * S Start Condition: SDA goes from high to low while SCL is high
 * AD Slave I2C address
 * W Write bit (0)
 * R Read bit (1)
 * ACK Acknowledge: SDA line is low while the SCL line is high at the
 * 9th clock cycle
 * NACK Not-Acknowledge: SDA line stays high at the 9th clock cycle
 * RA ICM20649 internal register address
 * DATA Transmit or received data
 * P Stop condition: SDA going from low to high while SCL is high
 */
#define IMU_Device_Address 0xD0

#define I2C_SLAVE_ADDRESS_HIGH 0b1101001
#define I2C_WRITE_BIT 0b1
#define I2C_READ_BIT  0b0
#define BASE_REGISTER_RESET_VALUE 0x00
#define PWR_MGMT_RESET_VALUE 0x40
#define WHO_AM_I_VALUE  0xE1

//Sample Rate Value
#define SAMPLE_RATE 100//Hz
#define GYRO_OUTPUT_RATE_DLPF_EN 1125 //Hz
#define GYRO_OUTPUT_RATE_DLPF_DIS 4400 //Hz

//ACC ressolution values
#define ACC_2G_WORD_LENGTH 16384 //LSB/g
#define ACC_4G_WORD_LENGTH 8192
#define ACC_8G_WORD_LENGTH 4096
#define ACC_16G_WORD_LENGTH 2048
#define ACC_30G_WORD_LENGTH 1024

//Data Defines
#define N_SAMPLES 1
#define SAMPLE_SIZE 12 //12 bytes per sample ax ay az gx gy gz
#define IMU_BUFFER_SIZE N_SAMPLES*SAMPLE_SIZE
#define WAVELOGBUFNO 30000 //Number of local buffers written to each wavelog file 5min*60sec*100Hz
#define WAVELOGNO 5 //Number of wave logs


//I2C Peripheral Defines

#define IMU_I2C I2C1					//I2C Port chosen for the IMU
#define IMU_SCL_PIN GPIO_PIN_8			// SCL Pin
#define IMU_SDA_PIN GPIO_PIN_9			//SDA Pin
#define IMU_SCL_PORT GPIOB				//SCL Pin Port
#define IMU_SDA_PORT GPIOB				//SDA Pin Port
#define IMU_I2C_AF	GPIO_AF4_I2C1		//AF Mapping for I2C peripheral
#define IMU_INT_Pin GPIO_PIN_5			//Interrupt GPIO Pin
#define IMU_INT_GPIO_Port GPIOC			//Interrupt GPIO Port
#define IMU_INT_EXTI_IRQn EXTI9_5_IRQn	//IRQn for the NVIC

//========================== 4. Global Variables ==========================================

extern uint32_t imu_sample_count;   //Keeps track of the number of samples from the IMU

extern uint32_t fifo_sample_count;   //Keeps track of the number of samples in the FIFO Buffer

extern uint32_t fifo_sample_complete;   //Current FIFO sample is complete

extern uint8_t IMU_Log_On;			 //used in EXTI IRQ to determine what routine to run

extern uint32_t loopTime;
//============================= 5. Handlers ===============================================

extern I2C_HandleTypeDef hi2c1;

extern DMA_HandleTypeDef hdma_i2c1_rx;

extern DMA_HandleTypeDef hdma_i2c1_tx;


//============================ 6. Data Buffers ============================================

extern uint8_t IMU_Buffer;	//Buffer to store data from the IMU

extern uint8_t FIFO_Buffer;	//Buffer to store data from the IMU

//======================== 8. Sensor Configuration Functions =========================================

imu_status_t ICM20649_Set_Gyro_FSR(I2C_HandleTypeDef *hi2c, uint8_t FSR);
imu_status_t ICM20649_Set_Acc_FSR(I2C_HandleTypeDef *hi2c,uint8_t FSR);
imu_status_t ICM20649_Set_FSync(I2C_HandleTypeDef *hi2c, uint8_t Fsync);
imu_status_t ICM20649_Set_Acc_DLPF(I2C_HandleTypeDef *hi2c, uint8_t DLPF, uint8_t cmd);
imu_status_t ICM20649_Set_Gyro_DLPF(I2C_HandleTypeDef *hi2c, uint8_t DLPF, uint8_t cmd);
imu_status_t ICM20649_Set_Sample_Rate(I2C_HandleTypeDef *hi2c);
imu_status_t ICM20649_Set_PLLSrc(I2C_HandleTypeDef *hi2c, uint8_t PLL);
imu_status_t  ICM20649_Config_FIFO(I2C_HandleTypeDef *hi2c, uint8_t accEnable,  uint8_t gyroEnable, uint8_t tempEnable, uint8_t cmd );
imu_status_t ICM20649_Config_Interrupt_Pin(Interrupt_source_t interrupt, uint8_t level, uint8_t latch);
imu_status_t ICM20649_FIFO_CMD(I2C_HandleTypeDef *hi2c,uint8_t cmd);
imu_status_t ICM20649_Register_Bank_Select(I2C_HandleTypeDef *i2c, uint8_t regNo);
imu_status_t ICM20649_Disable_LPF(I2C_HandleTypeDef *hi2c, uint8_t accel_dis_lpf, uint8_t gyro_dis_lpf);

//======================= 9. Initializaiton Function Prototypes ========================================

/*Function Name imy_status_t ICM20649_Init_IMU(uint8_t g_fsr,uint8_t a_fsr, uint8_t dlpf_coeff)
 *
 * @brief: Initialise I2C and DMA Microcontroller Peripherals. Wake Up sensor and configure
 *
 */

imu_status_t ICM20649_Init_IMU(uint8_t g_fsr,uint8_t a_fsr, uint8_t dlpf_coeff, uint8_t dlpf_acc_coeff);
imu_status_t ICM20649_Deinit_IMU(void);
imu_status_t ICM20649_Enable_Interrupt(I2C_HandleTypeDef *hi2c, uint8_t interrupts, uint8_t intAddress);
imu_status_t ICM20649_Disable_Interrupts(I2C_HandleTypeDef *hi2c);

/* Function Name imu_status_t ICM20649_Init_FIFO(I2C_HandleTypeDef *hi2c, uint8_t enable)
 *
 * @brief: Enables the IMU FIFO buffer and configures it to recieve
 * 		   Data from peripherals determined by the fifo_Mask
 * @param: hi2c - pointer to I2C handle typedef
 * 		   enable - can either be (ENABLE - enables FIFO Buffer), DISABLE - disables BUFFER, RESET (3)- resets buffer
 */
imu_status_t ICM20649_Init_FIFO(I2C_HandleTypeDef *hi2c, uint8_t enable);

/* Function Name imu_status_t ICM20649_Init_TempSensor(I2C_HandleTypeDef *hi2c, uint8_t cmd);
 *
 * @brief: Function to enable/disable the temperature sensor
 *
 * @param: hi2c - pointer to I2C handle
 * 		   cmd 	- set to ENABLE to enable the reading, DISABLE to disable
 */
imu_status_t ICM20649_Init_TempSensor(I2C_HandleTypeDef *hi2c, uint8_t cmd);

//======================= 10. Sensor Read Functions ====================================================
imu_status_t ICM20649_Is_Data_Ready(I2C_HandleTypeDef *hi2c, uint8_t *status);
imu_status_t ICM20649_Get_SelfTestResponse_Values(I2C_HandleTypeDef *hi2c,IMU_SelfTest_t *IMU);
imu_status_t ICM20649_Get_ID(I2C_HandleTypeDef *hi2c,uint8_t* ID);
imu_status_t ICM20649_Get_MST_Status(I2C_HandleTypeDef *hi2c, uint8_t* status_byte);
imu_status_t ICM20649_Get_FIFO_Count(I2C_HandleTypeDef *hi2c, uint16_t* count);
imu_status_t ICM20649_SelfTest(I2C_HandleTypeDef *hi2c,float* test_res);
imu_status_t ICM20649_Get_IMU_RawData(I2C_HandleTypeDef *hi2c, uint8_t *imu);
imu_status_t ICM20649_Get_IMU_RawData_FIFO(I2C_HandleTypeDef *hi2c, uint8_t* imuBuf, uint8_t count);
imu_status_t Reset_FIFO(I2C_HandleTypeDef *hi2c);
//======================= 11. Power Mode Config Function ===============================================

imu_status_t ICM20649_Set_Wake(I2C_HandleTypeDef *hi2c);
imu_status_t ICM20649_Set_Cycle_Power_Mode(I2C_HandleTypeDef *hi2c,uint8_t Cycles);
imu_status_t ICM20649_Set_Low_Power_Mode_Acc(I2C_HandleTypeDef *hi2c,uint8_t Cycles);
imu_status_t ICM20649_Set_Sleep_Power_Mode(I2C_HandleTypeDef *hi2c);
imu_status_t ICM20649_Get_Interrupt_Status(I2C_HandleTypeDef *hi2c, Interrupt_source_t interrupt_src,uint8_t* res);
imu_status_t ICM20649_Signal_conditioned_Reset(I2C_HandleTypeDef *hi2c);
imu_status_t ICM20649_reset(I2C_HandleTypeDef *hi2c);

void ICM20649_DMA_PeriphIRQHandler(void);
void ICM20649_FIFO_Interrupt_IRQ(void);
void ICM20649_IntPin_IRQ();
#endif /* HAL_ICM20649_H_ */
