#include "main.hpp"
#include "hal_interface.hpp"
#include "hal_impl.hpp"
#include "u-blox_m9n_Library.h"

#pragma GCC diagnostic ignored "-Wwrite-strings"
#pragma GCC diagnostic ignored "-Wformat"

#ifdef __cplusplus 
extern "C" {
#endif
#include "init.h"
#include "string.h"
#include <stdarg.h>
#include "stdio.h"
#include "arm_math.h"

//FreeRTOS includes
#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <timers.h>
#include <semphr.h>
#include <event_groups.h>
#ifdef __cplusplus 
}
#endif

//======================== 0. Peripheral Handles ========================================================================================
UART_HandleTypeDef hlpuart1;
UART_HandleTypeDef huart2;
HAL_Impl halImpl;

SFE_UBLOX_GNSS myGNSS;
//======================== 0. END ============================================================================================================= 

//======================== 1. Function Prototypes ========================================================================================

static void LED_task(void *args); 
static void GPS_task(void *args);

//======================== 1. END =======================================================================================================

int main(void) {
    
//======================== 1. SYSTEM INIT & CLOCK CONFIG ======================================================================================
    HAL_Init();
    SystemClock_Config();
    setupHAL(&halImpl);

    halImpl.MX_USART2_UART_Init();

    // Initialize the circular buffer
    myGNSS.circular_buffer_init(&myGNSS.RX_Buffer);

    // Initialize the UART interrupt
    initUARTInterrupt(&huart2);

	//printmsg("SHARC BUOY STARTING! \r\n");
//=================================== 1. END ============================================================================================

//=================================== 2. RTOS Tasks ======================================================================================

    // Create a blinking LED task for the on-board LED.
    static StaticTask_t exampleTaskTCB;
    static StackType_t exampleTaskStack[ 512 ];

    // Create the blink task
    const TickType_t xDelay = 500 / portTICK_PERIOD_MS;  // 500 ms delay

                       
    TaskHandle_t returnStatus_LED = xTaskCreateStatic( LED_task,
                                  "Blink_LED",
                                  configMINIMAL_STACK_SIZE,
                                  (void*)xDelay,
                                  1U, //configMAxPr
                                  &( exampleTaskStack[ 0 ] ),
                                  &( exampleTaskTCB ) );
                                  
    printmsg("returnStatus_LED: %d\r\n", returnStatus_LED);

    
    
/*     // Create GPS Task                       
    BaseType_t returnStatus_GPS = xTaskCreate(GPS_task, "GPS_Task", uint16_t(256), NULL, 2U, NULL);

    if (returnStatus_GPS != pdPASS) {
        printmsg("Failed to create GPS Task\r\n");
        // Handle error
    } */

//=================================== 2. END =========================================================================================
/* 
// ========================= 3. Wave Parameters ======================================================================================

    // Input signal
    float32_t inputSignal[SIGNAL_LENGTH];
    uint32_t f0 = 2;

    for (uint32_t i = 0; i < SIGNAL_LENGTH; i++) {
        inputSignal[i] = arm_sin_f32(2 * PI * i * f0/ (F_SAMPLE));
//        printmsg("%d,%f\r\n", i,inputSignal[i]);
//        delay(1000); delay(1000);
    }
    
    float32_t PSD[FFT_LENGTH/2];
    arm_fill_f32(0.0f, PSD, FFT_LENGTH/2);

    // Perform Welch Method
    WelchMethod(PSD, inputSignal);
    
    for (uint32_t i = 0; i < FFT_LENGTH/2; i++) {
        printmsg("%d,%f\r\n", i,PSD[i]);
        delay(1000);
        delay(1000); 
    }  

    Wave_Data_t waveData;
 	waveParamExtract(PSD, &waveData.Hm0, &waveData.TM01, &waveData.Hrms, &waveData.T0, &waveData.Tp, &waveData.M0, &waveData.M1, &waveData.M2);

	printmsg("Hm0 %.8f \r\n", waveData.Hm0);
	printmsg("Hrms %.8f \r\n", waveData.Hrms);
	printmsg("M0 %.8f \r\n", waveData.M0);
	printmsg("M1 %.8f \r\n", waveData.M1);
	printmsg("M2 %.8f \r\n", waveData.M2);
	printmsg("Tm01 %.8f \r\n", waveData.TM01);
	printmsg("T0 %.8f \r\n", waveData.T0);
	printmsg("Tp %.8f \r\n", waveData.Tp);

// ========================= 3. END ===============================================================================================
 */
// ========================= 4. Connect Receiver and Start Scheduler ==============================================================


    //myGNSS.enableDebugging(&hlpuart1); // Enable debug messages over LPUART1
    if (myGNSS.begin(&huart2) == true)
    {
        long firstEpoch = myGNSS.getUnixEpoch();
        printmsg("Unix Epoch: %d\r\n", firstEpoch);
        printmsg("GNSS serial connected \r\n");
        myGNSS.setUART1Output(COM_TYPE_UBX); //Set the UART port to output UBX only
        myGNSS.saveConfiguration(); //Save the current settings to flash and BBR;


        int dataIndex = 0;
        int32_t nedDownVel_mm = myGNSS.getNedDownVel(); //Returns the NED Down in mm
        float32_t nedDownVel_m = nedDownVel_mm/1000;
        printmsg("%d,%f\r\n",dataIndex, nedDownVel_m);
        dataIndex++;

        int32_t longitude = myGNSS.getLongitude(); 
        printmsg("%d,%d\r\n",dataIndex, longitude);
        dataIndex++;

        // Start scheduler 
        vTaskStartScheduler();

        printmsg("ERROR");
    }
    else
        printmsg("Unable to connect \r\n"); 

// ========================= 4. END ===================================================================================================

// Should not get to this point
    while(1){
        halImpl.HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_7);
        halImpl.HAL_Delay(1000);
    }
}

//======================== 5. Functions =============================================================================================

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    //static uint32_t old_pos = 0; // Define and initialize old_pos to 0
    // Check if the interrupt is for the correct UART instance
    if (huart->Instance == myGNSS._serialPort->Instance)
    {
        // Read the received byte and store it in the circular buffer
        myGNSS.circular_buffer_write(&myGNSS.RX_Buffer, myGNSS.receivedBytes, myGNSS.numberOfBytes);
        // Set the flag to indicate data has been received
        myGNSS.dataReceived = true;

        // Re-enable the UART receive interrupt
        HAL_UART_Receive_IT(huart, (uint8_t*)&myGNSS.receivedBytes, sizeof(myGNSS.receivedBytes));
    }
}

void initUARTInterrupt(UART_HandleTypeDef *huart)
{
    // Start the UART receive interrupt
    HAL_UART_Receive_IT(huart, (uint8_t*)&myGNSS.receivedBytes, sizeof(myGNSS.receivedBytes));
}

void performFFT(float32_t* segment, float32_t* fftOutput, uint32_t fftSize, uint32_t ifftFlag){
    // FFT instance
    arm_rfft_fast_instance_f32 fftInstance;
    arm_rfft_fast_init_f32(&fftInstance, fftSize);

    // Perform FFT
    arm_rfft_fast_f32(&fftInstance, segment, fftOutput, ifftFlag);

/*     for (uint32_t i = 0; i < FFT_LENGTH; i++) {
        printmsg("%d,%f\r\n", i,fftOutput[i]);
        delay(1000);
        delay(1000); 
    }   */
}

void WelchMethod(float32_t* PSD, float32_t* timeSeries)
{
	uint32_t i;
	uint32_t j;
	float32_t singleSegment[FFT_LENGTH];
	float32_t singleSegmentFFT[FFT_LENGTH];
	float32_t singleSegmentMag[FFT_LENGTH/2];
	float32_t tempSum;
	float32_t waveAmplitudeArray[SEGMENT_NO][FFT_LENGTH/2];
	float32_t normalConst =  (float32_t)(FFT_LENGTH*F_SAMPLE/((1.633*1.633)));//(float32_t)(FFT_LENGTH * F_SAMPLE); //normalizes periodogram estimate
	//Populate wave amplitude array with windowed FFT functions

	uint32_t waveLogNo = 1;

	for(i = 0; i<SEGMENT_NO; i++)
	{
		//populate singleSegment array
		singleSegmentPipeline(singleSegment, timeSeries, i);
		waveLogNo++;

/*         for(uint32_t k = 0; k < FFT_LENGTH; k++) {
            printmsg("%d,%f\r\n", k,singleSegment[k]);
            delay(1000);
            delay(1000); 
        }  */

		//FFT each segment
		performFFT(singleSegment, singleSegmentFFT, FFT_LENGTH, FFT);

		arm_cmplx_mag_f32(singleSegmentFFT, singleSegmentMag, FFT_LENGTH/2);

		 //Remove frequencies above 2 Hz 
		int index_1Hz = (20 * FFT_LENGTH) / ( F_SAMPLE / DECIMATION_CONSTANT);
		// Set all frequency bins beyond index_1Hz to zero
		for (int i = index_1Hz + 1; i < FFT_LENGTH / 2; i++) {
			singleSegmentMag[i] = 0.0f;  // Assuming outputF32 is your RFFT result array
		}
		int index_0_03Hz = (0.03 * FFT_LENGTH) / ( F_SAMPLE / DECIMATION_CONSTANT);
		// Set all frequency bins bleow index_0_03Hz to zero
		for (int i = 0; i < index_0_03Hz; i++) {
			singleSegmentMag[i] = 0.0f;  // Assuming outputF32 is your RFFT result array
		} 

        //Integrate to get displacement
        frequencyDomainIntegration(singleSegmentMag);

		//Fill waveAmplitudeArray with segments 
		for(j = 0; j<FFT_LENGTH/2; j++)
		{
			waveAmplitudeArray[i][j] = singleSegmentMag[j]; // At this point, waveAmplitudeArray contains velocity values
		}


/*          for (uint32_t i = 0; i < FFT_LENGTH/2-1; i++) {
            printmsg("%d,%f\r\n", i,singleSegmentMag[i]);
            delay(1000);
            delay(1000); 
        }   */
	}

	//Periodogram

	for (i = 0; i < SEGMENT_NO ; i++)
	{
		for (j = 0; j < FFT_LENGTH/2; j++)
		{
			waveAmplitudeArray[i][j] = (waveAmplitudeArray[i][j]*waveAmplitudeArray[i][j])/(normalConst);
		}

	}

	// Averaging
	for (i = 0; i < FFT_LENGTH/2 ; i++)
	{
		tempSum = 0;
		for (j = 0; j < SEGMENT_NO ; j++)
		{
			tempSum += waveAmplitudeArray [j][i];
		}
		PSD [i] = tempSum / SEGMENT_NO * FFT_LENGTH ;
	}

	PSD [0] /= 2;

}

void singleSegmentPipeline(float32_t* singleSegment, float32_t* timeSeries, uint32_t segmentIndex)
{
    // Apply Hanning window
    float32_t hanningWindow[FFT_LENGTH];
    arm_hanning_f32(hanningWindow, FFT_LENGTH);

    arm_mult_f32(&timeSeries[segmentIndex*FFT_LENGTH], hanningWindow, singleSegment, FFT_LENGTH);

/*     for (uint32_t i = 0; i < FFT_LENGTH; i++) {
         printmsg("%d,%f\r\n", i,singleSegment[i]);
        delay(1000);
        delay(1000); 
    } */
}


//Wave Parameter extraction
//Based on SWASH Model
void waveParamExtract(float32_t* PSD, float32_t* Hm0, float32_t* Tm01, float32_t* Hrms, float32_t* T0, float32_t* Tp, float32_t*  M0, float32_t* M1, float32_t* M2)
{

	//Iterators
	uint32_t i, j;
	//Length of PSD input
	float32_t sample_length = FFT_LENGTH/2;
	//Subint Length
	float32_t subInt_length;
	// Number of subintervals
	uint32_t subInt_no = sample_length - 1;
	//Integral
	float32_t Sum = 0;
	//Frequency array
	float32_t freq[FFT_LENGTH/2];
	//Max Amplitude freq
	float32_t fm;
	//Freq index of max amplitude
	uint32_t fi;
	//temp moment values
	float32_t m0, m1, m2;

	//Create frequency array
	for(i=0; i<FFT_LENGTH/2; i++)
	{
		freq[i] = ((float32_t)F_SAMPLE/(float32_t)DECIMATION_CONSTANT/2)*((float32_t)i/(FFT_LENGTH));
	}



	//Length of subinterval
	subInt_length=(freq[subInt_no+1]-freq[1])/subInt_no;

	//Newton Cotes Quadrature to calculate spectral moments
	//Note scaling by 10000 used to preserve resolution

	for(i=2; i<FFT_LENGTH/2; i++)
	{
		Sum = Sum + PSD[i];
	}

	m0 = 10000*0.5*subInt_length*(PSD[1]+2*Sum+PSD[subInt_no+1]);

	*M0 = m0/10000;

	printmsg("M0 calculated! \r\n");

	Sum = 0;

	for(i=2; i<FFT_LENGTH/2; i++)
	{
		Sum = Sum + (freq[i])*PSD[i];
	}

	printmsg("f^2 sum %.8f \r\n", Sum);

	m1 = 10000*0.5*subInt_length*(freq[1]*PSD[1]+2*Sum+(freq[subInt_no+1]*PSD[subInt_no+1]));

	*M1 = m1/10000;

	printmsg("M1 calculated!: m1 %.8f \r\n", m1);

	Sum = 0;

	for(i=2; i<FFT_LENGTH/2; i++)
		{
			Sum = Sum + (freq[i]*freq[i])*PSD[i];
		}

	 m2 = 10000*0.5*subInt_length*(freq[1]*freq[1]*PSD[1]+2*Sum+(freq[subInt_no+1]*freq[subInt_no+1]*PSD[subInt_no+1]));

	 *M2 = m2/10000;

	printmsg("M2 calculated! \r\n");

	//add M2, T0, Hrms and H0 calculations

	*Hm0 = 4*sqrt(fabsf(m0/10000)); //correct
	*Tm01 = (fabsf(m0/m1)); //correct
	*Hrms = sqrt(2)/2*(*Hm0); // Correct
	*T0 = sqrt(fabsf(m2)/fabsf(m0)); //Correct

	printmsg("Parameters calculated! \r\n");

	 for (i = 1; i < FFT_LENGTH/2; ++i) {
	    if (PSD[0] < PSD[i]) {
	    	fi = i;
	    }
	  }

	fm = freq[fi];

	*Tp = 1/fm;

	printmsg("Tp calculated! \r\n");

}

// Function to integrate waveAmplitudeArray in the frequency domain
void frequencyDomainIntegration(float32_t* velocityFFT) {

    // Loop through each frequency bin
    for (uint32_t i = 1; i < FFT_LENGTH / 2; i++) { // Start from 1 to avoid DC component
        float32_t frequency = (float)i * F_SAMPLE / (FFT_LENGTH);
        float32_t omega = 2.0f * PI * frequency;
        float32_t realPart = velocityFFT[i];

        // Integrate by dividing by jOmega
        velocityFFT[i] = realPart / omega;
        }

        // Handle DC component separately
        velocityFFT[0] = 0.0f;
    }

// Function to implement a busy-wait delay
void delay(volatile uint32_t count) {
    while (count) {
        count--;
        // Do nothing, just decrement the count
    }
}
//Debug Print
void printmsg(char *format,...) {
    char str[1024];

    /*Extract the the argument list using VA apis */
    va_list args;
    va_start(args, format);
    vsprintf(str, format,args);
    halImpl.HAL_UART_Transmit(&hlpuart1,(uint8_t *)str, strlen(str),HAL_MAX_DELAY);
    va_end(args);
}

#define DELAY_800_MS (800 / portTICK_PERIOD_MS)

uint32_t microseconds;

static void GPS_task(void *pvParameters){
    uint32_t dataIndex = 0;
    printmsg("Data Index: %d\r\n", dataIndex);

    TickType_t xLastWakeTime;
    
    /*
    * The xLastWakeTime variable needs to be initialized with the current tick
    * count. Note that this is the only time the variable is explicitly
    * written to. After this xLastWakeTime is managed automatically by the
    * vTaskDelayUntil() API function.
    */
    xLastWakeTime = xTaskGetTickCount();

    for (;;){
        vTaskDelay(DELAY_800_MS);
 
/*         long latitude = myGNSS.getLatitude();   //Returns the latitude in degrees x10^-7 as a long integer
        printmsg("Latitude: %d\r\n", latitude);

        long longitude = myGNSS.getLongitude(); //Returns the longitude in degrees x10^-7 as a long integer
        printmsg("Longitude: %d\r\n", longitude); */

        printmsg("Data Index: %d\r\n", dataIndex);

        float32_t nedDownVel = myGNSS.getNedDownVel(); //Returns the NED Down in mm
        nedDownVel = nedDownVel/1000;
        printmsg("%d,%f\r\n",dataIndex, nedDownVel);
        dataIndex++;

        if (dataIndex == 128){
            vTaskEndScheduler();
        }


/*        long PDOP = myGNSS.getPDOP(); //Positional Dilution of Precision
        printmsg("PDOP: %d\r\n", PDOP);

        long Alitude_MSL = myGNSS.getAltitudeMSL(); //Returns the altitude above Mean Sea Level in mm
        printmsg("Altitude MSL: %d\r\n", Alitude_MSL);

        long speed = myGNSS.getGroundSpeed(); //Returns the ground speed in mm/s
        printmsg("Speed: %d\r\n", speed);

        long heading = myGNSS.getHeading(); //Returns the heading in degrees
        printmsg("Heading: %d\r\n", heading);

        long year = myGNSS.getYear(); //Returns the year (UTC)
        printmsg("Year: %d\r\n", year); */

    }
}

static void LED_task(void *pvParameters) {

    TickType_t pxDelay = (TickType_t)pvParameters;

    TickType_t xLastWakeTime;

    const TickType_t xDelay3ms = pdMS_TO_TICKS( 3 );
    
    /*
    * The xLastWakeTime variable needs to be initialized with the current tick
    * count. Note that this is the only time the variable is explicitly
    * written to. After this xLastWakeTime is managed automatically by the
    * vTaskDelayUntil() API function.
    */
    xLastWakeTime = xTaskGetTickCount();
    /* As per most tasks, this task is implemented in an infinite loop. */

    for(;;) {
        halImpl.HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_7);
        vTaskDelay(pxDelay);

        printmsg("LED BLINKING\r\n");
    }
}

/**
 * @brief Default mode is to put the Cortex-M4 in sleep mode when the RTOS is idle.
 * 
 */
void vApplicationIdleHook(void) {
        // Put the Cortex-M4 into sleep mode
        printmsg("IDLE HOOK\r\n");
        __WFI();
    }

#if ( configCHECK_FOR_STACK_OVERFLOW > 0 )

    void vApplicationStackOverflowHook( TaskHandle_t xTask,
                                        char * pcTaskName )
    {
        /* Check pcTaskName for the name of the offending task,
         * or pxCurrentTCB if pcTaskName has itself been corrupted. */
        ( void ) xTask;
        ( void ) pcTaskName;
    }

#endif /* #if ( configCHECK_FOR_STACK_OVERFLOW > 0 ) */



