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

//======================== 0. Peripheral Handles ============================================================
UART_HandleTypeDef hlpuart1;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;
HAL_Impl halImpl;

SFE_UBLOX_GNSS myGNSS;

typedef struct {
    uint32_t epochTime;
    uint32_t downVelocity;
} GNSS_Data_t;

GNSS_Data_t gpsDataArray[MAX_DATA_PAIRS];
int dataIndex = 0;

//======================== 0. END ============================================================================

//======================== 1. Function Prototypes ============================================================

// Function pointers for RTOS Task functions
static void LED_task(void *args); 
static void GPS_task(void *args);

//======================== 1. END ============================================================================

int main(void) {
    
//======================== 1. SYSTEM INIT & CLOCK CONFIG ========================//
    HAL_Init();
    SystemClock_Config();
    setupHAL(&halImpl);

    halImpl.MX_USART2_UART_Init();
    MX_DMA_Init(); // To Do: Add to halImpl

    // Initialize the circular buffer
    myGNSS.circular_buffer_init(&myGNSS.RX_Buffer);

    // Initialize the UART interrupt
    initUARTInterrupt(&huart2);

    // Start the DMA transfer for UART reception
    //initUARTDMA(&huart2);


	//printmsg("SHARC BUOY STARTING! \r\n");
//=================================== 1. END ====================================//

    // Create a blinking LED task for the on-board LED.
    static StaticTask_t exampleTaskTCB;
    static StackType_t exampleTaskStack[ 512 ];

    // Create the blink task
    const TickType_t xDelay = 500 / portTICK_PERIOD_MS;  // 500 ms delay

                       
    TaskHandle_t returnStatus_LED = xTaskCreateStatic( LED_task,
                                  "Blink_LED",
                                  configMINIMAL_STACK_SIZE,
                                  (void*)xDelay,
                                  1, //configMAxPr
                                  &( exampleTaskStack[ 0 ] ),
                                  &( exampleTaskTCB ) );

    
    
    // Create GPS Task                       
    BaseType_t returnStatus_GPS = xTaskCreate(GPS_task, "GPS_Task", configMINIMAL_STACK_SIZE, NULL, 2, NULL);

    if (returnStatus_GPS != pdPASS) {
        printmsg("Failed to create GPS Task\r\n");
        // Handle error
    }

    //myGNSS.enableDebugging(&hlpuart1); // Enable debug messages over LPUART1
 
         float32_t params[MAX_PARAMS][3] = {
        {1.0f, 1.0f / 6.0f, 0.0f}
    };
    int num_params = 1;

    float32_t time[MAX_LENGTH];
    float32_t vertical_velocity[MAX_LENGTH];

    generate_wave(params, num_params, DURATION, SAMPLE_RATE, time, vertical_velocity);

    float32_t frequencies[NPERSEG];
    float32_t psd[NPERSEG];

    compute_welch(vertical_velocity, MAX_LENGTH, SAMPLE_RATE, frequencies, psd);

    float32_t displacement_spectrum[NPERSEG];
    integrate_displacement(psd, frequencies, displacement_spectrum);

    apply_bandpass_filter(displacement_spectrum, frequencies, 0.03f, 0.4f);

    float32_t significant_wave_height;
    float32_t mean_wave_period;
    float32_t average_zero_crossing_period;

    calculate_wave_parameters(displacement_spectrum, frequencies, significant_wave_height, mean_wave_period, average_zero_crossing_period);
 
    printmsg("Significant Wave Height (Hs): %d\r\n meters", significant_wave_height);
    printmsg("Mean Wave Period (T_bar): %d\r\n seconds", mean_wave_period);
    printmsg("Average up-crossing period between waves (T2): %d\r\n seconds", average_zero_crossing_period);

    if (myGNSS.begin(&huart2) == true)
    {
        long firstEpoch = myGNSS.getUnixEpoch();
        printmsg("Unix Epoch: %d\r\n", firstEpoch);
        printmsg("GNSS serial connected \r\n");
        myGNSS.setUART1Output(COM_TYPE_UBX); //Set the UART port to output UBX only
        myGNSS.saveConfiguration(); //Save the current settings to flash and BBR;
    // FFT operation using CMSIS DSP library
    const uint32_t fftSize = 1024;
    const uint32_t ifftFlag = 0;
    const uint32_t doBitReverse = 1;

    // Input signal (example: sine wave)
    float32_t inputSignal[fftSize];
    for (uint32_t i = 0; i < fftSize; i++) {
        inputSignal[i] = arm_sin_f32(2 * PI * i / fftSize);
    }

    // Output buffer
    float32_t outputSignal[fftSize];

    // FFT instance
    arm_rfft_fast_instance_f32 fftInstance;
    arm_rfft_fast_init_f32(&fftInstance, fftSize);

    // Perform FFT
    arm_rfft_fast_f32(&fftInstance, inputSignal, outputSignal, ifftFlag);

    // Print FFT result
    for (uint32_t i = 0; i < fftSize; i++) {
        printmsg("FFT Output[%d]: %f\r\n", i, outputSignal[i]);
    }

        // Start scheduler 
        vTaskStartScheduler();
    }
    else
        printmsg("Unable to connect \r\n"); 

    while(1){
        halImpl.HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_7);
        halImpl.HAL_Delay(1000);
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    //static uint32_t old_pos = 0; // Define and initialize old_pos to 0
    // Check if the interrupt is for the correct UART instance
    if (huart->Instance == myGNSS._serialPort->Instance) // Replace USARTx with your UART instance
    {
        // Read the received byte and store it in the circular buffer
        myGNSS.circular_buffer_write(&myGNSS.RX_Buffer, myGNSS.receivedBytes, myGNSS.numberOfBytes);
        // Set the flag to indicate data has been received
        myGNSS.dataReceived = true;

        // Re-enable the UART receive interrupt
        HAL_UART_Receive_IT(huart, (uint8_t*)&myGNSS.receivedBytes, sizeof(myGNSS.receivedBytes));

        // Re-enable UART DMA Receive
        //HAL_UART_Receive_DMA(huart, (uint8_t*)&myGNSS.receivedBytes, sizeof(myGNSS.receivedBytes));
    }
}

void initUARTInterrupt(UART_HandleTypeDef *huart)
{
    // Start the UART receive interrupt
    HAL_UART_Receive_IT(huart, (uint8_t*)&myGNSS.receivedBytes, sizeof(myGNSS.receivedBytes));
}

void initUARTDMA(UART_HandleTypeDef *huart)
{
    // Start the UART receive interrupt
    HAL_UART_Receive_DMA(huart, (uint8_t*)&myGNSS.receivedBytes, sizeof(myGNSS.receivedBytes));
}


 
void generate_wave(const float32_t params[MAX_PARAMS][3], int num_params, float32_t duration, float32_t sample_rate, float32_t time[MAX_LENGTH], float32_t combined_wave[MAX_LENGTH]) {
    int length = static_cast<int>(duration * sample_rate);

    for (int i = 0; i < length; ++i) {
        time[i] = i / sample_rate;
        combined_wave[i] = 0.0f;
    }

    for (int p = 0; p < num_params; ++p) {
        float32_t amplitude = params[p][0];
        float32_t frequency = params[p][1];
        float32_t phase = params[p][2];

        for (int i = 0; i < length; ++i) {
            combined_wave[i] += amplitude * arm_sin_f32(2 * PI * frequency * time[i] + phase);
        }
    }
}

void compute_welch(const float32_t signal[MAX_LENGTH], int length, float32_t sample_rate, float32_t frequencies[NPERSEG], float32_t psd[NPERSEG]) {
    int nperseg = NPERSEG;
    int noverlap = NOVERLAP;
    int step = nperseg - noverlap;
    int num_segments = (length - noverlap) / step;

    float32_t window[NPERSEG];
    //arm_hanning_f32(window, nperseg);

    float32_t segment[NPERSEG];
    float32_t fft_output[NPERSEG];
    float32_t periodogram[NPERSEG];

    for (int j = 0; j < nperseg; ++j) {
        psd[j] = 0.0f;
    }

    for (int i = 0; i < num_segments; ++i) {
        for (int j = 0; j < nperseg; ++j) {
            segment[j] = signal[i * step + j];
        }
        arm_mult_f32(segment, window, segment, nperseg);

        arm_rfft_fast_instance_f32 fft_instance;
        arm_rfft_fast_init_f32(&fft_instance, nperseg);
        arm_rfft_fast_f32(&fft_instance, segment, fft_output, 0);

        arm_cmplx_mag_squared_f32(fft_output, periodogram, nperseg);

        for (int j = 0; j < nperseg; ++j) {
            psd[j] += periodogram[j];
        }
    }

    for (int j = 0; j < nperseg; ++j) {
        psd[j] /= num_segments;
    }

    for (int j = 0; j < nperseg; ++j) {
        frequencies[j] = j * sample_rate / nperseg;
    }
}

void integrate_displacement(const float32_t psd[NPERSEG], const float32_t frequencies[NPERSEG], float32_t displacement_spectrum[NPERSEG]) {
    for (int i = 1; i < NPERSEG; ++i) {
        float32_t omega = 2 * PI * frequencies[i];
        displacement_spectrum[i] = psd[i] / (omega * omega);
    }
}

void apply_bandpass_filter(float32_t spectrum[NPERSEG], const float32_t frequencies[NPERSEG], float32_t low_cut, float32_t high_cut) {
    for (int i = 0; i < NPERSEG; ++i) {
        if (frequencies[i] < low_cut || frequencies[i] > high_cut) {
            spectrum[i] = 0.0f;
        }
    }
}

void calculate_wave_parameters(const float32_t displacement_spectrum[NPERSEG], const float32_t frequencies[NPERSEG], float32_t& significant_wave_height, float32_t& mean_wave_period, float32_t& average_zero_crossing_period) {
    float32_t m0 = 0.0f;
    float32_t m1 = 0.0f;
    float32_t m2 = 0.0f;

    for (int i = 1; i < NPERSEG; ++i) {
        float32_t df = frequencies[i] - frequencies[i - 1];
        m0 += displacement_spectrum[i] * df;
        m1 += frequencies[i] * displacement_spectrum[i] * df;
        m2 += frequencies[i] * frequencies[i] * displacement_spectrum[i] * df;
    }

    significant_wave_height = 4 * sqrtf(m0);
    mean_wave_period = m0 / m1;
    average_zero_crossing_period = m2 / m0;
}
 
//Debug Print
void printmsg(char *format,...) {
    char str[80];

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
    uint8_t dataIndex = 0;

    for (;;){
        vTaskDelay(DELAY_800_MS);
 
        long latitude = myGNSS.getLatitude();   //Returns the latitude in degrees x10^-7 as a long integer
        printmsg("Latitude: %d\r\n", latitude);

        long longitude = myGNSS.getLongitude(); //Returns the longitude in degrees x10^-7 as a long integer
        printmsg("Longitude: %d\r\n", longitude);

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

        uint32_t downVelocity = myGNSS.getNedDownVel();
        uint32_t epochTime = myGNSS.getUnixEpoch();

        if (dataIndex < MAX_DATA_PAIRS) {
            gpsDataArray[dataIndex].epochTime = epochTime;
            gpsDataArray[dataIndex].downVelocity = downVelocity;
            dataIndex++;
        } else {
            // Handle the case when the array is full (e.g., overwrite, reset, or stop storing)
            printmsg("Data array is full\r\n");
        }
        printmsg("Epoch Time: %d\r\n", epochTime);
        printmsg("Down Velocity: %d\r\n", downVelocity);

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

    }
}

/**
 * @brief Default mode is to put t  he Cortex-M4 in sleep mode when the RTOS is idle.
 * 
 */
void vApplicationIdleHook(void) {
        // Put the Cortex-M4 into sleep mode
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



