#ifndef ICM42688P_H
#define ICM42688P_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

// Error Codes
#define ICM42688P_OK      0
#define ICM42688P_ERR    -1

// Define NULL 
#define NULL 0 

// Device I2C Address (assuming AD0 pin is low)
#define ICM42688P_I2C_ADDRESS  0x68

// Register Addresses
#define ICM42688P_WHO_AM_I_REG      0x75
#define ICM42688P_DEVICE_ID         0x47  // Replace with actual device ID from datasheet
#define ICM42688P_PWR_MGMT0_REG     0x4E
#define ICM42688P_GYRO_CONFIG0_REG  0x4F
#define ICM42688P_ACCEL_CONFIG0_REG 0x50
#define ICM42688P_ACCEL_DATA_REG    0x1F
#define ICM42688P_GYRO_DATA_REG     0x25

// Device Structure
typedef struct {
    int8_t (*read)(uint8_t reg_addr, uint8_t *data, uint16_t len, void *intf_ptr);
    int8_t (*write)(uint8_t reg_addr, const uint8_t *data, uint16_t len, void *intf_ptr);
    int8_t (*transmit_receive)(uint8_t *tx_data, uint8_t *rx_data, uint16_t len, void *intf_ptr);
    void (*gpio_write_nss_pin)(uint8_t state);
    void (*delay_ms)(uint32_t period);
    void *intf_ptr;
} icm42688p_dev_t;

// Function Prototypes
int8_t icm42688p_init(icm42688p_dev_t *dev);
int8_t icm42688p_read_accel_data(icm42688p_dev_t *dev, int16_t *accel_data);
int8_t icm42688p_read_gyro_data(icm42688p_dev_t *dev, int16_t *gyro_data);

#ifdef __cplusplus
}
#endif

#endif // ICM42688P_H
