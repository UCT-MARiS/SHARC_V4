#include "ICM42688P.h"
#include "ICM42688P_registers.h"


/**
 * @brief Initialize the ICM42688P IMU.
 * 
 * @param dev 
 * @return int8_t 
 */
int8_t icm42688p_init(icm42688p_dev_t *dev) {

    // Check if the device structure is valid
    if (dev == NULL || dev->transmit_receive == NULL || dev->intf_ptr == NULL) {
        return ICM42688P_ERR;
    }

    //status variable
    int8_t result = ICM42688P_OK;

    // SPI command to check if ICM42688P IMU is present
    uint8_t who_am_i_reg = 0x75; // WHO_AM_I register address for ICM42688P
    uint8_t who_am_i_value = 0x00; // Variable to store the read value
    uint8_t tx_buffer[2] = { who_am_i_reg | 0x80, 0x00 }; // Read command (MSB set to 1 for read)
    uint8_t rx_buffer[2] = { 0x00, 0x00 }; // Buffer to store received data

    // Select the correct register bank on the IMU
    uint8_t bank_select_reg = 0x76; // Register address for BANK_SEL
    uint8_t bank_select_value = 0x00; // Value to select bank 0
    uint8_t tx_bank_buffer[2] = { bank_select_reg, bank_select_value };

    // Write to the BANK_SEL register
    result = dev->write(bank_select_reg, &bank_select_value, 1, dev->intf_ptr);
    if (result != ICM42688P_OK) {
        return result;
    }

    // Transmit and receive data
    result = dev->transmit_receive(tx_buffer, rx_buffer, 2, dev->intf_ptr);
    if(result != ICM42688P_OK) {
         return result;
    }

    // Copy the received data
    who_am_i_value = rx_buffer[1];

    // Verify Device ID
    if (who_am_i_value != ICM42688P_DEVICE_ID) 
    {
        return ICM42688P_ERR;
    }

    //Reset the device
    BANK_0 reg0 = BANK_0_DEVICE_CONFIG;
    DEVICE_CONFIG_BIT bitmask_soft_reset = DEVICE_CONFIG_BIT_SOFT_RESET_CONFIG; 
    result = dev->write(reg0, &bitmask_soft_reset, 1, dev->intf_ptr);
    if (result != ICM42688P_OK)    
    {
        return result;
    }

    // Wait for reset to complete
    dev->delay_ms(50); // Delay 50 ms
    

    // Set the low noise mode for the accelerometer and gyroscope
    PWR_MGMT0_BIT bitmask_accel_gyro_low_noise = PWR_MGMT0_BIT_GYRO_MODE_LOW_NOISE | PWR_MGMT0_BIT_ACCEL_MODE_LOW_NOISE;
    // Set mode
    result = dev->write(BANK_0_PWR_MGMT0, &bitmask_accel_gyro_low_noise, 1, dev->intf_ptr);
    if (result != ICM42688P_OK)    
    {
        return result;
    }

    return ICM42688P_OK;
}


/**
 * @brief Configure the ICM42688P for polling
 * 
 * @param dev 
 * @return int8_t 
 */
int8_t icm42688p_configure_polling(icm42688p_dev_t *dev)
{
    // Check if the device structure is valid
    if (dev == NULL || dev->transmit_receive == NULL || dev->intf_ptr == NULL) {
        return ICM42688P_ERR;
    }

    //status variable
    int8_t result = ICM42688P_OK;

    BANK_0 bank0 = BANK_0_GYRO_CONFIG0;

    // Set the low noise mode for the accelerometer and gyroscope
    PWR_MGMT0_BIT bitmask_accel_gyro_low_noise = PWR_MGMT0_BIT_GYRO_MODE_LOW_NOISE | PWR_MGMT0_BIT_ACCEL_MODE_LOW_NOISE;

    result = dev->write(ICM42688P_PWR_MGMT0_REG, &bitmask_accel_gyro_low_noise, 1, dev->intf_ptr);
    if (result != ICM42688P_OK)    
    {
        return result;
    }

    // Configure the accelerometer and gyroscope ranges
    ACCEL_CONFIG0_BIT bitmask_accel_range = ACCEL_CONFIG0_BIT_ACCEL_FS_SEL_16G;
    GYRO_CONFIG0_BIT bitmask_gyro_range = GYRO_CONFIG0_BIT_GYRO_FS_SEL_2000_DPS;


    result = icm42688p_set_accel_gyro_odr(dev, bitmask_accel_range, bitmask_gyro_range);
    if (result != ICM42688P_OK)    
    {
        return result;
    }
    // Set the accelerometer and gyroscope output data rates
    ACCEL_CONFIG0_BIT bitmask_accel_odr_set = ACCEL_CONFIG0_BIT_ACCEL_ODR_100HZ_SET;
    ACCEL_CONFIG0_BIT bitmask_accel_odr_clear = ~ACCEL_CONFIG0_BIT_ACCEL_ODR_100HZ_CLEAR;
    ACCEL_CONFIG0_BIT bitmask_accel_odr = bitmask_accel_odr_set | bitmask_accel_odr_clear;
    
    GYRO_CONFIG0_BIT bitmask_gyro_odr_set = GYRO_CONFIG0_BIT_GYRO_ODR_100HZ_SET;
    GYRO_CONFIG0_BIT bitmask_gyro_odr_clear = ~GYRO_CONFIG0_BIT_GYRO_ODR_100HZ_CLEAR;
    ACCEL_CONFIG0_BIT bitmask_gyro_odr = bitmask_gyro_odr_set | bitmask_gyro_odr_clear;

    result = icm42688p_set_accel_gyro_odr(dev, bitmask_accel_odr, bitmask_gyro_odr);

    if (result != ICM42688P_OK)    
    {
        return result;
    }


    return ICM42688P_OK;

}


/**
 * @brief Read accelerometer data from the ICM42688P.
 * 
 * @param dev 
 * @param accel_data 
 * @return int8_t 
 */
int8_t icm42688p_read_accel_data(icm42688p_dev_t *dev, int16_t *accel_data) {
    uint8_t raw_data[6];
    int8_t result;

    result = dev->read(ICM42688P_ACCEL_DATA_REG, raw_data, 6, dev->intf_ptr);
    if (result != ICM42688P_OK) {
        return result;
    }

    // Convert raw data to 16-bit values
    accel_data[0] = (int16_t)((raw_data[0] << 8) | raw_data[1]);
    accel_data[1] = (int16_t)((raw_data[2] << 8) | raw_data[3]);
    accel_data[2] = (int16_t)((raw_data[4] << 8) | raw_data[5]);

    return ICM42688P_OK;
}


/**
 * @brief Read gyroscope data from the ICM42688P.
 * 
 * @param dev 
 * @param gyro_data 
 * @return int8_t 
 */
int8_t icm42688p_read_gyro_data(icm42688p_dev_t *dev, int16_t *gyro_data) {
    uint8_t raw_data[6];
    int8_t result;

    result = dev->read(ICM42688P_GYRO_DATA_REG, raw_data, 6, dev->intf_ptr);
    if (result != ICM42688P_OK) {
        return result;
    }

    // Convert raw data to 16-bit values
    gyro_data[0] = (int16_t)((raw_data[0] << 8) | raw_data[1]);
    gyro_data[1] = (int16_t)((raw_data[2] << 8) | raw_data[3]);
    gyro_data[2] = (int16_t)((raw_data[4] << 8) | raw_data[5]);

    return ICM42688P_OK;
}


/**
 * @brief Change the register bank on the ICM42688P.
 * 
 * @param dev 
 * @param bank 
 * @return int8_t 
 */
int8_t icm42688p_change_bank(icm42688p_dev_t *dev, uint8_t bank) {
    // Check if the device structure is valid
    if (dev == NULL || dev->write == NULL || dev->intf_ptr == NULL) {
        return ICM42688P_ERR;
    }

    // Register address for BANK_SEL
    BANK_0 bank_select_reg = BANK_0_REG_BANK_SEL;
    uint8_t tx_buffer[2] = { bank_select_reg, bank };

    // Write to the BANK_SEL register
    int8_t result = dev->write(bank_select_reg, &bank, 1, dev->intf_ptr);
    if (result != ICM42688P_OK) {
        return result;
    }

    return ICM42688P_OK;
}

/**
 * @brief Set the accelerometer and gyroscope full-scale ranges.
 * 
 * @param dev 
 * @param accel_range 
 * @param gyro_range 
 * @return int8_t 
 */
int8_t icm42688p_set_accel_gyro_odr(icm42688p_dev_t *dev, uint8_t accel_range, uint8_t gyro_range) {
    // Check if the device structure is valid
    if (dev == NULL || dev->write == NULL || dev->intf_ptr == NULL) {
        return ICM42688P_ERR;
    }

    int8_t result;

    // Change to the appropriate bank for accelerometer and gyroscope configuration
    result = icm42688p_change_bank(dev, 0);
    if (result != ICM42688P_OK) {
        return result;
    }

    // Set accelerometer range
    uint8_t accel_config_reg = ICM42688P_ACCEL_CONFIG0_REG;
    result = dev->write(accel_config_reg, &accel_range, 1, dev->intf_ptr);
    if (result != ICM42688P_OK) {
        return result;
    }

    // Set gyroscope range
    uint8_t gyro_config_reg = ICM42688P_GYRO_CONFIG0_REG;
    result = dev->write(gyro_config_reg, &gyro_range, 1, dev->intf_ptr);
    if (result != ICM42688P_OK) {
        return result;
    }

    return ICM42688P_OK;
}


int8_t set_accel_gyro_odr(icm42688p_dev_t *dev, uint8_t accel_odr, uint8_t gyro_odr)
{
        // Check if the device structure is valid
    if (dev == NULL || dev->write == NULL || dev->intf_ptr == NULL) {
        return ICM42688P_ERR;
    }

    int8_t result;

    // Change to the appropriate bank for accelerometer and gyroscope configuration
    result = icm42688p_change_bank(dev, 0);
    if (result != ICM42688P_OK) {
        return result;
    }

    // Set accelerometer range
    uint8_t accel_config_reg = ICM42688P_ACCEL_CONFIG0_REG;
    result = dev->write(accel_config_reg, &accel_odr, 1, dev->intf_ptr);
    if (result != ICM42688P_OK) {
        return result;
    }

    // Set gyroscope range
    uint8_t gyro_config_reg = ICM42688P_GYRO_CONFIG0_REG;
    result = dev->write(gyro_config_reg, &gyro_odr, 1, dev->intf_ptr);
    if (result != ICM42688P_OK) {
        return result;
    }

    return ICM42688P_OK;
    
}




// int8_t icm42688p_read_fifo_dma(icm42688p_dev_t *dev, uint8_t *fifo_buffer, uint16_t buffer_length) {
//     int8_t result;

//     // Check if the device structure and buffer are valid
//     if (dev == NULL || fifo_buffer == NULL || buffer_length == 0) {
//         return ICM42688P_ERR;
//     }

//     // Select the device
//     dev->gpio_write_nss_pin(0);

//     // Set up the FIFO read command
//     uint8_t fifo_read_cmd = ICM42688P_FIFO_DATA_REG | 0x80; // Read command (MSB set to 1 for read)

//     // Start the DMA transfer
//     result = dev->transmit_receive_dma(&fifo_read_cmd, fifo_buffer, buffer_length, dev->intf_ptr);
//     if (result != ICM42688P_OK) {
//         // Deselect the device in case of error
//         dev->gpio_write_nss_pin(1);
//         return result;
//     }

//     // Wait for DMA transfer to complete
//     while (!dev->dma_transfer_complete) {
//         // Optionally, add a timeout mechanism here
//     }

//     // Deselect the device
//     dev->gpio_write_nss_pin(1);

//     return ICM42688P_OK;
// }

// int8_t icm42688p_start_fifo_dma_transfer(icm42688p_dev_t *dev, uint8_t *fifo_buffer, uint16_t buffer_length) {
//     int8_t result;

//     // Check if the device structure and buffer are valid
//     if (dev == NULL || fifo_buffer == NULL || buffer_length == 0) {
//         return ICM42688P_ERR;
//     }

//     // Select the device
//     dev->gpio_write_nss_pin(0);

//     // Set up the FIFO read command
//     uint8_t fifo_read_cmd = ICM42688P_FIFO_DATA_REG | 0x80; // Read command (MSB set to 1 for read)

//     // Start the DMA transfer
//     result = dev->transmit_receive_dma(&fifo_read_cmd, fifo_buffer, buffer_length, dev->intf_ptr);
//     if (result != ICM42688P_OK) {
//         // Deselect the device in case of error
//         dev->gpio_write_nss_pin(1);
//         return result;
//     }

//     return ICM42688P_OK;
// }