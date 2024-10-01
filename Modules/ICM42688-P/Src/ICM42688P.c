#include "ICM42688P.h"

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

    // Select the device
    dev->gpio_write_nss_pin(0);

    // Select the correct register bank on the IMU
    uint8_t bank_select_reg = 0x76; // Register address for BANK_SEL
    uint8_t bank_select_value = 0x00; // Value to select bank 0
    uint8_t tx_bank_buffer[2] = { bank_select_reg, bank_select_value };

    // Write to the BANK_SEL register
    dev->write(bank_select_reg, &bank_select_value, 1, dev->intf_ptr);

    // Transmit and receive data
    result = dev->transmit_receive(tx_buffer, rx_buffer, 2, dev->intf_ptr);
    if(result != ICM42688P_OK) {
         return result;
    }

    // Copy the received data
    who_am_i_value = rx_buffer[1];

    // Deselect the device
    dev->gpio_write_nss_pin(1);

    // Verify Device ID
    if (who_am_i_value != ICM42688P_DEVICE_ID) {
        return ICM42688P_ERR;
    }

    // Reset the device
    uint8_t reset_cmd = 0x00; // Assuming 0x01 resets the device; check datasheet
    result = dev->write(ICM42688P_PWR_MGMT0_REG, &reset_cmd, 1, dev->intf_ptr);
    if (result != ICM42688P_OK)    
    {
        return result;
    }

    // Wait for reset to complete
    dev->delay_ms(10);

    return ICM42688P_OK;

    // Configure Gyroscope
    // uint8_t gyro_config0 = 0x00; // Set default gyro settings; modify as needed
    // result = dev->write(ICM42688P_GYRO_CONFIG0_REG, &gyro_config0, 1, dev->intf_ptr);
    // if (result != ICM42688P_OK) 
    // {
    //      return result;
    // }

    // Configure Accelerometer
    // uint8_t accel_config0 = 0x00; // Set default accel settings; modify as needed
    // result = dev->write(ICM42688P_ACCEL_CONFIG0_REG, &accel_config0, 1, dev->intf_ptr);
    // if (result != ICM42688P_OK) {
    //     return result;
    // }

    //return ICM42688P_OK;
}

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