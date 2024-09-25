#include "ICM42688P.h"

int8_t icm42688p_init(icm42688p_dev_t *dev) {
    uint8_t who_am_i = 0;
    int8_t result;

    // Read WHO_AM_I register
    result = dev->read(ICM42688P_WHO_AM_I_REG, &who_am_i, 1, dev->intf_ptr);
    if (result != ICM42688P_OK) {
        return result;
    }

    // Verify Device ID
    if (who_am_i != ICM42688P_DEVICE_ID) {
        return ICM42688P_ERR;
    }

    // Reset the device
    uint8_t reset_cmd = 0x01; // Assuming 0x01 resets the device; check datasheet
    result = dev->write(ICM42688P_PWR_MGMT0_REG, &reset_cmd, 1, dev->intf_ptr);
    if (result != ICM42688P_OK) {
        return result;
    }

    // Wait for reset to complete
    dev->delay_ms(100);

    // Configure Gyroscope
    uint8_t gyro_config0 = 0x00; // Set default gyro settings; modify as needed
    result = dev->write(ICM42688P_GYRO_CONFIG0_REG, &gyro_config0, 1, dev->intf_ptr);
    if (result != ICM42688P_OK) {
        return result;
    }

    // Configure Accelerometer
    uint8_t accel_config0 = 0x00; // Set default accel settings; modify as needed
    result = dev->write(ICM42688P_ACCEL_CONFIG0_REG, &accel_config0, 1, dev->intf_ptr);
    if (result != ICM42688P_OK) {
        return result;
    }

    return ICM42688P_OK;
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
