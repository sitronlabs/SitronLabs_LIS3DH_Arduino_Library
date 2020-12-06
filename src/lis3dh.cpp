/* Self header */
#include <lis3dh.h>

/* Config */
#ifndef CONFIG_LIS3DH_DEBUG_ENABLED
#define CONFIG_LIS3DH_DEBUG_ENABLED 0
#endif

/* Macros */
#if CONFIG_LIS3DH_DEBUG_ENABLED
#define CONFIG_LIS3DH_DEBUG_FUNCTION(x) Serial.println(x)
#else
#define CONFIG_LIS3DH_DEBUG_FUNCTION(x)
#endif

/**
 * Configures the driver with access over I2C.
 * @note Call this from the Arduino setup function.
 * @note Make sure the I2C library has been initialized with a call to its begin function for example.
 * @note When SA0 is low, the address is 0x18, when SA0 is high it's 0x19.
 * @param[in] i2c_library A pointer to the i2c library to use.
 * @param[in] i2c_address The i2c address of the device.
 * @return 0 in case of success, or a negative error code otherwise.
 */
int lis3dh::setup(TwoWire *i2c_library, uint8_t i2c_address) {

    /* Disable spi */
    m_spi_library = NULL;

    /* Enable i2c */
    m_i2c_library = i2c_library;
    m_i2c_address = i2c_address;

    /* Return success */
    return 0;
}

/**
 * Configures the driver with access over SPI.
 * @note Call this from the Arduino setup function.
 * @note Make sure the SPI library has been initialized with a call to its begin function for example.
 * @note Only 4-wire SPI interface is supported for now. If you'd like to use the 3-wire SPI interface, you're welcome to submit a pull-request.
 * @param[in] spi_library A pointer to the spi library to use.
 * @param[in] spi_pin_ss The pin used for chip select.
 * @return 0 in case of success, or a negative error code otherwise.
 */
int lis3dh::setup(SPIClass *spi_library, const int spi_pin_cs, const int spi_speed) {
    int res;

    /* Disable i2c */
    m_i2c_library = NULL;

    /* Enable spi */
    m_spi_library = spi_library;
    m_spi_pin_cs = spi_pin_cs;
    m_spi_speed = spi_speed;

    /* Return success */
    return 0;
}

/**
 * Tries to detect the device.
 * @return true if the device has been detected, or false otherwise.
 */
bool lis3dh::detect(void) {
    int res;

    /* Read identification register */
    uint8_t reg_whoami;
    res = m_register_read(LIS3DH_REGISTER_WHO_AM_I, &reg_whoami);
    if (res < 0) return false;

    /* Return whether or not the value matches */
    return (reg_whoami == 0x33);
}

/**
 *
 * @return
 * @see https://community.st.com/s/question/0D50X00009fDWhfSAG/can-i-safely-use-lis3dh-on-a-common-spi-bus
 * @see https://community.st.com/s/question/0D50X00009XkbQTSAZ/use-lis3dh-on-the-same-spi-bus-as-an-eeprom-is-it-possible
 */
int lis3dh::i2c_disabled_force(void) {
    int res;

    /* Ensure device was not configured with i2c */
    if (m_i2c_library != NULL) {
        return -EINVAL;
    }

    /* Disable i2c */
    uint8_t reg_0x17;
    res = m_register_read((enum lis3dh_register)0x17, &reg_0x17);
    if (res < 0) return res;
    reg_0x17 |= 0x80;
    res = m_register_write((enum lis3dh_register)0x17, reg_0x17);
    if (res < 0) return res;

    /* Return success */
    return 0;
}

/**
 *
 * @see Datasheet section 8.11 "CTRL_REG4 (23h)"
 * @param[out] scale
 * @return 0 in case of success, or a negative error code otherwise.
 */
int lis3dh::range_get(enum lis3dh_range *const range_g) {
    int res;

    /* Read register */
    uint8_t reg_ctrl_reg4;
    res = m_register_read(LIS3DH_REGISTER_CTRL_REG4, &reg_ctrl_reg4);
    if (res < 0) {
        CONFIG_LIS3DH_DEBUG_FUNCTION(" [e] Failed to read full scale register!");
        return res;
    }

    /* Return scale */
    *range_g = (enum lis3dh_range)(1 << (((reg_ctrl_reg4 & 0x30) >> 4) + 1));

    /* Return success */
    return 0;
}

/**
 *
 * @see Datasheet section 8.11 "CTRL_REG4 (23h)"
 * @param[in] scale
 * @return 0 in case of success, or a negative error code otherwise.
 */
int lis3dh::range_set(const enum lis3dh_range range_g) {
    int res;

    /* Read register */
    uint8_t reg_ctrl_reg4;
    res = m_register_read(LIS3DH_REGISTER_CTRL_REG4, &reg_ctrl_reg4);
    if (res < 0) {
        CONFIG_LIS3DH_DEBUG_FUNCTION(" [e] Failed to read full scale register!");
        return res;
    }

    /* Modify register */
    reg_ctrl_reg4 &= 0xCF;
    switch (range_g) {
        case LIS3DH_RANGE_2G: {
            reg_ctrl_reg4 |= (0 << 4);
            break;
        }
        case LIS3DH_RANGE_4G: {
            reg_ctrl_reg4 |= (1 << 4);
            break;
        }
        case LIS3DH_RANGE_8G: {
            reg_ctrl_reg4 |= (2 << 4);
            break;
        }
        case LIS3DH_RANGE_16G: {
            reg_ctrl_reg4 |= (3 << 4);
            break;
        }
        default: {
            CONFIG_LIS3DH_DEBUG_FUNCTION(" [e] Invalid scale!");
            return -EINVAL;
        }
    }
    res = m_register_write(LIS3DH_REGISTER_CTRL_REG4, reg_ctrl_reg4);
    if (res < 0) {
        CONFIG_LIS3DH_DEBUG_FUNCTION(" [e] Failed to write full scale register!");
        return res;
    }

    /* Update activity threshold accordingly */
    if (m_activity_threshold > 0) {
        res = activity_configure(m_activity_threshold, m_activity_duration);
        if (res < 0) {
            CONFIG_LIS3DH_DEBUG_FUNCTION(" [e] Failed to re-adjust activity!");
            return res;
        }
    }

    /* Return success */
    return 0;
}

/**
 * Retrieves the sampling rate.
 * @see Datasheet section 8.8 "CTRL_REG1 (20h)"
 * @param[out] hz A pointer to a variable that will be updated with the rate in hertz.
 * @return 0 in case of success, or a negative error code otherwise.
 */
int lis3dh::sampling_get(enum lis3dh_sampling *const sampling_hz) {
    int res;

    /* Read register */
    uint8_t reg_ctrl_reg1;
    res = m_register_read(LIS3DH_REGISTER_CTRL_REG1, &reg_ctrl_reg1);
    if (res < 0) {
        CONFIG_LIS3DH_DEBUG_FUNCTION(" [e] Failed to read datarate register!");
        return res;
    }

    /* Convert register into hertz */
    switch (reg_ctrl_reg1 >> 4) {
        case 0b0000:
            *sampling_hz = LIS3DH_SAMPLING_NONE;
            break;
        case 0b0001:
            *sampling_hz = LIS3DH_SAMPLING_1HZ;
            break;
        case 0b0010:
            *sampling_hz = LIS3DH_SAMPLING_10HZ;
            break;
        case 0b0011 ... 0b0111:
            *sampling_hz = (enum lis3dh_sampling)(25 * 2 ^ ((reg_ctrl_reg1 >> 4) - 3));
            break;
        case 0b1000:
            *sampling_hz = LIS3DH_SAMPLING_1600HZ;
            break;
        case 0b1001:
            if (reg_ctrl_reg1 & 0x08) *sampling_hz = LIS3DH_SAMPLING_5376HZ;
            else *sampling_hz = LIS3DH_SAMPLING_1344HZ;
            break;
        default: {
            CONFIG_LIS3DH_DEBUG_FUNCTION(" [e] Invalid output data rate!");
            return -EINVAL;
        }
    }

    /* Return success */
    return 0;
}

/**
 * Sets the sampling rate.
 * @see Datasheet section 8.8 "CTRL_REG1 (20h)"
 * @param[in] hz The rate to use in hertz. Can be any of the following values: 0, 1, 10, 25, 50, 100, 200, 400, 1344, 1600, 5376.
 * @return 0 in case of success, or a negative error code otherwise.
 */
int lis3dh::sampling_set(const enum lis3dh_sampling smapling_hz) {
    int res;

    /* Read register */
    uint8_t reg_ctrl_reg1;
    res = m_register_read(LIS3DH_REGISTER_CTRL_REG1, &reg_ctrl_reg1);
    if (res < 0) {
        CONFIG_LIS3DH_DEBUG_FUNCTION(" [e] Failed to read datarate register!");
        return res;
    }

    /* Modify register */
    switch (smapling_hz) {
        case LIS3DH_SAMPLING_1HZ: {
            reg_ctrl_reg1 &= 0x0F;
            reg_ctrl_reg1 |= (1 << 4);
            break;
        }
        case LIS3DH_SAMPLING_10HZ: {
            reg_ctrl_reg1 &= 0x0F;
            reg_ctrl_reg1 |= (2 << 4);
            break;
        }
        case LIS3DH_SAMPLING_25HZ: {
            reg_ctrl_reg1 &= 0x0F;
            reg_ctrl_reg1 |= (3 << 4);
            break;
        }
        case LIS3DH_SAMPLING_50HZ: {
            reg_ctrl_reg1 &= 0x0F;
            reg_ctrl_reg1 |= (4 << 4);
            break;
        }
        case LIS3DH_SAMPLING_100HZ: {
            reg_ctrl_reg1 &= 0x0F;
            reg_ctrl_reg1 |= (5 << 4);
            break;
        }
        case LIS3DH_SAMPLING_200HZ: {
            reg_ctrl_reg1 &= 0x0F;
            reg_ctrl_reg1 |= (6 << 4);
            break;
        }
        case LIS3DH_SAMPLING_400HZ: {
            reg_ctrl_reg1 &= 0x0F;
            reg_ctrl_reg1 |= (7 << 4);
            break;
        }
        case LIS3DH_SAMPLING_1600HZ: {
            reg_ctrl_reg1 &= 0x0F;
            reg_ctrl_reg1 |= (8 << 4);
            reg_ctrl_reg1 |= (1 << 3);  // Force LPen low
            break;
        }
        case LIS3DH_SAMPLING_1344HZ: {
            reg_ctrl_reg1 &= 0x0F;
            reg_ctrl_reg1 |= (9 << 4);
            reg_ctrl_reg1 |= (1 << 3);  // Force LPen high
            break;
        }
        case LIS3DH_SAMPLING_5376HZ: {
            reg_ctrl_reg1 &= 0x07;  // Force LPen low
            reg_ctrl_reg1 |= (9 << 4);
            break;
        }
        default: {
            CONFIG_LIS3DH_DEBUG_FUNCTION(" [e] Invalid output data rate!");
            return -EINVAL;
        }
    }
    res = m_register_write(LIS3DH_REGISTER_CTRL_REG1, reg_ctrl_reg1);
    if (res < 0) {
        CONFIG_LIS3DH_DEBUG_FUNCTION(" [e] Failed to write odr register!");
        return res;
    }

    /* Update activity duration accordingly */
    if (m_activity_threshold > 0) {
        res = activity_configure(m_activity_threshold, m_activity_duration);
        if (res < 0) {
            CONFIG_LIS3DH_DEBUG_FUNCTION(" [e] Failed to re-adjust activity!");
            return res;
        }
    }

    /* Return success */
    return 0;
}

/**
 *
 * @param[in] enabled_x
 * @param[in] enabled_y
 * @param[in] enabled_z
 * @return 0 in case of success, or a negative error code otherwise.
 */
int lis3dh::axis_enabled_set(const bool enabled_x, const bool enabled_y, const bool enabled_z) {
    int res;

    /* Read register */
    uint8_t reg_ctrl_reg1;
    res = m_register_read(LIS3DH_REGISTER_CTRL_REG1, &reg_ctrl_reg1);
    if (res < 0) {
        CONFIG_LIS3DH_DEBUG_FUNCTION(" [e] Failed to read register!");
        return res;
    }

    /* Modify register */
    reg_ctrl_reg1 &= ~(0b111);
    if (enabled_x) reg_ctrl_reg1 |= (1 << 0);
    if (enabled_y) reg_ctrl_reg1 |= (1 << 1);
    if (enabled_z) reg_ctrl_reg1 |= (1 << 2);
    res = m_register_write(LIS3DH_REGISTER_CTRL_REG1, reg_ctrl_reg1);
    if (res < 0) {
        CONFIG_LIS3DH_DEBUG_FUNCTION(" [e] Failed to write register!");
        return res;
    }

    /* Return success */
    return 0;
}

/**
 *
 * @param[in] axis
 * @param[in] enabled
 * @return 0 in case of success, or a negative error code otherwise.
 */
int lis3dh::axis_enabled_set(const enum lis3dh_axis axis, const bool enabled) {
    int res;

    /* Read register */
    uint8_t reg_ctrl_reg1;
    res = m_register_read(LIS3DH_REGISTER_CTRL_REG1, &reg_ctrl_reg1);
    if (res < 0) {
        CONFIG_LIS3DH_DEBUG_FUNCTION(" [e] Failed to read register!");
        return res;
    }

    /* Modify register */
    switch (axis) {
        case LIS3DH_AXIS_X: {
            if (enabled) reg_ctrl_reg1 |= (1 << 0);
            else reg_ctrl_reg1 &= ~(1 << 0);
            break;
        }
        case LIS3DH_AXIS_Y: {
            if (enabled) reg_ctrl_reg1 |= (1 << 1);
            else reg_ctrl_reg1 &= ~(1 << 1);
            break;
        }
        case LIS3DH_AXIS_Z: {
            if (enabled) reg_ctrl_reg1 |= (1 << 2);
            else reg_ctrl_reg1 &= ~(1 << 2);
            break;
        }
        default: {
            CONFIG_LIS3DH_DEBUG_FUNCTION(" [e] Invalid axis!");
            return -EINVAL;
        }
    }
    res = m_register_write(LIS3DH_REGISTER_CTRL_REG1, reg_ctrl_reg1);
    if (res < 0) {
        CONFIG_LIS3DH_DEBUG_FUNCTION(" [e] Failed to write register!");
        return res;
    }

    /* Return success */
    return 0;
}

/**
 *
 * @param[out] accel_x
 * @param[out] accel_y
 * @param[out] accel_z
 * @return
 */
int lis3dh::acceleration_read(int16_t *const accel_x, int16_t *const accel_y, int16_t *const accel_z) {
    int res;

    /* Prepare to read all axis */
    uint8_t reg_out_h, reg_out_l;
    res = 0;

    /* Read x axis */
    res |= m_register_read(LIS3DH_REGISTER_OUT_X_H, &reg_out_h);
    res |= m_register_read(LIS3DH_REGISTER_OUT_X_L, &reg_out_l);
    *accel_x = reg_out_h;
    *accel_x <<= 8;
    *accel_x |= reg_out_l;

    /* Read y axis */
    res |= m_register_read(LIS3DH_REGISTER_OUT_Y_H, &reg_out_h);
    res |= m_register_read(LIS3DH_REGISTER_OUT_Y_L, &reg_out_l);
    *accel_y = reg_out_h;
    *accel_y <<= 8;
    *accel_y |= reg_out_l;

    /* Read z axis */
    res |= m_register_read(LIS3DH_REGISTER_OUT_Z_H, &reg_out_h);
    res |= m_register_read(LIS3DH_REGISTER_OUT_Z_L, &reg_out_l);
    *accel_z = reg_out_h;
    *accel_z <<= 8;
    *accel_z |= reg_out_l;

    /* Return success */
    if (res < 0) return -1;
    else return 0;
}

/**
 *
 * @param[out] accel_x
 * @param[out] accel_y
 * @param[out] accel_z
 * @return
 */
int lis3dh::acceleration_read(float *const accel_x, float *const accel_y, float *const accel_z) {
    int res;

    /* Retrieve scale */
    enum lis3dh_range range;
    res = range_get(&range);
    if (res < 0) {
        CONFIG_LIS3DH_DEBUG_FUNCTION(" [e] Failed to retrieve scale!");
        return res;
    }

    /* Compute divider according to range */
    float divider;
    switch (range) {
        case LIS3DH_RANGE_2G: {
            divider = 15987;
            break;
        }
        case LIS3DH_RANGE_4G: {
            divider = 7840;
            break;
        }
        case LIS3DH_RANGE_8G: {
            divider = 3883;
            break;
        }
        case LIS3DH_RANGE_16G: {
            divider = 1280;
            break;
        }
        default: {
            CONFIG_LIS3DH_DEBUG_FUNCTION(" [e] Invalid scale!");
            return -EINVAL;
        }
    }

    /* Prepare to read all axis */
    uint8_t reg_out_h, reg_out_l;
    int16_t accel_raw;
    res = 0;

    /* Read x axis */
    res |= m_register_read(LIS3DH_REGISTER_OUT_X_H, &reg_out_h);
    res |= m_register_read(LIS3DH_REGISTER_OUT_X_L, &reg_out_l);
    accel_raw = reg_out_h;
    accel_raw <<= 8;
    accel_raw |= reg_out_l;
    *accel_x = accel_raw / divider;

    /* Read y axis */
    res |= m_register_read(LIS3DH_REGISTER_OUT_Y_H, &reg_out_h);
    res |= m_register_read(LIS3DH_REGISTER_OUT_Y_L, &reg_out_l);
    accel_raw = reg_out_h;
    accel_raw <<= 8;
    accel_raw |= reg_out_l;
    *accel_y = accel_raw / divider;

    /* Read z axis */
    res |= m_register_read(LIS3DH_REGISTER_OUT_Z_H, &reg_out_h);
    res |= m_register_read(LIS3DH_REGISTER_OUT_Z_L, &reg_out_l);
    accel_raw = reg_out_h;
    accel_raw <<= 8;
    accel_raw |= reg_out_l;
    *accel_z = accel_raw / divider;

    /* Return success */
    if (res < 0) return -1;
    else return 0;
}

/**
 *
 * @param[out] accel_x
 * @param[out] accel_y
 * @param[out] accel_z
 * @return
 */
int lis3dh::acceleration_read(double *const accel_x, double *const accel_y, double *const accel_z) {
    int res;

    /* Retrieve scale */
    enum lis3dh_range range;
    res = range_get(&range);
    if (res < 0) {
        CONFIG_LIS3DH_DEBUG_FUNCTION(" [e] Failed to retrieve scale!");
        return res;
    }

    /* Compute divider according to range */
    double divider;
    switch (range) {
        case LIS3DH_RANGE_2G: {
            divider = 15987;
            break;
        }
        case LIS3DH_RANGE_4G: {
            divider = 7840;
            break;
        }
        case LIS3DH_RANGE_8G: {
            divider = 3883;
            break;
        }
        case LIS3DH_RANGE_16G: {
            divider = 1280;
            break;
        }
        default: {
            CONFIG_LIS3DH_DEBUG_FUNCTION(" [e] Invalid scale!");
            return -EINVAL;
        }
    }

    /* Prepare to read all axis */
    uint8_t reg_out_h, reg_out_l;
    int16_t accel_raw;
    res = 0;

    /* Read x axis */
    res |= m_register_read(LIS3DH_REGISTER_OUT_X_H, &reg_out_h);
    res |= m_register_read(LIS3DH_REGISTER_OUT_X_L, &reg_out_l);
    accel_raw = reg_out_h;
    accel_raw <<= 8;
    accel_raw |= reg_out_l;
    *accel_x = accel_raw / divider;

    /* Read y axis */
    res |= m_register_read(LIS3DH_REGISTER_OUT_Y_H, &reg_out_h);
    res |= m_register_read(LIS3DH_REGISTER_OUT_Y_L, &reg_out_l);
    accel_raw = reg_out_h;
    accel_raw <<= 8;
    accel_raw |= reg_out_l;
    *accel_y = accel_raw / divider;

    /* Read z axis */
    res |= m_register_read(LIS3DH_REGISTER_OUT_Z_H, &reg_out_h);
    res |= m_register_read(LIS3DH_REGISTER_OUT_Z_L, &reg_out_l);
    accel_raw = reg_out_h;
    accel_raw <<= 8;
    accel_raw |= reg_out_l;
    *accel_z = accel_raw / divider;

    /* Return success */
    if (res < 0) return -1;
    else return 0;
}

/**
 * 
 * @see Application note section 5.1 "Interrupt pin configuration"
 * @see Datasheet section 8.13 "CTRL_REG6 (25h)"
 * @return 0 in case of success, or a negative error code otherwise.
 */
int lis3dh::interrupts_polarity_set(const enum lis3dh_interrupt_polarity polarity) {
    int res;

    /* Read register */
    uint8_t reg_ctrl_reg6;
    res = m_register_read(LIS3DH_REGISTER_CTRL_REG6, &reg_ctrl_reg6);
    if (res < 0) {
        CONFIG_LIS3DH_DEBUG_FUNCTION(" [e] Failed to read register!");
        return res;
    }

    /* Modify register */
    if (polarity == LIS3DH_INTERRUPT_POLARITY_ACTIVE_LOW) {
        reg_ctrl_reg6 |= (1 << 1);
    } else {
        reg_ctrl_reg6 &= ~(1 << 1);
    }
    res = m_register_write(LIS3DH_REGISTER_CTRL_REG6, reg_ctrl_reg6);
    if (res < 0) {
        CONFIG_LIS3DH_DEBUG_FUNCTION(" [e] Failed to write register!");
        return res;
    }

    /* Return success */
    return 0;
}

/**
 * Configures the (in)activity detection feature.
 * @see Application note section 10 "Activity / Inactivity recognition"
 * @see Datasheet section 3.2.4 "Sleep-to-wake and Return-to-sleep"
 * @see Datasheet section 8.35 "ACT_THS (3Eh)"
 * @see Datasheet section 8.36 "ACT_DUR (3Fh)"
 * @param[in] threshold Threshold in millig above which movements will be considered as activity.
 * @param[in] duration Duration in milliseconds before entering inactivity state when movement is below threshold.
 * @return 0 in case of success, or a negative error code otherwise.
 */
int lis3dh::activity_configure(const uint16_t threshold_mg, const uint32_t duration_ms) {
    int res;

    /* Retrieve scale */
    enum lis3dh_range range;
    res = range_get(&range);
    if (res < 0) {
        CONFIG_LIS3DH_DEBUG_FUNCTION(" [e] Failed to retrieve scale!");
        return res;
    }

    /* Convert threshold to the closest value depending on scale */
    uint16_t reg_act_ths;
    switch (range) {
        case LIS3DH_RANGE_2G: {
            reg_act_ths = threshold_mg / 16;
            break;
        }
        case LIS3DH_RANGE_4G: {
            reg_act_ths = threshold_mg / 32;
            break;
        }
        case LIS3DH_RANGE_8G: {
            reg_act_ths = threshold_mg / 62;
            break;
        }
        case LIS3DH_RANGE_16G: {
            reg_act_ths = threshold_mg / 186;
            break;
        }
        default: {
            CONFIG_LIS3DH_DEBUG_FUNCTION(" [e] Invalid scale 2!");
            return -EINVAL;
        }
    }

    /* Ensure threshold register would not be out of scale */
    if (reg_act_ths & ~0x7F) {
        CONFIG_LIS3DH_DEBUG_FUNCTION(" [e] Threshold out of scale!");
        return -E2BIG;
    }

    /* Update threshold register */
    res = m_register_write(LIS3DH_REGISTER_ACT_THS, (uint8_t)(0x7F & reg_act_ths));
    if (res < 0) {
        CONFIG_LIS3DH_DEBUG_FUNCTION(" [e] Failed to write threshold register!");
        return res;
    }

    /* Debug */
    Serial.print(" [d] LIS3DH_REGISTER_ACT_THS = 0x");
    Serial.println((uint8_t)(0x7F & reg_act_ths), HEX);

    /* Retrieve data rate */
    enum lis3dh_sampling odr_hz;
    res = sampling_get(&odr_hz);
    if (res < 0) {
        CONFIG_LIS3DH_DEBUG_FUNCTION(" [e] Failed to retrieve data rate!");
        return res;
    }

    /* Ensure duration register would not be out of bounds */
    const uint32_t duration_ms_max = ((8L * 255 + 1) * 1000) / odr_hz;
    if (duration_ms > duration_ms_max) {
        CONFIG_LIS3DH_DEBUG_FUNCTION(" [e] Duration out of scale!");
        return -E2BIG;
    }

    /* Update duration register */
    uint8_t reg_inact_dur = (round((duration_ms * odr_hz) / 1000.0) - 1) / 8;
    res = m_register_write(LIS3DH_REGISTER_INACT_DUR, reg_inact_dur);
    if (res < 0) {
        CONFIG_LIS3DH_DEBUG_FUNCTION(" [e] Failed to write threshold register!");
        return res;
    }

    Serial.print(" [d] LIS3DH_REGISTER_INACT_DUR = 0x");
    Serial.println(reg_inact_dur, HEX);

    Serial.print(" [d] Using duration of ");
    Serial.print(((8 * reg_inact_dur) + 1) / ((float)odr_hz), 4);
    Serial.println("s");

    /* Store original values to be used when adjusting scale and odr */
    m_activity_threshold = threshold_mg;
    m_activity_duration = duration_ms;

    /* Return success */
    return 0;
}

/**
 * Route the (in)activity status to the INT2 pin.
 * With this feature INT2 is high when the system is in inactivity and goes low when the system is in activity.
 * The INT_POLARITY bit controls the polarity of the activity / inactivity signal.
 * @see Application note section 10 "Activity / Inactivity recognition"
 * @param[in] enabled
 * @return 0 in case of success, or a negative error code otherwise.
 */
int lis3dh::activity_int2_routed_set(const bool routed) {
    int res;

    /* Read register */
    uint8_t reg_ctrl_reg6;
    res = m_register_read(LIS3DH_REGISTER_CTRL_REG6, &reg_ctrl_reg6);
    if (res < 0) {
        CONFIG_LIS3DH_DEBUG_FUNCTION(" [e] Failed to read register!");
        return res;
    }

    /* Modify register */
    if (routed) reg_ctrl_reg6 |= (1 << 3);
    else reg_ctrl_reg6 &= ~(1 << 3);
    res = m_register_write(LIS3DH_REGISTER_CTRL_REG6, reg_ctrl_reg6);
    if (res < 0) {
        CONFIG_LIS3DH_DEBUG_FUNCTION(" [e] Failed to write register!");
        return res;
    }

    /* Return success */
    return 0;
}

/**
 * Reads the value of a register from the device over i2c or spi.
 * @param[in] reg_addr The address of the register.
 * @param[out] reg_value A pointer to a variable that will be updated with the value of the register.
 * @return 0 in case of success, or a negative error code otherwise.
 */
int lis3dh::m_register_read(const enum lis3dh_register reg_addr, uint8_t *const reg_value) {

    /* Access over i2c
	 * @see Datasheet section 6.1 "I2C serial interface" */
    if (m_i2c_library) {
        m_i2c_library->beginTransmission(m_i2c_address);
        m_i2c_library->write((uint8_t)reg_addr);
        if (m_i2c_library->endTransmission(false) != 0) return -EIO;
        if (m_i2c_library->requestFrom(m_i2c_address, (uint8_t)1, (uint8_t) true) != 1) return -EIO;
        *reg_value = m_i2c_library->read();
    }

    /* Access over spi
	 * @see Datasheet section 6.2 "SPI bus interface" */
    else if (m_spi_library) {
        m_spi_library->beginTransaction(SPISettings(m_spi_speed, MSBFIRST, SPI_MODE3));
        pinMode(m_spi_pin_cs, OUTPUT);
        digitalWrite(m_spi_pin_cs, LOW);
        m_spi_library->transfer(reg_addr | 0x80);
        *reg_value = m_spi_library->transfer(0x00);
        digitalWrite(m_spi_pin_cs, HIGH);
        m_spi_library->endTransaction();
    }

    /* No access medium configured */
    else {
        return -EINVAL;
    }

    /* Return success */
    return 0;
}

/**
 * Reads the value of multiple registers from the device over i2c or spi.
 * @param[in] reg_addr The address of the register.
 * @param[out] reg_values A pointer to a variable that will be updated with the value of the register.
 * @return 0 in case of success, or a negative error code otherwise.
 */
int lis3dh::m_register_read(const enum lis3dh_register reg_addr, uint8_t *const reg_values, const size_t count) {

    /* Access over i2c
	 * @see Datasheet section 6.1 "I2C serial interface" */
    if (m_i2c_library) {
        m_i2c_library->beginTransmission(m_i2c_address);
        m_i2c_library->write((uint8_t)reg_addr);
        if (m_i2c_library->endTransmission(false) != 0) return -EIO;
        if (m_i2c_library->requestFrom(m_i2c_address, (uint8_t)count, (uint8_t) true) != count) return -EIO;
        for (size_t i = 0; i < count; i++) {
            reg_values[i] = m_i2c_library->read();
        }
    }

    /* Access over spi
	 * @see Datasheet section 6.2 "SPI bus interface" */
    else if (m_spi_library) {
        m_spi_library->beginTransaction(SPISettings(m_spi_speed, MSBFIRST, SPI_MODE3));
        pinMode(m_spi_pin_cs, OUTPUT);
        digitalWrite(m_spi_pin_cs, LOW);
        m_spi_library->transfer(reg_addr | 0x80 | 0x40);
        for (size_t i = 0; i < count; i++) {
            reg_values[i] = m_spi_library->transfer(0x00);
        }
        digitalWrite(m_spi_pin_cs, HIGH);
        m_spi_library->endTransaction();
    }

    /* No access medium configured */
    else {
        return -EINVAL;
    }

    /* Return success */
    return 0;
}

/**
 * Writes a value in a register of the device over i2c or spi.
 * @param[in] reg_addr The address of the register.
 * @param[in] reg_value The new value of the register.
 * @return 0 in case of success, or a negative error code otherwise.
 */
int lis3dh::m_register_write(const enum lis3dh_register reg_addr, const uint8_t reg_value) {

    /* Access over i2c
	 * @see Datasheet section 6.1 "I2C serial interface" */
    if (m_i2c_library) {
        m_i2c_library->beginTransmission(m_i2c_address);
        m_i2c_library->write((uint8_t)reg_addr);
        m_i2c_library->write(reg_value);
        if (m_i2c_library->endTransmission(true) != 0) return -EIO;
    }

    /* Access over spi
	 * @see Datasheet section 6.2 "SPI bus interface" */
    else if (m_spi_library) {
        m_spi_library->beginTransaction(SPISettings(m_spi_speed, MSBFIRST, SPI_MODE3));
        pinMode(m_spi_pin_cs, OUTPUT);
        digitalWrite(m_spi_pin_cs, LOW);
        m_spi_library->transfer(reg_addr);
        m_spi_library->transfer(reg_value);
        digitalWrite(m_spi_pin_cs, HIGH);
        m_spi_library->endTransaction();
    }

    /* No access medium configured */
    else {
        return -EINVAL;
    }

    /* Return success */
    return 0;
}
