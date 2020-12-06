#ifndef LIS3DH_H
#define LIS3DH_H

/* Arduino libraries */
#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>

/* C library */
#include <errno.h>
#include <stdint.h>

/**
 * List of available sensor ranges.
 */
enum lis3dh_range {
    LIS3DH_RANGE_2G = 2,
    LIS3DH_RANGE_4G = 4,
    LIS3DH_RANGE_8G = 8,
    LIS3DH_RANGE_16G = 16,
};

/**
 * List of available sampling rates.
 */
enum lis3dh_sampling {
    LIS3DH_SAMPLING_NONE = 0,
    LIS3DH_SAMPLING_1HZ = 1,
    LIS3DH_SAMPLING_10HZ = 10,
    LIS3DH_SAMPLING_25HZ = 25,
    LIS3DH_SAMPLING_50HZ = 50,
    LIS3DH_SAMPLING_100HZ = 100,
    LIS3DH_SAMPLING_200HZ = 200,
    LIS3DH_SAMPLING_400HZ = 400,
    LIS3DH_SAMPLING_1344HZ = 1344,
    LIS3DH_SAMPLING_1600HZ = 1600,
    LIS3DH_SAMPLING_5376HZ = 5376,
};

/**
 * List of available axis.
 */
enum lis3dh_axis {
    LIS3DH_AXIS_X,
    LIS3DH_AXIS_Y,
    LIS3DH_AXIS_Z,
};

/**
 * List of possible interrupt polarities.
 */
enum lis3dh_interrupt_polarity {
    LIS3DH_INTERRUPT_POLARITY_ACTIVE_HIGH,
    LIS3DH_INTERRUPT_POLARITY_ACTIVE_LOW,
};

/**
 * List of internal registers.
 */
enum lis3dh_register {
    LIS3DH_REGISTER_STATUS_REG_AUX = 0x07,   //!<
    LIS3DH_REGISTER_OUT_ADC1_L = 0x08,       //!<
    LIS3DH_REGISTER_OUT_ADC1_H = 0x09,       //!<
    LIS3DH_REGISTER_OUT_ADC2_L = 0x0A,       //!<
    LIS3DH_REGISTER_OUT_ADC2_H = 0x0B,       //!<
    LIS3DH_REGISTER_OUT_ADC3_L = 0x0C,       //!<
    LIS3DH_REGISTER_OUT_ADC3_H = 0x0D,       //!<
    LIS3DH_REGISTER_INT_COUNTER_REG = 0x0E,  //!<
    LIS3DH_REGISTER_WHO_AM_I = 0x0F,         //!<
    LIS3DH_REGISTER_CTRL_REG0 = 0x1E,        //!<
    LIS3DH_REGISTER_TEMP_CFG_REG = 0x1F,     //!<
    LIS3DH_REGISTER_CTRL_REG1 = 0x20,        //!<
    LIS3DH_REGISTER_CTRL_REG2 = 0x21,        //!<
    LIS3DH_REGISTER_CTRL_REG3 = 0x22,        //!<
    LIS3DH_REGISTER_CTRL_REG4 = 0x23,        //!<
    LIS3DH_REGISTER_CTRL_REG5 = 0x24,        //!<
    LIS3DH_REGISTER_CTRL_REG6 = 0x25,        //!<
    LIS3DH_REGISTER_REFERENCE = 0x26,        //!<
    LIS3DH_REGISTER_STATUS_REG = 0x27,       //!<
    LIS3DH_REGISTER_OUT_X_L = 0x28,          //!<
    LIS3DH_REGISTER_OUT_X_H = 0x29,          //!<
    LIS3DH_REGISTER_OUT_Y_L = 0x2A,          //!<
    LIS3DH_REGISTER_OUT_Y_H = 0x2B,          //!<
    LIS3DH_REGISTER_OUT_Z_L = 0x2C,          //!<
    LIS3DH_REGISTER_OUT_Z_H = 0x2D,          //!<
    LIS3DH_REGISTER_FIFO_CTRL_REG = 0x2E,    //!<
    LIS3DH_REGISTER_FIFO_SRC_REG = 0x2F,     //!<
    LIS3DH_REGISTER_INT1_CFG = 0x30,         //!<
    LIS3DH_REGISTER_INT1_SRC = 0x31,         //!<
    LIS3DH_REGISTER_INT1_THS = 0x32,         //!<
    LIS3DH_REGISTER_INT1_DURATION = 0x33,    //!<
    LIS3DH_REGISTER_INT2_CFG = 0x34,         //!<
    LIS3DH_REGISTER_INT2_SRC = 0x35,         //!<
    LIS3DH_REGISTER_INT2_THS = 0x36,         //!<
    LIS3DH_REGISTER_INT2_DURATION = 0x37,    //!<
    LIS3DH_REGISTER_CLICK_CFG = 0x38,        //!<
    LIS3DH_REGISTER_CLICK_SRC = 0x39,        //!<
    LIS3DH_REGISTER_CLICK_THS = 0x3A,        //!<
    LIS3DH_REGISTER_TIME_LIMIT = 0x3B,       //!<
    LIS3DH_REGISTER_TIME_LATENCY = 0x3C,     //!<
    LIS3DH_REGISTER_TIME_WINDOW = 0x3D,      //!<
    LIS3DH_REGISTER_ACT_THS = 0x3E,          //!<
    LIS3DH_REGISTER_INACT_DUR = 0x3F,        //!<
};

/**
 *
 */
class lis3dh {

   public:
    int setup(TwoWire *i2c_library, uint8_t i2c_address = 0x18);
    int setup(SPIClass *spi_library, const int spi_pin_cs, const int spi_speed = 10000000);
    bool detect(void);
    int i2c_disabled_force(void);
    int range_get(enum lis3dh_range *const range_g);
    int range_set(const enum lis3dh_range range_g);
    int sampling_get(enum lis3dh_sampling *const sampling_hz);
    int sampling_set(const enum lis3dh_sampling sampling_hz);
    int axis_enabled_set(const bool enabled_x, const bool enabled_y, const bool enabled_z);
    int axis_enabled_set(const enum lis3dh_axis axis, const bool enabled);
    int acceleration_read(const enum lis3dh_axis axis, int16_t *const accel);
    int acceleration_read(int16_t *const accel_x, int16_t *const accel_y, int16_t *const accel_z);
    int acceleration_read(const enum lis3dh_axis axis, float *const accel);
    int acceleration_read(float *const accel_x, float *const accel_y, float *const accel_z);
    int acceleration_read(double *const accel_x, double *const accel_y, double *const accel_z);
    int interrupts_polarity_set(const enum lis3dh_interrupt_polarity polarity);
    int activity_configure(const uint16_t threshold_mg, const uint32_t duration_ms);
    int activity_int2_routed_set(const bool routed);

   protected:
    TwoWire *m_i2c_library = NULL;
    uint8_t m_i2c_address = 0;
    SPIClass *m_spi_library = NULL;
    int m_spi_pin_cs = 0;
    int m_spi_speed;
    int m_register_read(const enum lis3dh_register reg_addr, uint8_t *const reg_value);
    int m_register_read(const enum lis3dh_register reg_addr, uint8_t *const reg_values, const size_t count);
    int m_register_write(const enum lis3dh_register reg_addr, const uint8_t reg_value);
    uint16_t m_activity_threshold = 0;
    uint32_t m_activity_duration = 0;
};

#endif
