// =========================================================================
// This library is placed under the MIT License
// Copyright 2017-2018 Natanael Josue Rabello. All rights reserved.
// For the license information refer to LICENSE file in root directory.
// =========================================================================

/**
 * @file MPU.cpp
 * Implement MPU class.
 */

#include "MPU.hpp"
#include <math.h>
#include <string.h>
// #include "MPU_err.h"
// #include "freertos/FreeRTOS.h"
// #include "freertos/portmacro.h"
// #include "freertos/task.h"
#include "mpu/math.hpp"
#include "mpu/registers.hpp"
#include "mpu/types.hpp"
#include "config_mpu_lib.h"

// static const char* TAG = CONFIG_MPU_CHIP_MODEL;

#include "mpu/log.hpp"

/*! MPU Driver namespace */
namespace mpud
{
/**
 * @brief Initialize MPU device and set basic configurations.
 * @details
 *  Init configuration:
 *  - Accel FSR: 4G
 *  - Gyro FSR: 500DPS
 *  - Sample rate: 100Hz
 *  - DLPF: 42Hz
 *  - INT pin: disabled
 *  - FIFO: disabled
 *  - Clock source: gyro PLL \n
 *  For MPU9150 and MPU9250:
 *  - Aux I2C Master: enabled, clock: 400KHz
 *  - Compass: enabled on Aux I2C's Slave 0 and Slave 1
 *
 * @note
 *  - A soft reset is performed first, which takes 100-200ms.
 *  - When using SPI, the primary I2C Slave module is disabled right away.
 * */
void MPU::initialize()
{
    // The default log_leval is DebugLogLevel::LVL_INFO
    // 0: NONE, 1: ERROR, 2: WARN, 3: INFO, 4: DEBUG, 5: TRACE
    switch (CONFIG_MPU_LOG_LEVEL) {
        case 0: LOG_SET_LEVEL(DebugLogLevel::LVL_NONE); break;
        case 1: LOG_SET_LEVEL(DebugLogLevel::LVL_ERROR); break;
        case 2: LOG_SET_LEVEL(DebugLogLevel::LVL_WARN); break;
        case 3: LOG_SET_LEVEL(DebugLogLevel::LVL_INFO); break;
        case 4: LOG_SET_LEVEL(DebugLogLevel::LVL_DEBUG); break;
        case 5: LOG_SET_LEVEL(DebugLogLevel::LVL_TRACE); break;
        default: LOG_SET_LEVEL(DebugLogLevel::LVL_WARN); break;
    }

    bus.begin();
    // reset device (wait a little to clear all registers)
    reset();
    delayMicroseconds(10);
    // wake-up the device (power on-reset state is asleep for some models)
    setSleep(false);
        // disable MPU's I2C slave module when using SPI
#ifdef CONFIG_MPU_SPI
    writeBit(regs::USER_CTRL, regs::USERCTRL_I2C_IF_DIS_BIT, 1);
#endif
    // set clock source to gyro PLL which is better than internal clock
    setClockSource(CLOCK_PLL);

#ifdef CONFIG_MPU6500
    // MPU6500 / MPU9250 share 4kB of memory between the DMP and the FIFO. Since the
    // first 3kB are needed by the DMP, we'll use the last 1kB for the FIFO.
    MPU_LOGI("CONFIG_MPU6500");
    writeBits(regs::ACCEL_CONFIG2, regs::ACONFIG2_FIFO_SIZE_BIT, regs::ACONFIG2_FIFO_SIZE_LENGTH,
                                FIFO_SIZE_1K);

#endif

    // set Full Scale range
    setGyroFullScale(GYRO_FS_500DPS);
    setAccelFullScale(ACCEL_FS_4G);
    // set Digital Low Pass Filter to get smoother data
    setDigitalLowPassFilter(DLPF_42HZ);

        // setup magnetometer
#ifdef CONFIG_MPU_AK89xx
    compassInit();
#ifdef CONFIG_MPU_AK8963
    delayMicroseconds(1);
    compassSetSensitivity(MAG_SENSITIVITY_0_15_uT);
#endif
#endif
    // set sample rate to 100Hz
    setSampleRate(100);
    MPU_LOGI("Initialization complete");
    
}

/**
 * @brief Reset internal registers and restore to default start-up state.
 * @note
 *  - This function delays 100ms when using I2C and 200ms when using SPI.
 *  - It does not initialize the MPU again, just call initialize() instead.
 * */
void MPU::reset()
{
    writeBit(regs::PWR_MGMT1, regs::PWR1_DEVICE_RESET_BIT, 1);
    delay(100);
#ifdef CONFIG_MPU_SPI
    resetSignalPath();
#endif
    MPU_LOGI("Reset!");
    
}

/**
 * @brief Enable / disable sleep mode
 * @param enable enable value
 * */
void MPU::setSleep(bool enable)
{
    writeBit(regs::PWR_MGMT1, regs::PWR1_SLEEP_BIT, (uint8_t) enable);
}

/**
 * @brief Get current sleep state.
 * @return
 *  - `true`: sleep enabled.
 *  - `false`: sleep disabled.
 */
bool MPU::getSleep()
{
    readBit(regs::PWR_MGMT1, regs::PWR1_SLEEP_BIT, buffer);
    return buffer[0];
}

/**
 * @brief Test connection with MPU.
 * @details It reads the WHO_AM_IM register and check its value against the correct chip model.
 * @return
 *  - `MPU_OK`: The MPU is connected and matchs the model.
 *  - `MPU_ERR_NOT_FOUND`: A device is connect, but does not match the chip selected in _menuconfig_.
 *  - May return other communication bus errors. e.g: `ESP_FAIL`, `intIMEOUT`.
 * */
mpu_err_t MPU::testConnection()
{
    const uint8_t wai = whoAmI();
#if defined CONFIG_MPU6000 || defined CONFIG_MPU6050 || defined CONFIG_MPU9150
    return (wai == 0x68) ? MPU_OK : MPU_ERR_NOT_FOUND;
#elif defined CONFIG_MPU9255
    return (wai == 0x73) ? MPU_OK : MPU_ERR_NOT_FOUND;
#elif defined CONFIG_MPU9250
    return (wai == 0x71) ? MPU_OK : MPU_ERR_NOT_FOUND;
#elif defined CONFIG_MPU6555
    return (wai == 0x7C) ? MPU_OK : MPU_ERR_NOT_FOUND;
#elif defined CONFIG_MPU6500    
    return (wai == 0x70) ? MPU_OK : MPU_ERR_NOT_FOUND;
#endif
}

/**
 * @brief Returns the value from WHO_AM_I register.
 */
uint8_t MPU::whoAmI()
{
    readByte(regs::WHO_AM_I, buffer);
    return buffer[0];
}

/**
 * @brief Set sample rate of data output.
 *
 * Sample rate controls sensor data output rate and FIFO sample rate.
 * This is the update rate of sensor register. \n
 * Formula: Sample Rate = Internal Output Rate / (1 + SMPLRT_DIV)
 *
 * @param rate 4Hz ~ 1KHz
 *  - For sample rate 8KHz: set digital low pass filter to DLPF_256HZ_NOLPF.
 *  - For sample rate 32KHZ [MPU6500 / MPU9250]: set fchoice to FCHOICE_0, see setFchoice().
 *
 * @note
 *  For MPU9150 & MPU9250:
 *   - When using compass, this function alters Aux I2C Master `sample_delay` property
 *     to adjust the compass sample rate. (also, `wait_for_es` property to adjust interrupt).
 *   - If sample rate lesser than 100 Hz, data-ready interrupt will wait for compass data.
 *   - If sample rate greater than 100 Hz, data-ready interrupt will not be delayed by the compass.
 * */
void MPU::setSampleRate(uint16_t rate)
{
    // Check value range
    if (rate < 4) {
        MPU_LOGW(msgs::INVALID_SAMPLE_RATE, rate, ", minimum rate is 4" );
        rate = 4;
    }
    else if (rate > 1000) {
        //MPU_LOGWMSG(msgs::INVALID_SAMPLE_RATE, " %d, maximum rate is 1000", rate);
        MPU_LOGW(msgs::INVALID_SAMPLE_RATE, rate, ", maximum rate is 1000" );

        rate = 1000;
    }

#if CONFIG_MPU_LOG_LEVEL >= MPU_LOG_WARN
        // Check selected Fchoice [MPU6500 and MPU9250 only]
#ifdef CONFIG_MPU6500
    fchoice_t fchoice = getFchoice();
    if (fchoice != FCHOICE_3) {
        MPU_LOGW(msgs::INVALID_STATE, ", sample rate divider is not effective when Fchoice != 3");
    }
#endif
    // Check dlpf configuration
    dlpf_t dlpf = getDigitalLowPassFilter();
    if (dlpf == 0 || dlpf == 7)
        MPU_LOGW(msgs::INVALID_STATE, ", sample rate divider is not effective when DLPF is (0 or 7)");
#endif

    constexpr uint16_t internalSampleRate = 1000;
    uint16_t divider                      = internalSampleRate / rate - 1;
    // Check for rate match
    uint16_t finalRate = (internalSampleRate / (1 + divider));
    if (finalRate != rate) {
        MPU_LOGW("Sample rate constrained to", finalRate, "Hz");
    }
    else {
        MPU_LOGI("Sample rate set to", finalRate, "Hz");
    }
    // Write divider to register
    writeByte(regs::SMPLRT_DIV, (uint8_t) divider);

        // check and set compass sample rate
#ifdef CONFIG_MPU_AK89xx
    const auxi2c_slv_config_t magSlaveChgModeConf = getAuxI2CSlaveConfig(MAG_SLAVE_CHG_MODE);
    const bool magSlaveChgModeEnabled = getAuxI2CSlaveEnabled(MAG_SLAVE_CHG_MODE);
    if (magSlaveChgModeEnabled && magSlaveChgModeConf.addr == COMPASS_I2CADDRESS &&
        (magSlaveChgModeConf.txdata & 0xF) == MAG_MODE_SINGLE_MEASURE) {
        auxi2c_config_t auxi2cConf = getAuxI2CConfig();
            if (rate <= COMPASS_SAMPLE_RATE_MAX) {
            auxi2cConf.wait_for_es  = 1;
            auxi2cConf.sample_delay = 0;
        }
        else {
            auxi2cConf.wait_for_es    = 0;
            auxi2cConf.sample_delay   = (uint8_t)(ceil(static_cast<double>(finalRate) / COMPASS_SAMPLE_RATE_MAX) - 1);
            const uint8_t compassRate = (finalRate / (auxi2cConf.sample_delay + 1);
            MPU_LOGW("Compass sample rate constrained to", compassRate, ", magnetometer's maximum is", COMPASS_SAMPLE_RATE_MAX, "Hz");
        }
        setAuxI2CConfig(auxi2cConf);
    }
#endif

    
}

/**
 * @brief Retrieve sample rate divider and calculate the actual rate.
 */
uint16_t MPU::getSampleRate()
{
#ifdef CONFIG_MPU6500
    fchoice_t fchoice = getFchoice();
    if (fchoice != FCHOICE_3) return SAMPLE_RATE_MAX;
#endif

    constexpr uint16_t sampleRateMax_nolpf = 8000;
    dlpf_t dlpf                            = getDigitalLowPassFilter();
    if (dlpf == 0 || dlpf == 7) return sampleRateMax_nolpf;

    constexpr uint16_t internalSampleRate = 1000;
    readByte(regs::SMPLRT_DIV, buffer);
    uint16_t rate = internalSampleRate / (1 + buffer[0]);
    return rate;
}

/**
 * @brief Select clock source.
 * @note The gyro PLL is better than internal clock.
 * @param clockSrc clock source
 */
void MPU::setClockSource(clock_src_t clockSrc)
{
    writeBits(regs::PWR_MGMT1, regs::PWR1_CLKSEL_BIT, regs::PWR1_CLKSEL_LENGTH, clockSrc);
}

/**
 * @brief Return clock source.
 */
clock_src_t MPU::getClockSource()
{
    readBits(regs::PWR_MGMT1, regs::PWR1_CLKSEL_BIT, regs::PWR1_CLKSEL_LENGTH, buffer);
    return (clock_src_t) buffer[0];
}

/**
 * @brief Configures Digital Low Pass Filter (DLPF) setting for both the gyroscope and accelerometer.
 * @param dlpf digital low-pass filter value
 */
void MPU::setDigitalLowPassFilter(dlpf_t dlpf)
{
    writeBits(regs::CONFIG, regs::CONFIG_DLPF_CFG_BIT, regs::CONFIG_DLPF_CFG_LENGTH, dlpf);
#ifdef CONFIG_MPU6500
    writeBits(regs::ACCEL_CONFIG2, regs::ACONFIG2_A_DLPF_CFG_BIT, regs::ACONFIG2_A_DLPF_CFG_LENGTH, dlpf);
#endif
    
}

/**
 * @brief Return Digital Low Pass Filter configuration
 */
dlpf_t MPU::getDigitalLowPassFilter()
{
    readBits(regs::CONFIG, regs::CONFIG_DLPF_CFG_BIT, regs::CONFIG_DLPF_CFG_LENGTH, buffer);
    return (dlpf_t) buffer[0];
}

/**
 * @brief Reset sensors signal path.
 *
 * Reset all gyro digital signal path, accel digital signal path, and temp
 * digital signal path. This also clears all the sensor registers.
 *
 * @note This function delays 100 ms, needed for reset to complete.
 * */
void MPU::resetSignalPath()
{
    writeBit(regs::USER_CTRL, regs::USERCTRL_SIG_COND_RESET_BIT, 1);
    delay(100);
    
}

/**
 * @brief Enter Low Power Accelerometer mode.
 *
 * In low-power accel mode, the chip goes to sleep and only wakes up to sample
 * the accelerometer at a certain frequency.
 * See setLowPowerAccelRate() to set the frequency.
 *
 * @param enable value
 *  + This function does the following to enable:
 *   - Set CYCLE bit to 1
 *   - Set SLEEP bit to 0
 *   - Set TEMP_DIS bit to 1
 *   - Set STBY_XG, STBY_YG, STBY_ZG bits to 1
 *   - Set STBY_XA, STBY_YA, STBY_ZA bits to 0
 *   - Set FCHOICE to 0 (ACCEL_FCHOICE_B bit to 1) [MPU6500 / MPU9250 only]
 *   - Disable Auxiliary I2C Master I/F
 *
 *  + This function does the following to disable:
 *   - Set CYCLE bit to 0
 *   - Set TEMP_DIS bit to 0
 *   - Set STBY_XG, STBY_YG, STBY_ZG bits to 0
 *   - Set STBY_XA, STBY_YA, STBY_ZA bits to 0
 *   - Set FCHOICE to 3 (ACCEL_FCHOICE_B bit to 0) [MPU6500 / MPU9250 only]
 *   - Enable Auxiliary I2C Master I/F
 * */
void MPU::setLowPowerAccelMode(bool enable)
{
// set FCHOICE
#ifdef CONFIG_MPU6500
    fchoice_t fchoice = enable ? FCHOICE_0 : FCHOICE_3;
    setFchoice(fchoice);
    MPU_LOGV("Fchoice set to", fchoice);
#endif
    // read PWR_MGMT1 and PWR_MGMT2 at once
    readBytes(regs::PWR_MGMT1, 2, buffer);
    if (enable) {
        // set CYCLE bit to 1 and SLEEP bit to 0 and TEMP_DIS bit to 1
        buffer[0] |= 1 << regs::PWR1_CYCLE_BIT;
        buffer[0] &= ~(1 << regs::PWR1_SLEEP_BIT);
        buffer[0] |= 1 << regs::PWR1_TEMP_DIS_BIT;
        // set STBY_XG, STBY_YG, STBY_ZG bits to 1
        buffer[1] |= regs::PWR2_STBY_XYZG_BITS;
    }
    else {  // disable
        // set CYCLE bit to 0 and TEMP_DIS bit to 0
        buffer[0] &= ~(1 << regs::PWR1_CYCLE_BIT);
        buffer[0] &= ~(1 << regs::PWR1_TEMP_DIS_BIT);
        // set STBY_XG, STBY_YG, STBY_ZG bits to 0
        buffer[1] &= ~(regs::PWR2_STBY_XYZG_BITS);
    }
    // set STBY_XA, STBY_YA, STBY_ZA bits to 0
    buffer[1] &= ~(regs::PWR2_STBY_XYZA_BITS);
    // write back PWR_MGMT1 and PWR_MGMT2 at once
    writeBytes(regs::PWR_MGMT1, 2, buffer);
    // disable Auxiliary I2C Master I/F in case it was active
    setAuxI2CEnabled(!enable);
    
}

/**
 * @brief Return Low Power Accelerometer state.
 *
 * Condition to return true:
 *  - CYCLE bit is 1
 *  - SLEEP bit is 0
 *  - TEMP_DIS bit is 1
 *  - STBY_XG, STBY_YG, STBY_ZG bits are 1
 *  - STBY_XA, STBY_YA, STBY_ZA bits are 0
 *  - FCHOICE is 0 (ACCEL_FCHOICE_B bit is 1) [MPU6500 / MPU9250 only]
 *
 * */
bool MPU::getLowPowerAccelMode()
{
// check FCHOICE
#ifdef CONFIG_MPU6500
    fchoice_t fchoice = getFchoice();
    if (fchoice != FCHOICE_0) {
        return false;
    }
#endif
    // read PWR_MGMT1 and PWR_MGMT2 at once
    readBytes(regs::PWR_MGMT1, 2, buffer);
    // define configuration bits
    constexpr uint8_t LPACCEL_CONFIG_BITMASK[2] = {
        (1 << regs::PWR1_SLEEP_BIT) | (1 << regs::PWR1_CYCLE_BIT) | (1 << regs::PWR1_TEMP_DIS_BIT),
        regs::PWR2_STBY_XYZA_BITS | regs::PWR2_STBY_XYZG_BITS};
    constexpr uint8_t LPACCEL_ENABLED_VALUE[2] = {(1 << regs::PWR1_CYCLE_BIT) | (1 << regs::PWR1_TEMP_DIS_BIT),
                                                  regs::PWR2_STBY_XYZG_BITS};
    // get just the configuration bits
    buffer[0] &= LPACCEL_CONFIG_BITMASK[0];
    buffer[1] &= LPACCEL_CONFIG_BITMASK[1];
    // check pattern
    return buffer[0] == LPACCEL_ENABLED_VALUE[0] && buffer[1] == LPACCEL_ENABLED_VALUE[1];
}

/**
 * @brief Set Low Power Accelerometer frequency of wake-up.
 * */
void MPU::setLowPowerAccelRate(lp_accel_rate_t rate)
{
#if defined CONFIG_MPU6050
    writeBits(regs::PWR_MGMT2, regs::PWR2_LP_WAKE_CTRL_BIT, regs::PWR2_LP_WAKE_CTRL_LENGTH, rate);
#elif defined CONFIG_MPU6500
    writeBits(regs::LP_ACCEL_ODR, regs::LPA_ODR_CLKSEL_BIT, regs::LPA_ODR_CLKSEL_LENGTH, rate);
#endif
}

/**
 * @brief Get Low Power Accelerometer frequency of wake-up.
 */
lp_accel_rate_t MPU::getLowPowerAccelRate()
{
#if defined CONFIG_MPU6050
    readBits(regs::PWR_MGMT2, regs::PWR2_LP_WAKE_CTRL_BIT, regs::PWR2_LP_WAKE_CTRL_LENGTH, buffer);
#elif defined CONFIG_MPU6500
    readBits(regs::LP_ACCEL_ODR, regs::LPA_ODR_CLKSEL_BIT, regs::LPA_ODR_CLKSEL_LENGTH, buffer);
#endif
    return (lp_accel_rate_t) buffer[0];
}

/**
 * @brief Enable/disable Motion modules (Motion detect, Zero-motion, Free-Fall).
 *
 * @attention
 *  The configurations must've already been set with setMotionDetectConfig() before
 *  enabling the module!
 * @note
 *  - Call getMotionDetectStatus() to find out which axis generated motion interrupt. [MPU6000, MPU6050, MPU9150].
 *  - It is recommended to set the Motion Interrupt to propagate to the INT pin. To do that, use setInterruptEnabled().
 * @param enable
 *  - On _true_, this function modifies the DLPF, put gyro and temperature in standby,
 *    and disable Auxiliary I2C Master I/F.
 *  - On _false_, this function sets DLPF to 42Hz and enables Auxiliary I2C master I/F.
 * */
void MPU::setMotionFeatureEnabled(bool enable)
{
#if defined CONFIG_MPU6050
    
            writeBits(regs::ACCEL_CONFIG, regs::ACONFIG_HPF_BIT, regs::ACONFIG_HPF_LENGTH, ACCEL_DHPF_RESET);
#endif
    /* enabling */
    if (enable) {
#if defined CONFIG_MPU6050
        constexpr dlpf_t kDLPF = DLPF_256HZ_NOLPF;
#elif defined CONFIG_MPU6500
        constexpr dlpf_t kDLPF = DLPF_188HZ;
#endif
        setDigitalLowPassFilter(kDLPF);
#if defined CONFIG_MPU6050
        // give a time for accumulation of samples
        delay(10);
        
        writeBits(regs::ACCEL_CONFIG, regs::ACONFIG_HPF_BIT, regs::ACONFIG_HPF_LENGTH, ACCEL_DHPF_HOLD);
#elif defined CONFIG_MPU6500
        writeByte(regs::ACCEL_INTEL_CTRL, (1 << regs::ACCEL_INTEL_EN_BIT) | (1 << regs::ACCEL_INTEL_MODE_BIT));
#endif
        /* disabling */
    }
    else {
#if defined CONFIG_MPU6500
        writeBits(regs::ACCEL_INTEL_CTRL, regs::ACCEL_INTEL_EN_BIT, 2, 0x0);
#endif
        constexpr dlpf_t kDLPF = DLPF_42HZ;
        setDigitalLowPassFilter(kDLPF);
    }
    // disable Auxiliary I2C Master I/F in case it was active
    setAuxI2CEnabled(!enable);
    
}

/**
 * @brief Return true if a Motion Dectection module is enabled.
 */
bool MPU::getMotionFeatureEnabled()
{
    uint8_t data;
#if defined CONFIG_MPU6050
    readBits(regs::ACCEL_CONFIG, regs::ACONFIG_HPF_BIT, regs::ACONFIG_HPF_LENGTH, &data);
    if (data != ACCEL_DHPF_HOLD) return false;
    constexpr dlpf_t kDLPF = DLPF_256HZ_NOLPF;
#elif defined CONFIG_MPU6500
    readByte(regs::ACCEL_INTEL_CTRL, &data);
    constexpr uint8_t kAccelIntel = (1 << regs::ACCEL_INTEL_EN_BIT) | (1 << regs::ACCEL_INTEL_MODE_BIT);
    if ((data & kAccelIntel) != kAccelIntel) return false;
    constexpr dlpf_t kDLPF = DLPF_188HZ;
#endif
    dlpf_t dlpf = getDigitalLowPassFilter();
    if (dlpf != kDLPF) return false;
    return true;
}

/**
 * @brief Configure Motion-Detect or Wake-on-motion feature.
 *
 * The behaviour of this feature is very different between the MPU6050 (MPU9150) and the
 * MPU6500 (MPU9250). Each chip's version of this feature is explained below.
 *
 * [MPU6050, MPU6000, MPU9150]:
 * Accelerometer measurements are passed through a configurable digital high pass filter (DHPF)
 * in order to eliminate bias due to gravity. A qualifying motion sample is one where the high
 * passed sample from any axis has an absolute value exceeding a user-programmable threshold. A
 * counter increments for each qualifying sample, and decrements for each non-qualifying sample.
 * Once the counter reaches a user-programmable counter threshold, a motion interrupt is triggered.
 * The axis and polarity which caused the interrupt to be triggered is flagged in the
 * MOT_DETECT_STATUS register.
 *
 * [MPU6500, MPU9250]:
 * Unlike the MPU6050 version, the hardware does not "lock in" a reference sample.
 * The hardware monitors the accel data and detects any large change over a short period of time.
 * A qualifying motion sample is one where the high passed sample from any axis has
 * an absolute value exceeding the threshold.
 * The hardware motion threshold can be between 4mg and 1020mg in 4mg increments.
 *
 * @note
 * It is possible to enable **wake-on-motion** mode by doing the following:
 *  1. Enter Low Power Accelerometer mode with setLowPowerAccelMode();
 *  2. Select the wake-up rate with setLowPowerAccelRate();
 *  3. Configure motion-detect interrupt with setMotionDetectConfig();
 *  4. Enable the motion detection module with setMotionFeatureEnabled();
 * */
void MPU::setMotionDetectConfig(mot_config_t& config)
{
#if defined CONFIG_MPU6050
    writeByte(regs::MOTION_DUR, config.time);
    writeBits(regs::MOTION_DETECT_CTRL, regs::MOTCTRL_ACCEL_ON_DELAY_BIT,
                                regs::MOTCTRL_ACCEL_ON_DELAY_LENGTH, config.accel_on_delay);
    writeBits(regs::MOTION_DETECT_CTRL, regs::MOTCTRL_MOT_COUNT_BIT, regs::MOTCTRL_MOT_COUNT_LENGTH,
                                config.counter);
#endif
    writeByte(regs::MOTION_THR, config.threshold);
}

/**
 * @brief Return Motion Detection Configuration.
 */
mot_config_t MPU::getMotionDetectConfig()
{
    mot_config_t config{};
#if defined CONFIG_MPU6050
    readByte(regs::MOTION_DUR, &config.time);
    readByte(regs::MOTION_DETECT_CTRL, buffer);
    config.accel_on_delay =
        (buffer[0] >> (regs::MOTCTRL_ACCEL_ON_DELAY_BIT - regs::MOTCTRL_ACCEL_ON_DELAY_LENGTH + 1)) & 0x3;
    config.counter =
        (mot_counter_t)((buffer[0] >> (regs::MOTCTRL_MOT_COUNT_BIT - regs::MOTCTRL_MOT_COUNT_LENGTH + 1)) & 0x3);
#endif
    readByte(regs::MOTION_THR, &config.threshold);
    return config;
}

#if defined CONFIG_MPU6050
/**
 * @brief Configure Zero-Motion.
 *
 * The Zero Motion detection capability uses the digital high pass filter (DHPF) and a similar
 * threshold scheme to that of Free Fall detection. Each axis of the high passed accelerometer
 * measurement must have an absolute value less than a threshold specified in the ZRMOT_THR
 * register, which can be increased in 1 mg increments. Each time a motion sample meets this
 * condition, a counter increments. When this counter reaches a threshold specified in ZRMOT_DUR, an
 * interrupt is generated.
 *
 * Unlike Free Fall or Motion detection, Zero Motion detection triggers an interrupt both when Zero
 * Motion is first detected and when Zero Motion is no longer detected. While Free Fall and Motion
 * are indicated with a flag which clears after being read, reading the state of the Zero Motion
 * detected from the MOT_DETECT_STATUS register does not clear its status.
 *
 * @note Enable by calling setMotionFeatureEnabled();
 * */
void MPU::setZeroMotionConfig(zrmot_config_t& config)
{
    buffer[0] = config.threshold;
    buffer[1] = config.time;
    writeBytes(regs::ZRMOTION_THR, 2, buffer);
}

/**
 * @brief Return Zero-Motion configuration.
 */
zrmot_config_t MPU::getZeroMotionConfig()
{
    readBytes(regs::ZRMOTION_THR, 2, buffer);
    zrmot_config_t config{};
    config.threshold = buffer[0];
    config.time      = buffer[1];
    return config;
}

/**
 * @brief Configure Free-Fall.
 *
 * Free fall is detected by checking if the accelerometer measurements from all 3 axes have an
 * absolute value below a user-programmable threshold (acceleration threshold). For each sample
 * where this condition is true (a qualifying sample), a counter is incremented. For each sample
 * where this condition is false (a non- qualifying sample), the counter is decremented. Once the
 * counter reaches a user-programmable threshold (the counter threshold), the Free Fall interrupt is
 * triggered and a flag is set. The flag is cleared once the counter has decremented to zero. The
 * counter does not increment above the counter threshold or decrement below zero.
 *
 * @note Enable by calling setMotionFeatureEnabled().
 * */
void MPU::setFreeFallConfig(ff_config_t& config)
{
    buffer[0] = config.threshold;
    buffer[1] = config.time;
    writeBytes(regs::FF_THR, 2, buffer);
    writeBits(regs::MOTION_DETECT_CTRL, regs::MOTCTRL_ACCEL_ON_DELAY_BIT,
                                regs::MOTCTRL_ACCEL_ON_DELAY_LENGTH, config.accel_on_delay);
    writeBits(regs::MOTION_DETECT_CTRL, regs::MOTCTRL_MOT_COUNT_BIT, regs::MOTCTRL_MOT_COUNT_LENGTH,
                                config.counter);
    
}

/**
 * @brief Return Free-Fall Configuration.
 */
ff_config_t MPU::getFreeFallConfig()
{
    ff_config_t config{};
    readBytes(regs::FF_THR, 2, buffer);
    config.threshold = buffer[0];
    config.time      = buffer[1];
    readByte(regs::MOTION_DETECT_CTRL, buffer);
    config.accel_on_delay =
        (buffer[0] >> (regs::MOTCTRL_ACCEL_ON_DELAY_BIT - regs::MOTCTRL_ACCEL_ON_DELAY_LENGTH + 1)) & 0x3;
    config.counter =
        (mot_counter_t)((buffer[0] >> (regs::MOTCTRL_MOT_COUNT_BIT - regs::MOTCTRL_MOT_COUNT_LENGTH + 1)) & 0x3);
    return config;
}

/**
 * @brief Return Motion Detection Status.
 * @note Reading this register clears all motion detection status bits.
 * */
mot_stat_t MPU::getMotionDetectStatus()
{
    readByte(regs::MOTION_DETECT_STATUS, buffer);
    return (mot_stat_t) buffer[0];
}
#endif  // MPU6050's stuff

/**
 * @brief Configure sensors' standby mode.
 * */
void MPU::setStandbyMode(stby_en_t mask)
{
    const uint8_t kPwr1StbyBits = mask >> 6;
    writeBits(regs::PWR_MGMT1, regs::PWR1_GYRO_STANDBY_BIT, 2, kPwr1StbyBits);
    writeBits(regs::PWR_MGMT2, regs::PWR2_STBY_XA_BIT, 6, mask);
}

/**
 * @brief Return Standby configuration.
 * */
stby_en_t MPU::getStandbyMode()
{
    readBytes(regs::PWR_MGMT1, 2, buffer);
    constexpr uint8_t kStbyTempAndGyroPLLBits = STBY_EN_TEMP | STBY_EN_LOWPWR_GYRO_PLL_ON;
    stby_en_t mask                            = buffer[0] << 3 & kStbyTempAndGyroPLLBits;
    constexpr uint8_t kStbyAccelAndGyroBits   = STBY_EN_ACCEL | STBY_EN_GYRO;
    mask |= buffer[1] & kStbyAccelAndGyroBits;
    return mask;
}

#ifdef CONFIG_MPU6500
/**
 * @brief Select FCHOICE.
 *
 * Dev note: FCHOICE is the inverted value of FCHOICE_B (e.g. FCHOICE=2b’00 is same as FCHOICE_B=2b’11).
 * Reset value is FCHOICE_3
 * */
void MPU::setFchoice(fchoice_t fchoice)
{
    buffer[0] = (~(fchoice) &0x3);  // invert to fchoice_b
    
            writeBits(regs::GYRO_CONFIG, regs::GCONFIG_FCHOICE_B, regs::GCONFIG_FCHOICE_B_LENGTH, buffer[0]);
    writeBit(regs::ACCEL_CONFIG2, regs::ACONFIG2_ACCEL_FCHOICE_B_BIT, (buffer[0] == 0) ? 0 : 1);
}

/**
 * @brief Return FCHOICE.
 */
fchoice_t MPU::getFchoice()
{
    readBits(regs::GYRO_CONFIG, regs::GCONFIG_FCHOICE_B, regs::GCONFIG_FCHOICE_B_LENGTH, buffer);
    return (fchoice_t)(~(buffer[0]) & 0x3);
}
#endif

/**
 * @brief Select Gyroscope Full-scale range.
 * */
void MPU::setGyroFullScale(gyro_fs_t fsr)
{
    writeBits(regs::GYRO_CONFIG, regs::GCONFIG_FS_SEL_BIT, regs::GCONFIG_FS_SEL_LENGTH, fsr);
}

/**
 * @brief Return Gyroscope Full-scale range.
 */
gyro_fs_t MPU::getGyroFullScale()
{
    readBits(regs::GYRO_CONFIG, regs::GCONFIG_FS_SEL_BIT, regs::GCONFIG_FS_SEL_LENGTH, buffer);
    return (gyro_fs_t) buffer[0];
}

/**
 * @brief Select Accelerometer Full-scale range.
 * */
void MPU::setAccelFullScale(accel_fs_t fsr)
{
    writeBits(regs::ACCEL_CONFIG, regs::ACONFIG_FS_SEL_BIT, regs::ACONFIG_FS_SEL_LENGTH, fsr);
}

/**
 * @brief Return Accelerometer Full-scale range.
 */
accel_fs_t MPU::getAccelFullScale()
{
    readBits(regs::ACCEL_CONFIG, regs::ACONFIG_FS_SEL_BIT, regs::ACONFIG_FS_SEL_LENGTH, buffer);
    return (accel_fs_t) buffer[0];
}

/**
 * @brief Push biases to the gyro offset registers.
 *
 * This function expects biases relative to the current sensor output, and
 * these biases will be added to the factory-supplied values.
 *
 * Note: Bias inputs are LSB in +-1000dps format.
 * */
void MPU::setGyroOffset(raw_axes_t bias)
{
    buffer[0] = (uint8_t)(bias.x >> 8);
    buffer[1] = (uint8_t)(bias.x);
    buffer[2] = (uint8_t)(bias.y >> 8);
    buffer[3] = (uint8_t)(bias.y);
    buffer[4] = (uint8_t)(bias.z >> 8);
    buffer[5] = (uint8_t)(bias.z);
    writeBytes(regs::XG_OFFSET_H, 6, buffer);
}

/**
 * @brief Return biases from the gyro offset registers.
 *
 * Note: Bias output are LSB in +-1000dps format.
 * */
raw_axes_t MPU::getGyroOffset()
{
    readBytes(regs::XG_OFFSET_H, 6, buffer);
    raw_axes_t bias;
    bias.x = (buffer[0] << 8) | buffer[1];
    bias.y = (buffer[2] << 8) | buffer[3];
    bias.z = (buffer[4] << 8) | buffer[5];
    return bias;
}

/**
 * @brief Push biases to the accel offset registers.
 *
 * This function expects biases relative to the current sensor output, and
 * these biases will be added to the factory-supplied values.
 *
 * Note: Bias inputs are LSB in +-16G format.
 * */
void MPU::setAccelOffset(raw_axes_t bias)
{
    raw_axes_t facBias;
    // first, read OTP values of Accel factory trim

#if defined CONFIG_MPU6050
    readBytes(regs::XA_OFFSET_H, 6, buffer);
    facBias.x = (buffer[0] << 8) | buffer[1];
    facBias.y = (buffer[2] << 8) | buffer[3];
    facBias.z = (buffer[4] << 8) | buffer[5];

#elif defined CONFIG_MPU6500
    readBytes(regs::XA_OFFSET_H, 8, buffer);
    // note: buffer[2] and buffer[5], stay the same,
    //  they are read just to keep the burst reading
    facBias.x = (buffer[0] << 8) | buffer[1];
    facBias.y = (buffer[3] << 8) | buffer[4];
    facBias.z = (buffer[6] << 8) | buffer[7];
#endif

    // note: preserve bit 0 of factory value (for temperature compensation)
    facBias.x += (bias.x & ~1);
    facBias.y += (bias.y & ~1);
    facBias.z += (bias.z & ~1);

#if defined CONFIG_MPU6050
    buffer[0] = (uint8_t)(facBias.x >> 8);
    buffer[1] = (uint8_t)(facBias.x);
    buffer[2] = (uint8_t)(facBias.y >> 8);
    buffer[3] = (uint8_t)(facBias.y);
    buffer[4] = (uint8_t)(facBias.z >> 8);
    buffer[5] = (uint8_t)(facBias.z);
    writeBytes(regs::XA_OFFSET_H, 6, buffer);

#elif defined CONFIG_MPU6500
    buffer[0] = (uint8_t)(facBias.x >> 8);
    buffer[1] = (uint8_t)(facBias.x);
    buffer[3] = (uint8_t)(facBias.y >> 8);
    buffer[4] = (uint8_t)(facBias.y);
    buffer[6] = (uint8_t)(facBias.z >> 8);
    buffer[7] = (uint8_t)(facBias.z);
    writeBytes(regs::XA_OFFSET_H, 8, buffer);
#endif

    
}

/**
 * @brief Return biases from accel offset registers.
 * This returns the biases with OTP values from factory trim added,
 * so returned values will be different than that ones set with setAccelOffset().
 *
 * Note: Bias output are LSB in +-16G format.
 * */
raw_axes_t MPU::getAccelOffset()
{
    raw_axes_t bias;

#if defined CONFIG_MPU6050
    readBytes(regs::XA_OFFSET_H, 6, buffer);
    bias.x = (buffer[0] << 8) | buffer[1];
    bias.y = (buffer[2] << 8) | buffer[3];
    bias.z = (buffer[4] << 8) | buffer[5];

#elif defined CONFIG_MPU6500
    readBytes(regs::XA_OFFSET_H, 8, buffer);
    bias.x                        = (buffer[0] << 8) | buffer[1];
    bias.y                        = (buffer[3] << 8) | buffer[4];
    bias.z                        = (buffer[6] << 8) | buffer[7];
#endif

    return bias;
}

/**
 * @brief Compute Accelerometer and Gyroscope offsets.
 *
 * This takes about ~400ms to compute offsets.
 * When calculating the offsets the MPU must remain as horizontal as possible (0 degrees), facing
 * up. It is better to call computeOffsets() before any configuration is done (better right after
 * initialize()).
 *
 * Note: Gyro offset output are LSB in 1000DPS format.
 * Note: Accel offset output are LSB in 16G format.
 * */
void MPU::computeOffsets(raw_axes_t* accel, raw_axes_t* gyro)
{
    constexpr accel_fs_t kAccelFS = ACCEL_FS_2G;     // most sensitive
    constexpr gyro_fs_t kGyroFS   = GYRO_FS_250DPS;  // most sensitive
    getBiases(kAccelFS, kGyroFS, accel, gyro, false);
    // convert offsets to 16G and 1000DPS format and invert values
    for (int i = 0; i < 3; i++) {
        (*accel)[i] = -((*accel)[i] >> (types::ACCEL_FS_16G - kAccelFS));
        (*gyro)[i]  = -((*gyro)[i] >> (types::GYRO_FS_1000DPS - kGyroFS));
    }
    
}

/**
 * @brief Read accelerometer raw data.
 * */
void MPU::acceleration(raw_axes_t* accel)
{
    readBytes(regs::ACCEL_XOUT_H, 6, buffer);
    accel->x = buffer[0] << 8 | buffer[1];
    accel->y = buffer[2] << 8 | buffer[3];
    accel->z = buffer[4] << 8 | buffer[5];
    
}

/**
 * @brief Read accelerometer raw data.
 * */
void MPU::acceleration(int16_t* x, int16_t* y, int16_t* z)
{
    readBytes(regs::ACCEL_XOUT_H, 6, buffer);
    *x = buffer[0] << 8 | buffer[1];
    *y = buffer[2] << 8 | buffer[3];
    *z = buffer[4] << 8 | buffer[5];
    
}

/**
 * @brief Read gyroscope raw data.
 * */
void MPU::rotation(raw_axes_t* gyro)
{
    readBytes(regs::GYRO_XOUT_H, 6, buffer);
    gyro->x = buffer[0] << 8 | buffer[1];
    gyro->y = buffer[2] << 8 | buffer[3];
    gyro->z = buffer[4] << 8 | buffer[5];
    
}

/**
 * @brief Read gyroscope raw data.
 * */
void MPU::rotation(int16_t* x, int16_t* y, int16_t* z)
{
    readBytes(regs::GYRO_XOUT_H, 6, buffer);
    *x = buffer[0] << 8 | buffer[1];
    *y = buffer[2] << 8 | buffer[3];
    *z = buffer[4] << 8 | buffer[5];
    
}

/**
 * Read temperature raw data.
 * */
void MPU::temperature(int16_t* temp)
{
    readBytes(regs::TEMP_OUT_H, 2, buffer);
    *temp = buffer[0] << 8 | buffer[1];
    
}

/**
 * @brief Read accelerometer and gyroscope data at once.
 * */
void MPU::motion(raw_axes_t* accel, raw_axes_t* gyro)
{
    readBytes(regs::ACCEL_XOUT_H, 14, buffer);
    accel->x = buffer[0] << 8 | buffer[1];
    accel->y = buffer[2] << 8 | buffer[3];
    accel->z = buffer[4] << 8 | buffer[5];
    gyro->x  = buffer[8] << 8 | buffer[9];
    gyro->y  = buffer[10] << 8 | buffer[11];
    gyro->z  = buffer[12] << 8 | buffer[13];
    
}

#if defined CONFIG_MPU_AK89xx
/**
 * @brief Read compass data.
 * */
void MPU::heading(raw_axes_t* mag)
{
    readBytes(regs::EXT_SENS_DATA_01, 6, buffer);
    mag->x = buffer[1] << 8 | buffer[0];
    mag->y = buffer[3] << 8 | buffer[2];
    mag->z = buffer[5] << 8 | buffer[4];
    
}

/**
 * @brief Read compass data.
 * */
void MPU::heading(int16_t* x, int16_t* y, int16_t* z)
{
    readBytes(regs::EXT_SENS_DATA_01, 6, buffer);
    *x = buffer[1] << 8 | buffer[0];
    *y = buffer[3] << 8 | buffer[2];
    *z = buffer[5] << 8 | buffer[4];
    
}

/**
 * @brief Read accelerometer, gyroscope, compass raw data.
 * */
void MPU::motion(raw_axes_t* accel, raw_axes_t* gyro, raw_axes_t* mag)
{
    uint8_t buffer[22];
    readBytes(regs::ACCEL_XOUT_H, 22, buffer);
    accel->x = buffer[0] << 8 | buffer[1];
    accel->y = buffer[2] << 8 | buffer[3];
    accel->z = buffer[4] << 8 | buffer[5];
    gyro->x  = buffer[8] << 8 | buffer[9];
    gyro->y  = buffer[10] << 8 | buffer[11];
    gyro->z  = buffer[12] << 8 | buffer[13];
    mag->x   = buffer[16] << 8 | buffer[15];
    mag->y   = buffer[18] << 8 | buffer[17];
    mag->z   = buffer[20] << 8 | buffer[19];
    
}
#endif  // AK89xx

/**
 * @brief Read data from all internal sensors.
 * */
void MPU::sensors(raw_axes_t* accel, raw_axes_t* gyro, int16_t* temp)
{
    readBytes(regs::ACCEL_XOUT_H, 14, buffer);
    accel->x = buffer[0] << 8 | buffer[1];
    accel->y = buffer[2] << 8 | buffer[3];
    accel->z = buffer[4] << 8 | buffer[5];
    *temp    = buffer[6] << 8 | buffer[7];
    gyro->x  = buffer[8] << 8 | buffer[9];
    gyro->y  = buffer[10] << 8 | buffer[11];
    gyro->z  = buffer[12] << 8 | buffer[13];
    
}

/**
 * @brief Read data from all sensors, including external sensors in Aux I2C.
 * */
void MPU::sensors(sensors_t* sensors, size_t extsens_len)
{
    constexpr size_t kIntSensLenMax = 14;  // internal sensors data length max
    constexpr size_t kExtSensLenMax = 24;  // external sensors data length max
    uint8_t buffer[kIntSensLenMax + kExtSensLenMax];
#if defined CONFIG_MPU_AK89xx
    constexpr size_t kMagLen = 8;  // magnetometer data length
    const size_t length      = kIntSensLenMax + extsens_len + kMagLen;
#else
    const size_t length           = kIntSensLenMax + extsens_len;
#endif
    readBytes(regs::ACCEL_XOUT_H, length, buffer);
    sensors->accel.x = buffer[0] << 8 | buffer[1];
    sensors->accel.y = buffer[2] << 8 | buffer[3];
    sensors->accel.z = buffer[4] << 8 | buffer[5];
    sensors->temp    = buffer[6] << 8 | buffer[7];
    sensors->gyro.x  = buffer[8] << 8 | buffer[9];
    sensors->gyro.y  = buffer[10] << 8 | buffer[11];
    sensors->gyro.z  = buffer[12] << 8 | buffer[13];
#if CONFIG_MPU_AK89xx
    sensors->mag.x = buffer[16] << 8 | buffer[15];
    sensors->mag.y = buffer[18] << 8 | buffer[17];
    sensors->mag.z = buffer[20] << 8 | buffer[19];
#endif
    memcpy(sensors->extsens, buffer + (length - extsens_len), extsens_len);
    
}

#if defined CONFIG_MPU9150 || (defined CONFIG_MPU6050 && !defined CONFIG_MPU6000)
/**
 * @brief The MPU-6050’s I/O logic levels are set to be either VDD or VLOGIC.
 *
 * VLOGIC may be set to be equal to VDD or to another voltage. However, VLOGIC must be ≤ VDD at all
 * times. When AUX_VDDIO is set to 0 (its power-on-reset value), VLOGIC is the power supply voltage
 * for both the microprocessor system bus and the auxiliary I C bus. When AUX_VDDIO is set to 1,
 * VLOGIC is the power supply voltage for the microprocessor system bus and VDD is the supply for
 * the auxiliary I2C bus
 * */
void MPU::setAuxVDDIOLevel(auxvddio_lvl_t level)
{
    writeBit(regs::YG_OTP_OFFSET_TC, regs::TC_PWR_MODE_BIT, level);
}

/**
 * Return MPU-6050’s I/O logic levels.
 */
auxvddio_lvl_t MPU::getAuxVDDIOLevel()
{
    readBit(regs::YG_OTP_OFFSET_TC, regs::TC_PWR_MODE_BIT, buffer);
    return (auxvddio_lvl_t) buffer[0];
}
#endif

/**
 * @brief Configure the Interrupt pin (INT).
 * @param config configuration desired.
 */
void MPU::setInterruptConfig(int_config_t config)
{
    readByte(regs::INT_PIN_CONFIG, buffer);
    // zero the bits we're setting, but keep the others we're not setting as they are;
    constexpr uint8_t INT_PIN_CONFIG_BITMASK = (1 << regs::INT_CFG_LEVEL_BIT) | (1 << regs::INT_CFG_OPEN_BIT) |
                                               (1 << regs::INT_CFG_LATCH_EN_BIT) |
                                               (1 << regs::INT_CFG_ANYRD_2CLEAR_BIT);
    buffer[0] &= ~INT_PIN_CONFIG_BITMASK;
    // set the configurations
    buffer[0] |= config.level << regs::INT_CFG_LEVEL_BIT;
    buffer[0] |= config.drive << regs::INT_CFG_OPEN_BIT;
    buffer[0] |= config.mode << regs::INT_CFG_LATCH_EN_BIT;
    buffer[0] |= config.clear << regs::INT_CFG_ANYRD_2CLEAR_BIT;
    writeByte(regs::INT_PIN_CONFIG, buffer[0]);
}

/**
 * @brief Return Interrupt pin (INT) configuration.
 */
int_config_t MPU::getInterruptConfig()
{
    readByte(regs::INT_PIN_CONFIG, buffer);
    int_config_t config{};
    config.level = (int_lvl_t)((buffer[0] >> regs::INT_CFG_LEVEL_BIT) & 0x1);
    config.drive = (int_drive_t)((buffer[0] >> regs::INT_CFG_OPEN_BIT) & 0x1);
    config.mode  = (int_mode_t)((buffer[0] >> regs::INT_CFG_LATCH_EN_BIT) & 0x1);
    config.clear = (int_clear_t)((buffer[0] >> regs::INT_CFG_ANYRD_2CLEAR_BIT) & 0x1);
    return config;
}

/**
 * @brief Enable features to generate signal at Interrupt pin
 * @param mask ORed features.
 */
void MPU::setInterruptEnabled(int_en_t mask)
{
    writeByte(regs::INT_ENABLE, mask);
}

/**
 * @brief Return enabled features configured to generate signal at Interrupt pin.
 */
int_en_t MPU::getInterruptEnabled()
{
    readByte(regs::INT_ENABLE, buffer);
    return (int_en_t) buffer[0];
}

/**
 * @brief Return the Interrupt status from INT_STATUS register.
 *
 * Note: Reading this register, clear all bits.
 */
int_stat_t MPU::getInterruptStatus()
{
    readByte(regs::INT_STATUS, buffer);
    return (int_stat_t) buffer[0];
}

/**
 * @brief Change FIFO mode.
 *
 * Options:
 * `FIFO_MODE_OVERWRITE`: When the fifo is full, additional writes will be
 *  written to the fifo,replacing the oldest data.
 * `FIFO_MODE_STOP_FULL`: When the fifo is full, additional writes will not be written to fifo.
 * */
void MPU::setFIFOMode(fifo_mode_t mode)
{
    writeBit(regs::CONFIG, regs::CONFIG_FIFO_MODE_BIT, mode);
}

/**
 * @brief Return FIFO mode.
 */
fifo_mode_t MPU::getFIFOMode()
{
    readBit(regs::CONFIG, regs::CONFIG_FIFO_MODE_BIT, buffer);
    return (fifo_mode_t) buffer[0];
}

/**
 * @brief Configure the sensors that will be written to the FIFO.
 * */
void MPU::setFIFOConfig(fifo_config_t config)
{
    writeByte(regs::FIFO_EN, (uint8_t) config);
    writeBit(regs::I2C_MST_CTRL, regs::I2CMST_CTRL_SLV_3_FIFO_EN_BIT, config >> 8);
}

/**
 * @brief Return FIFO configuration.
 */
fifo_config_t MPU::getFIFOConfig()
{
    readBytes(regs::FIFO_EN, 2, buffer);
    fifo_config_t config = buffer[0];
    config |= (buffer[1] & (1 << mpud::regs::I2CMST_CTRL_SLV_3_FIFO_EN_BIT)) << 3;
    return config;
}

/**
 * @brief Enabled / disable FIFO module.
 * */
void MPU::setFIFOEnabled(bool enable)
{
    writeBit(regs::USER_CTRL, regs::USERCTRL_FIFO_EN_BIT, (uint8_t) enable);
}

/**
 * @brief Return FIFO module state.
 */
bool MPU::getFIFOEnabled()
{
    readBit(regs::USER_CTRL, regs::USERCTRL_FIFO_EN_BIT, buffer);
    return buffer[0];
}

/**
 * @brief Reset FIFO module.
 *
 * Zero FIFO count, reset is asynchronous. \n
 * The bit auto clears after one clock cycle.
 * */
void MPU::resetFIFO()
{
    writeBit(regs::USER_CTRL, regs::USERCTRL_FIFO_RESET_BIT, 1);
}

/**
 * @brief Return number of written bytes in the FIFO.
 * @note FIFO overflow generates an interrupt which can be check with getInterruptStatus().
 * */
uint16_t MPU::getFIFOCount()
{
    readBytes(regs::FIFO_COUNT_H, 2, buffer);
    uint16_t count = buffer[0] << 8 | buffer[1];
    return count;
}

/**
 * @brief Read data contained in FIFO buffer.
 * */
void MPU::readFIFO(size_t length, uint8_t* data)
{
    return readBytes(regs::FIFO_R_W, length, data);
}

/**
 * @brief Write data to FIFO buffer.
 * */
void MPU::writeFIFO(size_t length, uint8_t* data)
{
    writeBytes(regs::FIFO_R_W, length, data);
}

/**
 * @brief Configure the Auxiliary I2C Master.
 * @note For [MPU9150, MPU9250]: The Auxiliary I2C is configured in the initialization stage
 *  to connect with the compass in Slave 0 and Slave 1.
 * */
void MPU::setAuxI2CConfig(const auxi2c_config_t& config)
{
    // TODO: check compass enabled, to constrain sample_delay which defines the compass read sample
    // rate
    readBit(regs::I2C_MST_CTRL, regs::I2CMST_CTRL_SLV_3_FIFO_EN_BIT, buffer);
    buffer[0] <<= regs::I2CMST_CTRL_SLV_3_FIFO_EN_BIT;
    buffer[0] |= config.multi_master_en << regs::I2CMST_CTRL_MULT_EN_BIT;
    buffer[0] |= config.wait_for_es << regs::I2CMST_CTRL_WAIT_FOR_ES_BIT;
    buffer[0] |= config.transition << regs::I2CMST_CTRL_P_NSR_BIT;
    buffer[0] |= config.clock;
    writeByte(regs::I2C_MST_CTRL, buffer[0]);
    writeBits(regs::I2C_SLV4_CTRL, regs::I2C_SLV4_MST_DELAY_BIT, regs::I2C_SLV4_MST_DELAY_LENGTH,
                                config.sample_delay);
    writeBit(regs::I2C_MST_DELAY_CRTL, regs::I2CMST_DLY_ES_SHADOW_BIT, config.shadow_delay_en);
    MPU_LOGV(msgs::EMPTY,
                "Master:: multi_master_en:",config.multi_master_en, ", wait_for_es:", config.wait_for_es,
                ", transition:", config.transition,", clock:", config.clock,
                ", sample_delay:", config.sample_delay,", shadow_delay_en:", config.shadow_delay_en);
    
}

/**
 * @brief Get Auxiliary I2C Master configuration.
 */
auxi2c_config_t MPU::getAuxI2CConfig()
{
    readByte(regs::I2C_MST_CTRL, buffer);
    auxi2c_config_t config{};
    config.multi_master_en = buffer[0] >> regs::I2CMST_CTRL_MULT_EN_BIT;
    config.wait_for_es     = (buffer[0] >> regs::I2CMST_CTRL_WAIT_FOR_ES_BIT) & 0x1;
    config.transition      = (auxi2c_trans_t)((buffer[0] >> regs::I2CMST_CTRL_P_NSR_BIT) & 0x1);
    config.clock           = (auxi2c_clock_t)(buffer[0] & ((1 << regs::I2CMST_CTRL_CLOCK_LENGTH) - 1));
    
        readBits(regs::I2C_SLV4_CTRL, regs::I2C_SLV4_MST_DELAY_BIT, regs::I2C_SLV4_MST_DELAY_LENGTH, buffer + 1);
    config.sample_delay = buffer[1];
    readBit(regs::I2C_MST_DELAY_CRTL, regs::I2CMST_DLY_ES_SHADOW_BIT, buffer + 2);
    config.shadow_delay_en = buffer[2];
    return config;
}

/**
 * @brief Enable / disable Auxiliary I2C Master module.
 * */
void MPU::setAuxI2CEnabled(bool enable)
{
    writeBit(regs::USER_CTRL, regs::USERCTRL_I2C_MST_EN_BIT, (uint8_t) enable);
    if (enable) {
        writeBit(regs::INT_PIN_CONFIG, regs::INT_CFG_I2C_BYPASS_EN_BIT, 0);
    }
    
}

/**
 * @brief Return Auxiliary I2C Master state.
 */
bool MPU::getAuxI2CEnabled()
{
    readBit(regs::USER_CTRL, regs::USERCTRL_I2C_MST_EN_BIT, buffer);
    readBit(regs::INT_PIN_CONFIG, regs::INT_CFG_I2C_BYPASS_EN_BIT, buffer + 1);
    return buffer[0] && (!buffer[1]);
}

/**
 * @brief Configure communication with a Slave connected to Auxiliary I2C bus.
 * */
void MPU::setAuxI2CSlaveConfig(const auxi2c_slv_config_t& config)
{
    // slaves' config registers are grouped as 3 regs in a row
    const uint8_t regAddr = config.slave * 3 + regs::I2C_SLV0_ADDR;
    // data for regs::I2C_SLVx_ADDR
    buffer[0] = config.rw << regs::I2C_SLV_RNW_BIT;
    buffer[0] |= config.addr;
    // data for regs::I2C_SLVx_REG
    buffer[1] = config.reg_addr;
    // data for regs::I2C_SLVx_CTRL
    readByte(regAddr + 2, buffer + 2);
    if (config.rw == AUXI2C_READ) {
        buffer[2] &= 1 << regs::I2C_SLV_EN_BIT;  // keep enable bit, clear the rest
        buffer[2] |= config.reg_dis << regs::I2C_SLV_REG_DIS_BIT;
        buffer[2] |= config.swap_en << regs::I2C_SLV_BYTE_SW_BIT;
        buffer[2] |= config.end_of_word << regs::I2C_SLV_GRP_BIT;
        buffer[2] |= config.rxlength & 0xF;
    }
    else {                                                     // AUXI2C_WRITE
        buffer[2] &= ~(1 << regs::I2C_SLV_REG_DIS_BIT | 0xF);  // clear length bits and register disable bit
        buffer[2] |= config.reg_dis << regs::I2C_SLV_REG_DIS_BIT;
        buffer[2] |= 0x1;  // set length to write 1 byte
        writeByte(regs::I2C_SLV0_DO + config.slave, config.txdata);
    }
    writeBytes(regAddr, 3, buffer);
    // sample_delay enable/disable
    writeBit(regs::I2C_MST_DELAY_CRTL, config.slave, config.sample_delay_en);
    MPU_LOGV("Slave",config.slave,":: r/w:", (config.rw == AUXI2C_READ ? "read" : "write"), 
            ", addr: 0x", DebugLogBase::HEX,  config.addr, 
            ", reg_addr: 0x", DebugLogBase::HEX, config.reg_addr,
            ", reg_dis: ,", config.reg_dis, (config.rw == AUXI2C_READ ? "rxlength" : "txdata"), ": 0x", DebugLogBase::HEX, config.txdata,
            ", sample_delay_en:",config.sample_delay_en);
    
}

/**
 * @brief Return configuration of a Aux I2C Slave.
 * @param slave slave number.
 */
auxi2c_slv_config_t MPU::getAuxI2CSlaveConfig(auxi2c_slv_t slave)
{
    auxi2c_slv_config_t config;
    const uint8_t regAddr = slave * 3 + regs::I2C_SLV0_ADDR;
    config.slave          = slave;
    readBytes(regAddr, 3, buffer);
    config.rw       = (auxi2c_rw_t)((buffer[0] >> regs::I2C_SLV_RNW_BIT) & 0x1);
    config.addr     = buffer[0] & 0x7F;
    config.reg_addr = buffer[1];
    config.reg_dis  = (buffer[2] >> regs::I2C_SLV_REG_DIS_BIT) & 0x1;
    if (config.rw == AUXI2C_READ) {
        config.swap_en     = (buffer[2] >> regs::I2C_SLV_BYTE_SW_BIT) & 0x1;
        config.end_of_word = (auxi2c_eow_t)((buffer[2] >> regs::I2C_SLV_GRP_BIT) & 0x1);
        config.rxlength    = buffer[2] & 0xF;
    }
    else {
        readByte(regs::I2C_SLV0_DO + slave, buffer + 3);
        config.txdata = buffer[3];
    }
    readByte(regs::I2C_MST_DELAY_CRTL, buffer + 4);
    config.sample_delay_en = (buffer[4] >> slave) & 0x1;
    return config;
}

/**
 * @brief Enable the Auxiliary I2C module to transfer data with a slave at sample rate.
 * */
void MPU::setAuxI2CSlaveEnabled(auxi2c_slv_t slave, bool enable)
{
    const uint8_t regAddr = slave * 3 + regs::I2C_SLV0_CTRL;
    writeBit(regAddr, regs::I2C_SLV_EN_BIT, enable);
}

/**
 * @brief Return enable state of a Aux I2C's Slave.
 */
bool MPU::getAuxI2CSlaveEnabled(auxi2c_slv_t slave)
{
    const uint8_t regAddr = slave * 3 + regs::I2C_SLV0_CTRL;
    readBit(regAddr, regs::I2C_SLV_EN_BIT, buffer);
    return buffer[0];
}

/**
 * @brief Enable / disable Auxiliary I2C bypass mode.
 * @param enable
 *  - `true`: Auxiliar I2C Master I/F is disabled, and Bypass enabled.
 *  - `false`: Bypass is disabled, but the Auxiliar I2C Master I/F is not enabled back,
 *             if needed, enabled it again with setAuxI2CmasterEnabled().
 * */
void MPU::setAuxI2CBypass(bool enable)
{
#ifdef CONFIG_MPU_SPI
    if (enable) {
        MPU_LOGW("Setting Aux I2C to bypass mode while MPU is connected via SPI");
    }
#endif
    if (enable) {
        writeBit(regs::USER_CTRL, regs::USERCTRL_I2C_MST_EN_BIT, 0);
    }
    writeBit(regs::INT_PIN_CONFIG, regs::INT_CFG_I2C_BYPASS_EN_BIT, enable);
    if (enable) {
        MPU_LOGV(msgs::AUX_I2C_DISABLED, ", entering Bypass mode.");
    }
    else {
        MPU_LOGV("Leaving Bypass mode.");
    }
    
}

/**
 * @brief Return Auxiliary I2C Master bypass mode state.
 */
bool MPU::getAuxI2CBypass()
{
    readBit(regs::USER_CTRL, regs::USERCTRL_I2C_MST_EN_BIT, buffer);
    readBit(regs::INT_PIN_CONFIG, regs::INT_CFG_I2C_BYPASS_EN_BIT, buffer + 1);
    return (!buffer[0]) && buffer[1];
}

/**
 * @brief Read data from slaves connected to Auxiliar I2C bus.
 *
 * Data is placed in these external sensor data registers according to I2C_SLV0_CTRL,
 * I2C_SLV1_CTRL, I2C_SLV2_CTRL, and I2C_SLV3_CTRL (Registers 39, 42, 45, and 48). When
 * more than zero bytes are read (I2C_SLVx_LEN > 0) from an enabled slave (I2C_SLVx_EN = 1), the
 * slave is read at the Sample Rate (as defined in Register 25) or delayed rate (if specified in
 * Register 52 and 103). During each sample cycle, slave reads are performed in order of Slave
 * number. If all slaves are enabled with more than zero bytes to be read, the order will be Slave
 * 0, followed by Slave 1, Slave 2, and Slave 3.
 *
 * If the sum of the read lengths of all SLVx transactions exceed the number of available
 * EXT_SENS_DATA registers, the excess bytes will be dropped. There are 24 EXT_SENS_DATA
 * registers and hence the total read lengths between all the slaves cannot be greater than 24 or
 * some bytes will be lost.
 *
 * @attention Set `skip` to `8` when using compass, because compass data takes up the first `8` bytes.
 * */
void MPU::readAuxI2CRxData(size_t length, uint8_t* data, size_t skip)
{
    if (length + skip > 24) {
        MPU_LOGE(msgs::INVALID_LENGTH, length, ", mpu has only 24 external sensor data registers!");
        // MPU_ERR_INVALID_SIZE;
    }
// check if I2C Master is enabled, just for warning and debug
#if CONFIG_MPU_LOG_LEVEL >= MPU_LOG_WARN
    const bool kAuxI2CEnabled = getAuxI2CEnabled();
    if (!kAuxI2CEnabled) MPU_LOGW(msgs::AUX_I2C_DISABLED, ", better turn on.");
#endif
    // read the specified amount of registers
    return readBytes(regs::EXT_SENS_DATA_00 + skip, length, data);
}

/**
 * @brief Restart Auxiliary I2C Master module, reset is asynchronous.
 *
 * This bit (I2C_MST_RST) should only be set when the I2C master has hung. If this bit
 * is set during an active I2C master transaction, the I2C slave will hang, which
 * will require the host to reset the slave.
 * */
void MPU::restartAuxI2C()
{
    writeBit(regs::USER_CTRL, regs::USERCTRL_I2C_MST_RESET_BIT, 1);
}

/**
 * @brief Return Auxiliary I2C Master status from register I2C_MST_STATUS.
 * Reading this register clear all its bits.
 * */
auxi2c_stat_t MPU::getAuxI2CStatus()
{
    readByte(regs::I2C_MST_STATUS, buffer);
    return (auxi2c_stat_t) buffer[0];
}

/**
 * @brief Write to a slave a single byte just once (use for configuring a slave at initialization).
 *
 * This function uses Slave 4 to perform single transfers to the slave device on Aux I2C. \n
 * The byte will be transfered at first sample take, so when sample rate is at minimum (4 Hz)
 * it may take up to a quarter of a second to start the transfer.
 * @attention Auxiliary I2C Master must have already been configured before calling this function.
 *
 * @return
 *  - `MPU_ERR_INVALID_STATE`: Auxiliary I2C Master not enabled;
 *  - `MPU_ERR_NOT_FOUND`:     Slave doesn't ACK the transfer;
 *  - `MPU_FAIL`:               Auxiliary I2C Master lost arbitration of the bus;
 *  - or other standard I2C driver error codes.
 * */
void MPU::auxI2CWriteByte(uint8_t devAddr, uint8_t regAddr, const uint8_t data)
{
    // check for Aux I2C master enabled first
    const bool kAuxI2CEnabled = getAuxI2CEnabled();
    if (!kAuxI2CEnabled) {
        MPU_LOGE(msgs::AUX_I2C_DISABLED, ", must enable first");
        // MPU_ERR_INVALID_STATE;
    }
    // data for regs::I2C_SLV4_ADDR
    buffer[0] = AUXI2C_WRITE << regs::I2C_SLV_RNW_BIT;
    buffer[0] |= devAddr & (0x7F);
    // data for regs::I2C_SLV4_REG
    buffer[1] = regAddr;
    // data for regs::I2C_SLV4_DO
    buffer[2] = data;
    // write configuration above to slave 4 registers
    writeBytes(regs::I2C_SLV4_ADDR, 3, buffer);
    // clear status register before enable this transfer
    readByte(regs::I2C_MST_STATUS, buffer + 15);
    // enable transfer in slave 4
    writeBit(regs::I2C_SLV4_CTRL, regs::I2C_SLV4_EN_BIT, 1);
    // check status until transfer is done
    unsigned long startTick = millis();
    unsigned long endTick   = startTick + 1000; // in miliseconds
    auxi2c_stat_t status;
    do {
        readByte(regs::I2C_MST_STATUS, &status);
        if (status & (1 << regs::I2CMST_STAT_SLV4_NACK_BIT)) {
            MPU_LOGW(msgs::AUX_I2C_SLAVE_NACK);
            // MPU_ERR_NOT_FOUND;
        }
        if (status & (1 << regs::I2CMST_STAT_LOST_ARB_BIT)) {
            MPU_LOGW(msgs::AUX_I2C_LOST_ARB);
            // MPU_FAIL;
        }
        if (millis() >= endTick) {
            MPU_LOGE(msgs::TIMEOUT, ". Aux I2C might've hung. Restart it.");
            // MPU_BUS_TIMEOUT;
        }
    } while (!(status & (1 << regs::I2C_SLV4_DONE_INT_BIT)));

    // MPU_OK;
}

/**
 * @brief Read a single byte frome slave just once (use for configuring a slave at initialization).
 *
 * This function uses Slave 4 to perform single transfers to the slave device on Aux I2C. \n
 * The byte will be transfered at first sample take, so when sample rate is at minimum (4 Hz)
 * it may take up to a quarter of a second to start the transfer.
 * @attention Auxiliary I2C Master must have already been configured before calling this function.
 *
 * @return
 *  - MPU_ERR_INVALID_STATE  Auxiliary I2C Master not enabled;
 *  - MPU_ERR_NOT_FOUND      Slave doesn't ACK the transfer;
 *  - MPU_FAIL               Auxiliary I2C Master lost arbitration of the bus;
 *  - or other standard I2C driver error codes.
 * */
void MPU::auxI2CReadByte(uint8_t devAddr, uint8_t regAddr, uint8_t* data)
{
    // check for Aux I2C master enabled first
    const bool kAuxI2CEnabled = getAuxI2CEnabled();
    if (!kAuxI2CEnabled) {
        MPU_LOGE(msgs::AUX_I2C_DISABLED, ", must enable first");
        // MPU_ERR_INVALID_STATE;
    }
    // data for regs::I2C_SLV4_ADDR
    buffer[0] = AUXI2C_READ << regs::I2C_SLV_RNW_BIT;
    buffer[0] |= devAddr & (0x7F);
    // data for regs::I2C_SLV4_REG
    buffer[1] = regAddr;
    // write configuration above to slave 4 registers
    writeBytes(regs::I2C_SLV4_ADDR, 2, buffer);
    // clear status register before enable this transfer
    readByte(regs::I2C_MST_STATUS, buffer + 15);
    // enable transfer in slave 4
    writeBit(regs::I2C_SLV4_CTRL, regs::I2C_SLV4_EN_BIT, 1);
    // check status until transfer is done
    unsigned long startTick = millis();
    unsigned long endTick   = startTick + 1000; // in miliseconds
    auxi2c_stat_t status;
    do {
        readByte(regs::I2C_MST_STATUS, &status);
        if (status & (1 << regs::I2CMST_STAT_SLV4_NACK_BIT)) {
            MPU_LOGW(msgs::AUX_I2C_SLAVE_NACK);
            // MPU_ERR_NOT_FOUND;
        }
        if (status & (1 << regs::I2CMST_STAT_LOST_ARB_BIT)) {
            MPU_LOGW(msgs::AUX_I2C_LOST_ARB);
            // MPU_FAIL;
        }
        if (millis() >= endTick) {
            MPU_LOGE(msgs::TIMEOUT, ". Aux I2C might've hung. Restart it.");
            // MPU_BUS_TIMEOUT;
        }
    } while (!(status & (1 << regs::I2C_SLV4_DONE_INT_BIT)));
    // get read value
    return readByte(regs::I2C_SLV4_DI, data);
}

/**
 * @brief Configure the active level of FSYNC pin that will cause an interrupt.
 * @details Use setFsyncEnabled() to enable / disable this interrupt.
 * */
void MPU::setFsyncConfig(int_lvl_t level)
{
    writeBit(regs::INT_PIN_CONFIG, regs::INT_CFG_FSYNC_LEVEL_BIT, level);
}

/**
 * @brief Return FSYNC pin active level configuration.
 */
int_lvl_t MPU::getFsyncConfig()
{
    readBit(regs::INT_PIN_CONFIG, regs::INT_CFG_FSYNC_LEVEL_BIT, buffer);
    return (int_lvl_t) buffer[0];
}

/**
 * @brief Enable / disable FSYNC pin to cause an interrupt.
 * @note
 * - The interrupt status is located in I2C_MST_STATUS register, so use
 *   the method getAuxI2CStatus() which reads this register to get FSYNC status.
 *   Keep in mind that a read from I2C_MST_STATUS register clears all its status bits,
 *   so take care to miss status bits when using Auxiliary I2C bus too.
 *
 * - It is possible to enable the FSYNC interrupt propagate to INT pin
 *   with setInterruptEnabled(), then the status can also be read with getInterruptStatus().
 *
 * @see setFsyncConfig().
 * */
void MPU::setFsyncEnabled(bool enable)
{
    writeBit(regs::INT_PIN_CONFIG, regs::INT_CFG_FSYNC_INT_MODE_EN_BIT, enable);
}

/**
 * @brief Return FSYNC enable state.
 */
bool MPU::getFsyncEnabled()
{
    readBit(regs::INT_PIN_CONFIG, regs::INT_CFG_FSYNC_INT_MODE_EN_BIT, buffer);
    return buffer[0];
}

/**
 * @brief Print out register values for debugging purposes.
 * @param start first register number.
 * @param end last register number.
 */
void MPU::registerDump(uint8_t start, uint8_t end)
{
    // constexpr uint8_t kNumOfRegs = 128;
    // if (end - start < 0 || start >= kNumOfRegs || end >= kNumOfRegs) MPU_FAIL;
    MPU_LOGD(LOG_COLOR_W ">> " CONFIG_MPU_CHIP_MODEL " register dump:" LOG_RESET_COLOR "\n");
    uint8_t data;
    for (int i = start; i <= end; i++) {
        readByte(i, &data);
        MPU_LOGD("MPU: reg[ 0x", DebugLogBase::HEX, i," ]  data( 0x", DebugLogBase::HEX, data," )\n");
    }
    
}

#if defined CONFIG_MPU_AK89xx
/**
 * @brief Read a single byte from magnetometer.
 *
 * How it's done: \n
 * It will check the communication protocol which the MPU is connected by.
 *  - I2C, Auxiliary I2C bus will set to bypass mode and the reading will be performed directly (faster).
 *  - SPI, the function will use Slave 4 of Auxiliary I2C bus to read the byte (slower).
 * */
void MPU::compassReadByte(uint8_t regAddr, uint8_t* data)
{
// in case of I2C
#if defined CONFIG_MPU_I2C
    const bool kPrevAuxI2CBypassState = getAuxI2CBypass();
    if (kPrevAuxI2CBypassState == false) {
        setAuxI2CBypass(true);
    }
    bus->readByte(COMPASS_I2CADDRESS, regAddr, data);
    if (kPrevAuxI2CBypassState == false) {
        setAuxI2CBypass(false);
    }
        // in case of SPI
#elif defined CONFIG_MPU_SPI
    return auxI2CReadByte(COMPASS_I2CADDRESS, regAddr, data);
#endif
    
}

/**
 * @brief Write a single byte to magnetometer.
 *
 * How it's done: \n
 * It will check the communication protocol which the MPU is connected by.
 *  - I2C, Auxiliary I2C bus will set to bypass mode and the reading will be performed directly (faster).
 *  - SPI, the function will use Slave 4 of Auxiliary I2C bus to read the byte (slower).
 * */
void MPU::compassWriteByte(uint8_t regAddr, const uint8_t data)
{
// in case of I2C
#if defined CONFIG_MPU_I2C
    const bool kPrevAuxI2CBypassState = getAuxI2CBypass();
    if (kPrevAuxI2CBypassState == false) {
        setAuxI2CBypass(true);
    }
    bus->writeByte(COMPASS_I2CADDRESS, regAddr, data);
    if (kPrevAuxI2CBypassState == false) {
        setAuxI2CBypass(false);
    }
        // in case of SPI
#elif defined CONFIG_MPU_SPI
    return auxI2CWriteByte(COMPASS_I2CADDRESS, regAddr, data);
#endif
    
}

/**
 * @brief Initialize Magnetometer sensor.
 *
 * Initial configuration:
 *  - Mode: single measurement (permits variable sample rate).
 *  - Sensitivity: 0.15 uT/LSB  =  16-bit output.
 *
 * To disable the compass, call compassSetMode(MAG_MODE_POWER_DOWN).
 * */
void MPU::compassInit()
{
#ifdef CONFIG_MPU_I2C
    setAuxI2CBypass(true);

#elif CONFIG_MPU_SPI
    constexpr auxi2c_config_t kAuxI2CConfig = {
        .clock           = AUXI2C_CLOCK_400KHZ,
        .multi_master_en = 1,
        .sample_delay    = 0,
        .shadow_delay_en = 0,
        .wait_for_es     = 0,
        .transition      = AUXI2C_TRANS_RESTART  //
    };
    setAuxI2CConfig(kAuxI2CConfig);
    setAuxI2CEnabled(true);
#endif

        /* configure the magnetometer */
#ifdef CONFIG_MPU_AK8963
    compassReset();
#endif
    compassSetMode(MAG_MODE_SINGLE_MEASURE);

        // finished configs, disable bypass mode
#ifdef CONFIG_MPU_I2C
    setAuxI2CBypass(false);
#endif

        /* For the MPU9150, the auxiliary I2C bus needs to be set to VDD (no idea why) */
#ifdef CONFIG_MPU9150
    setAuxVDDIOLevel(AUXVDDIO_LVL_VDD);
#endif

    MPU_LOGV("Magnetometer configured successfully");
    
}

/**
 * @brief Test connection with Magnetometer by checking WHO_I_AM register.
 * */
void MPU::compassTestConnection()
{
    const uint8_t wai = compassWhoAmI();
    return (wai == 0x48) ? MPU_OK : MPU_ERR_NOT_FOUND;
}

/**
 * @brief Return value from WHO_I_AM register.
 * @details Should be `0x48` for AK8963 and AK8975.
 * */
uint8_t MPU::compassWhoAmI()
{
    compassReadByte(regs::mag::WHO_I_AM, buffer);
    return buffer[0];
}

/**
 * @brief Return value from magnetometer's INFO register.
 * */
uint8_t MPU::compassGetInfo()
{
    compassReadByte(regs::mag::INFO, buffer);
    return buffer[0];
}

/**
 * @brief Change magnetometer's measurement mode.
 * @note
 *  - When user wants to change operation mode, transit to power-down mode first and then transit to other modes.
 *    After power-down mode is set, at least 100µs(Twat) is needed before setting another mode.
 *  - Setting to MAG_MODE_POWER_DOWN will disable readings from compass and disable (free) Aux I2C slaves 0 and 1.
 *    It will not disable Aux I2C Master I/F though! To enable back, use compassInit().
 * */
void MPU::compassSetMode(mag_mode_t mode)
{
// keep previous sensitivity value
#if defined CONFIG_MPU_AK8963
    const uint8_t kControl1Value = mode | (compassGetSensitivity() << regs::mag::CONTROL1_BIT_OUTPUT_BIT);
#else
    const uint8_t kControl1Value = mode;
#endif
    /* POWER-DOWN */
    if (mode == MAG_MODE_POWER_DOWN) {
        setAuxI2CSlaveEnabled(MAG_SLAVE_CHG_MODE, false);
        setAuxI2CSlaveEnabled(MAG_SLAVE_READ_DATA, false);
        compassWriteByte(regs::mag::CONTROL1, kControl1Value);
        MPU_LOGV(msgs::COMPASS_DISABLED, ", set to power-down mode");

        /* SINGLE MEASUREMENT */
    }
    else if (mode == MAG_MODE_SINGLE_MEASURE) {
        // set to power-down first
        compassSetMode(MAG_MODE_POWER_DOWN);
        // set to single measurement
        compassWriteByte(regs::mag::CONTROL1, kControl1Value);

        // slave 0 reads from magnetometer data register
        const auxi2c_slv_config_t kSlaveReadDataConfig = {
            .slave           = MAG_SLAVE_READ_DATA,
            .addr            = COMPASS_I2CADDRESS,
            .rw              = AUXI2C_READ,
            .reg_addr        = regs::mag::STATUS1,
            .reg_dis         = 0,
            .sample_delay_en = 1,
            {{
                .swap_en     = 0,
                .end_of_word = (auxi2c_eow_t) 0,
                .rxlength    = 8  //
            }}                    //
        };
        setAuxI2CSlaveConfig(kSlaveReadDataConfig);

        // slave 1 changes mode to single measurement
        const auxi2c_slv_config_t kSlaveChgModeConfig = {
            .slave           = MAG_SLAVE_CHG_MODE,
            .addr            = COMPASS_I2CADDRESS,
            .rw              = AUXI2C_WRITE,
            .reg_addr        = regs::mag::CONTROL1,
            .reg_dis         = 0,
            .sample_delay_en = 1,
            {.txdata = kControl1Value}  //
        };
        setAuxI2CSlaveConfig(kSlaveChgModeConfig);
        // enable slaves
        setAuxI2CSlaveEnabled(MAG_SLAVE_CHG_MODE, true);
        setAuxI2CSlaveEnabled(MAG_SLAVE_READ_DATA, true);
        MPU_LOGV(msgs::EMPTY, "magnetometer mode set to single measurement");

        /* SELF-TEST */
    }
    else if (mode == MAG_MODE_SELF_TEST) {
        // set to power-down first
        compassSetMode(MAG_MODE_POWER_DOWN);
        // set to self-test
        compassWriteByte(regs::mag::CONTROL1, kControl1Value);
        MPU_LOGV(msgs::EMPTY, "magnetometer mode set to self-test");

        /* FUSE-ROM ACCESS */
    }
    else if (mode == MAG_MODE_FUSE_ROM) {
        // set to power-down first
        compassSetMode(MAG_MODE_POWER_DOWN);
        // set to fuse rom
        compassWriteByte(regs::mag::CONTROL1, kControl1Value);
        MPU_LOGV(msgs::EMPTY, "magnetometer mode set to fuse ROM");

        /* OTHER MODES */
    }
    else {
        MPU_LOGE(msgs::NOT_SUPPORTED, " yet");
        MPU_ERR_NOT_SUPPORTED;
    }

    MPU_OK;
}

/**
 * @brief Return magnetometer's measurement mode.
 * */
mag_mode_t MPU::compassGetMode()
{
    const auxi2c_slv_config_t kSlaveChgModeConfig = getAuxI2CSlaveConfig(MAG_SLAVE_CHG_MODE);
    const bool kSlaveChgModeEnabled = getAuxI2CSlaveEnabled(MAG_SLAVE_CHG_MODE);
    mag_mode_t mode;
    // check if slave 1 is writing the mode to compass
    if (kSlaveChgModeEnabled && kSlaveChgModeConfig.addr == COMPASS_I2CADDRESS) {
        mode = (mag_mode_t)(kSlaveChgModeConfig.txdata & 0xF);
        // otherwise, get directly from the magnetometer register
    }
    else {
        compassReadByte(regs::mag::CONTROL1, buffer);
        mode = (mag_mode_t)(buffer[0] & 0xF);
    }
    return mode;
}

/**
 * @brief Return Magnetometer's sensitivity adjustment data for each axis.
 * */
void MPU::compassGetAdjustment(uint8_t* x, uint8_t* y, uint8_t* z)
{
    mag_mode_t prevMode = compassGetMode();
    compassSetMode(MAG_MODE_FUSE_ROM);
    compassReadByte(regs::mag::ASAX, x);
    compassReadByte(regs::mag::ASAY, y);
    compassReadByte(regs::mag::ASAZ, z);
    // set back previous mode
    return compassSetMode(prevMode);
}

/**
 * @brief Perform Compass self-test.
 * @bug Not fully functional yet.
 * @todo Elaborate comment. Add more `tries`.
 */
bool MPU::compassSelfTest(raw_axes_t* result)
{
    bool ret            = true;
    mag_mode_t prevMode = compassGetMode();
    uint8_t adjValue[3];
    compassGetAdjustment(adjValue, adjValue + 1, adjValue + 2);
    compassSetMode(MAG_MODE_POWER_DOWN);
    // read all data to reset status for any case, start clean
    for (int i = 0; i < 8; i++) compassReadByte(regs::mag::STATUS1 + i, buffer);
    compassWriteByte(regs::mag::ASTC, (1 << regs::mag::ASTC_SELF_TEST_BIT));
    compassSetMode(MAG_MODE_SELF_TEST);
    // wait for data-ready
    uint8_t status1;
    do {
        compassReadByte(regs::mag::STATUS1, &status1);
    } while (!(status1 & (1 << regs::mag::STATUS1_DATA_RDY_BIT));
    MPU_LOGD("status1: 0x", DebugLogBase::HEX, status1);
    for (int i = 0; i < 7; i++) compassReadByte(regs::mag::HXL + i, buffer + i);
    MPU_LOGD("status2: 0x", DebugLogBase::HEX, buffer[6]);
    // convert data
    raw_axes_t data;
    if (result == nullptr) result = &data;
    result->x = math::magAdjust(buffer[1] << 8 | buffer[0], adjValue[0]);
    result->y = math::magAdjust(buffer[3] << 8 | buffer[2], adjValue[1]);
    result->z = math::magAdjust(buffer[5] << 8 | buffer[4], adjValue[2]);
    MPU_LOGD("raw self-test values: %+d %+d %+d", buffer[1] << 8 | buffer[0], buffer[3] << 8 | buffer[2],
             buffer[5] << 8 | buffer[4]);
// check self-test data
#if defined CONFIG_MPU_AK8975
    constexpr int16_t HX_MIN = -100;
    constexpr int16_t HX_MAX = 100;
    constexpr int16_t HY_MIN = -100;
    constexpr int16_t HY_MAX = 100;
    constexpr int16_t HZ_MIN = -1000;
    constexpr int16_t HZ_MAX = -300;
    if (result->x < HX_MIN || result->x > HX_MAX || result->y < HY_MIN || result->y > HY_MAX || result->z < HZ_MIN ||
        result->z > HZ_MAX) {
        ret = false;
    }
#elif defined CONFIG_MPU_AK8963
    // HX_MIN[0] = 14-bit, [1] = 16-bit
    constexpr int16_t HX_MIN[2] = {-50, -200};
    constexpr int16_t HX_MAX[2] = {50, 200};
    constexpr int16_t HY_MIN[2] = {-50, -200};
    constexpr int16_t HY_MAX[2] = {50, 200};
    constexpr int16_t HZ_MIN[2] = {-800, -3200};
    constexpr int16_t HZ_MAX[2] = {-200, -800};
    mag_sensy_t sensy           = compassGetSensitivity();
    if (result->x < HX_MIN[sensy] || result->x > HX_MAX[sensy] || result->y < HY_MIN[sensy] ||
        result->y > HY_MAX[sensy] || result->z < HZ_MIN[sensy] || result->z > HZ_MAX[sensy]) {
        ret = false;
    }
#endif
    // finish test
    compassWriteByte(regs::mag::ASTC, 0x0);
    compassSetMode(prevMode);
    return ret;
}

#ifdef CONFIG_MPU_AK8963
/**
 * @brief Set magnetometer sensitivity.
 * @details Options:
 *  - `MAG_SENSITIVITY_0_6_uT`:   0.6  uT/LSB = 14-bit output
 *  - `MAG_SENSITIVITY_0_15_uT`:  0.15 uT/LSB = 16-bit output
 * */
void MPU::compassSetSensitivity(mag_sensy_t sensy)
{
    auxi2c_slv_config_t slaveChgModeConfig = getAuxI2CSlaveConfig(MAG_SLAVE_CHG_MODE);
    slaveChgModeConfig.txdata &= ~(1 << regs::mag::CONTROL1_BIT_OUTPUT_BIT);
    slaveChgModeConfig.txdata |= (sensy & 0x1) << regs::mag::CONTROL1_BIT_OUTPUT_BIT;
    setAuxI2CSlaveConfig(slaveChgModeConfig);
    // write directly (assurance)
    return compassWriteByte(regs::mag::CONTROL1, slaveChgModeConfig.txdata);
}

/**
 * @brief Return magnetometer sensitivity.
 * */
mag_sensy_t MPU::compassGetSensitivity()
{
    auxi2c_slv_config_t slaveChgModeConfig = getAuxI2CSlaveConfig(MAG_SLAVE_CHG_MODE);
    const bool kSlaveChgModeEnabled = getAuxI2CSlaveEnabled(MAG_SLAVE_CHG_MODE);
    mag_sensy_t sensy;
    // get from slave 1 config, if enabled
    if (kSlaveChgModeEnabled && slaveChgModeConfig.addr == COMPASS_I2CADDRESS) {
        sensy = (mag_sensy_t)((slaveChgModeConfig.txdata >> regs::mag::CONTROL1_BIT_OUTPUT_BIT) & 0x1);
        // otherwise, get directly
    }
    else {
        compassReadByte(regs::mag::CONTROL1, buffer);
        sensy = (mag_sensy_t)((buffer[0] >> regs::mag::CONTROL1_BIT_OUTPUT_BIT) & 0x1);
    }
    return sensy;
}

/**
 * @brief Soft reset AK8963.
 * */
void MPU::compassReset()
{
    return compassWriteByte(regs::mag::CONTROL2, 0x1);
}

#endif  // ak8963 stuff
#endif  // compass methods

/**
 * @brief Trigger gyro and accel hardware self-test.
 * @attention when calling this function, the MPU must remain as horizontal as possible (0 degrees), facing up.
 * @param result Should be ZERO if gyro and accel passed.
 * @todo Elaborate doc.
 * */
void MPU::selfTest(selftest_t* result)
{
#ifdef CONFIG_MPU6050
    constexpr accel_fs_t kAccelFS = ACCEL_FS_16G;
    constexpr gyro_fs_t kGyroFS   = GYRO_FS_250DPS;
#elif defined CONFIG_MPU6500
    constexpr accel_fs_t kAccelFS = ACCEL_FS_2G;
    constexpr gyro_fs_t kGyroFS   = GYRO_FS_250DPS;
#endif
    raw_axes_t gyroRegBias, accelRegBias;
    raw_axes_t gyroSTBias, accelSTBias;
    // get regular biases
    getBiases(kAccelFS, kGyroFS, &accelRegBias, &gyroRegBias, false);
    // get self-test biases
    getBiases(kAccelFS, kGyroFS, &accelSTBias, &gyroSTBias, true);
    // perform self-tests
    uint8_t accelST, gyroST;
    accelSelfTest(accelRegBias, accelSTBias, &accelST);
    gyroSelfTest(gyroRegBias, gyroSTBias, &gyroST);
    // check results
    *result = 0;
    if (accelST != 0) *result |= SELF_TEST_ACCEL_FAIL;
    if (gyroST != 0) *result |= SELF_TEST_GYRO_FAIL;
    
}

#if defined CONFIG_MPU6500
// Production Self-Test table for MPU6500 based models,
// used in accel and gyro self-test code below.
static constexpr uint16_t kSelfTestTable[256] = {
    2620,  2646,  2672,  2699,  2726,  2753,  2781,  2808,   // 7
    2837,  2865,  2894,  2923,  2952,  2981,  3011,  3041,   // 15
    3072,  3102,  3133,  3165,  3196,  3228,  3261,  3293,   // 23
    3326,  3359,  3393,  3427,  3461,  3496,  3531,  3566,   // 31
    3602,  3638,  3674,  3711,  3748,  3786,  3823,  3862,   // 39
    3900,  3939,  3979,  4019,  4059,  4099,  4140,  4182,   // 47
    4224,  4266,  4308,  4352,  4395,  4439,  4483,  4528,   // 55
    4574,  4619,  4665,  4712,  4759,  4807,  4855,  4903,   // 63
    4953,  5002,  5052,  5103,  5154,  5205,  5257,  5310,   // 71
    5363,  5417,  5471,  5525,  5581,  5636,  5693,  5750,   // 79
    5807,  5865,  5924,  5983,  6043,  6104,  6165,  6226,   // 87
    6289,  6351,  6415,  6479,  6544,  6609,  6675,  6742,   // 95
    6810,  6878,  6946,  7016,  7086,  7157,  7229,  7301,   // 103
    7374,  7448,  7522,  7597,  7673,  7750,  7828,  7906,   // 111
    7985,  8065,  8145,  8227,  8309,  8392,  8476,  8561,   // 119
    8647,  8733,  8820,  8909,  8998,  9088,  9178,  9270,   //
    9363,  9457,  9551,  9647,  9743,  9841,  9939,  10038,  //
    10139, 10240, 10343, 10446, 10550, 10656, 10763, 10870,  //
    10979, 11089, 11200, 11312, 11425, 11539, 11654, 11771,  //
    11889, 12008, 12128, 12249, 12371, 12495, 12620, 12746,  //
    12874, 13002, 13132, 13264, 13396, 13530, 13666, 13802,  //
    13940, 14080, 14221, 14363, 14506, 14652, 14798, 14946,  //
    15096, 15247, 15399, 15553, 15709, 15866, 16024, 16184,  //
    16346, 16510, 16675, 16842, 17010, 17180, 17352, 17526,  //
    17701, 17878, 18057, 18237, 18420, 18604, 18790, 18978,  //
    19167, 19359, 19553, 19748, 19946, 20145, 20347, 20550,  //
    20756, 20963, 21173, 21385, 21598, 21814, 22033, 22253,  //
    22475, 22700, 22927, 23156, 23388, 23622, 23858, 24097,  //
    24338, 24581, 24827, 25075, 25326, 25579, 25835, 26093,  //
    26354, 26618, 26884, 27153, 27424, 27699, 27976, 28255,  //
    28538, 28823, 29112, 29403, 29697, 29994, 30294, 30597,  //
    30903, 31212, 31524, 31839, 32157, 32479, 32804, 33132   //
};
#endif

/**
 * @brief Accel Self-test.
 * @param result self-test error for each axis (X=bit0, Y=bit1, Z=bit2). Zero is a pass.
 * @note Bias should be in 16G format for MPU6050 and 2G for MPU6500 based models.
 * */
void MPU::accelSelfTest(raw_axes_t& regularBias, raw_axes_t& selfTestBias, uint8_t* result)
{
#if defined CONFIG_MPU6050
    constexpr accel_fs_t kAccelFS = ACCEL_FS_16G;
    // Criteria A: must be within 14% variation
    constexpr float kMaxVariation = .14f;
    // Criteria B: must be between 300 mg and 950 mg
    constexpr float kMinGravity = .3f, kMaxGravity = .95f;

#elif defined CONFIG_MPU6500
    constexpr accel_fs_t kAccelFS = ACCEL_FS_2G;
    // Criteria A: must be within 50% variation
    constexpr float kMaxVariation = .5f;
    // Criteria B: must be between 255 mg and 675 mg
    constexpr float kMinGravity = .225f, kMaxGravity = .675f;
    // Criteria C: 500 mg for accel
    constexpr float kMaxGravityOffset = .5f;
#endif

    /* Convert biases */
    float_axes_t regularBiasGravity  = math::accelGravity(regularBias, kAccelFS);
    float_axes_t selfTestBiasGravity = math::accelGravity(selfTestBias, kAccelFS);
    MPU_LOGV("regularBias:", regularBias.x, regularBias.y, regularBias.z,
             "| regularBiasGravity:",   DebugLogPrecision::TWO, regularBiasGravity.x, 
                                        DebugLogPrecision::TWO, regularBiasGravity.y,
                                        DebugLogPrecision::TWO, regularBiasGravity.z);
    MPU_LOGV("selfTestBias:", selfTestBias.x, selfTestBias.y, selfTestBias.z,
             "| selfTestBiasGravity:",  DebugLogPrecision::TWO, selfTestBiasGravity.x, 
                                        DebugLogPrecision::TWO, selfTestBiasGravity.y,
                                        DebugLogPrecision::TWO, selfTestBiasGravity.z);

    /* Get OTP production shift code */
    uint8_t shiftCode[3];
#if defined CONFIG_MPU6050
    readBytes(regs::SELF_TEST_X, 4, buffer);
    shiftCode[0] = ((buffer[0] & 0xE0) >> 3) | ((buffer[3] & 0x30) >> 4);
    shiftCode[1] = ((buffer[1] & 0xE0) >> 3) | ((buffer[3] & 0x0C) >> 2);
    shiftCode[2] = ((buffer[2] & 0xE0) >> 3) | (buffer[3] & 0x03);

#elif defined CONFIG_MPU6500
    readBytes(regs::SELF_TEST_X_ACCEL, 3, shiftCode);
#endif
    MPU_LOGV("shiftCode:", shiftCode[0], shiftCode[1], shiftCode[2]);

    /* Calulate production shift value */
    float shiftProduction[3] = {0};
    for (int i = 0; i < 3; i++) {
        if (shiftCode[i] != 0) {
#if defined CONFIG_MPU6050
            // Equivalent to.. shiftProduction[i] = 0.34f * powf(0.92f/0.34f, (shiftCode[i]-1)
            // / 30.f)
            shiftProduction[i] = 0.34f;
            while (--shiftCode[i]) shiftProduction[i] *= 1.034f;

#elif defined CONFIG_MPU6500
            shiftProduction[i] = kSelfTestTable[shiftCode[i] - 1];
            shiftProduction[i] /= math::accelSensitivity(ACCEL_FS_2G);
#endif
        }
    }
    MPU_LOGV("shiftProduction:",    DebugLogPrecision::TWO, shiftProduction[0], 
                                    DebugLogPrecision::TWO, shiftProduction[1],
                                    DebugLogPrecision::TWO, shiftProduction[2]);

    /* Evaluate criterias */
    *result                 = 0;
    float shiftResponse[3]  = {0};
    float shiftVariation[3] = {0};
    for (int i = 0; i < 3; i++) {
        shiftResponse[i] = fabs(selfTestBiasGravity[i] - regularBiasGravity[i]);
        // Criteria A
        if (shiftProduction[i] != 0) {
            shiftVariation[i] = shiftResponse[i] / shiftProduction[i] - 1;
            if (fabs(shiftVariation[i]) > kMaxVariation) *result |= 1 << i;
            // Criteria B
        }
        else if (shiftResponse[i] < kMinGravity || shiftResponse[i] > kMaxGravity) {
            *result |= 1 << i;
        }
// Criteria C
#if defined CONFIG_MPU6050
            // no criteria C
#elif defined CONFIG_MPU6500
        if (fabs(regularBiasGravity[i] > kMaxGravityOffset)) *result |= 1 << i;
#endif
    }
    MPU_LOGV("shiftResponse:",  DebugLogPrecision::TWO, shiftResponse[0], 
                                DebugLogPrecision::TWO, shiftResponse[1],
                                DebugLogPrecision::TWO, shiftResponse[2]);
    MPU_LOGV("shiftVariation:", shiftVariation[0], shiftVariation[1], shiftVariation[2]);

    MPU_LOGD("Accel self-test: [X=", ((*result & 0x1) ? "FAIL" : "OK"),
                            "] [Y=", ((*result & 0x2) ? "FAIL" : "OK"),
                            "] [Z=", ((*result & 0x4) ? "FAIL" : "OK"),"]");
    
}

/**
 * @brief Gyro Self-test.
 * @param result Self-test error for each axis (X=bit0, Y=bit1, Z=bit2). Zero is a pass.
 * @note Bias should be in 250DPS format for both MPU6050 and MPU6500 based models.
 * */
void MPU::gyroSelfTest(raw_axes_t& regularBias, raw_axes_t& selfTestBias, uint8_t* result)
{
    constexpr gyro_fs_t kGyroFS = GYRO_FS_250DPS;

#if defined CONFIG_MPU6050
    // Criteria A: must not exceed +14% variation
    constexpr float kMaxVariation = .14f;
    // Criteria B: must be between 10 dps and 105 dps
    constexpr float kMinDPS = 10.f, kMaxDPS = 105.f;

#elif defined CONFIG_MPU6500
    // Criteria A: must be within 50% variation
    constexpr float kMaxVariation = .5f;
    // Criteria B: must be between 20 dps and 60 dps
    constexpr float kMinDPS = 20.f, kMaxDPS = 60.f;
#endif

    /* Convert biases */
    float_axes_t regularBiasDPS  = math::gyroDegPerSec(regularBias, kGyroFS);
    float_axes_t selfTestBiasDPS = math::gyroDegPerSec(selfTestBias, kGyroFS);

    MPU_LOGV("regularBias:", regularBias.x, regularBias.y, regularBias.z,
             "| regularBiasGravity:",   DebugLogPrecision::TWO, regularBiasDPS.x, 
                                        DebugLogPrecision::TWO, regularBiasDPS.y,
                                        DebugLogPrecision::TWO, regularBiasDPS.z);
    MPU_LOGV("selfTestBias:", selfTestBias.x, selfTestBias.y, selfTestBias.z,
             "| selfTestBiasGravity:",  DebugLogPrecision::TWO, selfTestBiasDPS.x, 
                                        DebugLogPrecision::TWO, selfTestBiasDPS.y,
                                        DebugLogPrecision::TWO, selfTestBiasDPS.z);

    /* Get OTP production shift code */
    uint8_t shiftCode[3];
#if defined CONFIG_MPU6050
    readBytes(regs::SELF_TEST_X, 3, buffer);
    shiftCode[0] = buffer[0] & 0x1F;
    shiftCode[1] = buffer[1] & 0x1F;
    shiftCode[2] = buffer[2] & 0x1F;

#elif defined CONFIG_MPU6500
    readBytes(regs::SELF_TEST_X_GYRO, 3, shiftCode);
#endif
    MPU_LOGV("shiftCode:", shiftCode[0], shiftCode[1], shiftCode[2]);

    /* Calulate production shift value */
    float shiftProduction[3] = {0};
    for (int i = 0; i < 3; i++) {
        if (shiftCode[i] != 0) {
#if defined CONFIG_MPU6050
            shiftProduction[i] = 3275.f / math::gyroSensitivity(kGyroFS);  // should yield 25
            while (--shiftCode[i]) shiftProduction[i] *= 1.046f;

#elif defined CONFIG_MPU6500
            shiftProduction[i] = kSelfTestTable[shiftCode[i] - 1];
            shiftProduction[i] /= math::gyroSensitivity(kGyroFS);
#endif
        }
    }
    MPU_LOGV("shiftProduction:",    DebugLogPrecision::TWO, shiftProduction[0], 
                                    DebugLogPrecision::TWO, shiftProduction[1],
                                    DebugLogPrecision::TWO, shiftProduction[2]);

    /* Evaluate criterias */
    *result                 = 0;
    float shiftResponse[3]  = {0};
    float shiftVariation[3] = {0};
    for (int i = 0; i < 3; i++) {
        shiftResponse[i] = fabs(selfTestBiasDPS[i] - regularBiasDPS[i]);
        // Criteria A
        if (shiftProduction[i] != 0) {
            shiftVariation[i] = shiftResponse[i] / shiftProduction[i] - 1;
            if (fabs(shiftVariation[i]) > kMaxVariation) *result |= 1 << i;
            // Criteria B
        }
        else if (shiftResponse[i] < kMinDPS || shiftResponse[i] > kMaxDPS) {
            *result |= 1 << i;
        }
    }
    MPU_LOGV("shiftResponse:",  DebugLogPrecision::TWO, shiftResponse[0], 
                                DebugLogPrecision::TWO, shiftResponse[1],
                                DebugLogPrecision::TWO, shiftResponse[2]);
    MPU_LOGV("shiftVariation:", shiftVariation[0], shiftVariation[1], shiftVariation[2]);

    MPU_LOGD("Accel self-test: [X=", ((*result & 0x1) ? "FAIL" : "OK"),
                            "] [Y=", ((*result & 0x2) ? "FAIL" : "OK"),
                            "] [Z=", ((*result & 0x4) ? "FAIL" : "OK"),"]");
                                         
    
}

/**
 * @brief Compute the Biases in regular mode and self-test mode.
 * @attention When calculating the biases the MPU must remain as horizontal as possible (0 degrees), facing up.
 * This algorithm takes about ~400ms to compute offsets.
 * */
void MPU::getBiases(accel_fs_t accelFS, gyro_fs_t gyroFS, raw_axes_t* accelBias, raw_axes_t* gyroBias,
                         bool selftest)
{
    // configurations to compute biases
    constexpr uint16_t kSampleRate      = 1000;
    constexpr dlpf_t kDLPF              = DLPF_188HZ;
    constexpr fifo_config_t kFIFOConfig = FIFO_CFG_ACCEL | FIFO_CFG_GYRO;
    constexpr size_t kPacketSize        = 12;
    // backup previous configuration
    const uint16_t prevSampleRate      = getSampleRate();
    const dlpf_t prevDLPF              = getDigitalLowPassFilter();
    const accel_fs_t prevAccelFS       = getAccelFullScale();
    const gyro_fs_t prevGyroFS         = getGyroFullScale();
    const fifo_config_t prevFIFOConfig = getFIFOConfig();
    const bool prevFIFOState           = getFIFOEnabled();
    // setup
    setSampleRate(kSampleRate);
    setDigitalLowPassFilter(kDLPF);
    setAccelFullScale(accelFS);
    setGyroFullScale(gyroFS);
    setFIFOConfig(kFIFOConfig);
    setFIFOEnabled(true);
    if (selftest) {
        writeBits(regs::ACCEL_CONFIG, regs::ACONFIG_XA_ST_BIT, 3, 0x7);
        writeBits(regs::GYRO_CONFIG, regs::GCONFIG_XG_ST_BIT, 3, 0x7);
    }
    // wait for 200ms for sensors to stabilize
    delay(200);
    // fill FIFO for 100ms
    resetFIFO();
    delay(100);
    setFIFOConfig(FIFO_CFG_NONE);
    // get FIFO count
    const uint16_t fifoCount = getFIFOCount();
    const int packetCount = fifoCount / kPacketSize;
    // read overrun bytes, if any
    const int overrunCount      = fifoCount - (packetCount * kPacketSize);
    uint8_t buffer[kPacketSize] = {0};
    if (overrunCount > 0) {
        readFIFO(overrunCount, buffer);
    }
    // fetch data and add up
    axes_t<int> accelAvg, gyroAvg;
    for (int i = 0; i < packetCount; i++) {
        readFIFO(kPacketSize, buffer);
        // retrieve data
        raw_axes_t accelCur, gyroCur;
        accelCur.x = (buffer[0] << 8) | buffer[1];
        accelCur.y = (buffer[2] << 8) | buffer[3];
        accelCur.z = (buffer[4] << 8) | buffer[5];
        gyroCur.x  = (buffer[6] << 8) | buffer[7];
        gyroCur.y  = (buffer[8] << 8) | buffer[9];
        gyroCur.z  = (buffer[10] << 8) | buffer[11];
        // add up
        accelAvg.x += accelCur.x;
        accelAvg.y += accelCur.y;
        accelAvg.z += accelCur.z;
        gyroAvg.x += gyroCur.x;
        gyroAvg.y += gyroCur.y;
        gyroAvg.z += gyroCur.z;
    }
    // calculate average
    accelAvg.x /= packetCount;
    accelAvg.y /= packetCount;
    accelAvg.z /= packetCount;
    gyroAvg.x /= packetCount;
    gyroAvg.y /= packetCount;
    gyroAvg.z /= packetCount;
    // remove gravity from Accel Z axis
    const uint16_t gravityLSB = INT16_MAX >> (accelFS + 1);
    accelAvg.z -= gravityLSB;
    // save biases
    for (int i = 0; i < 3; i++) {
        (*accelBias)[i] = (int16_t) accelAvg[i];
        (*gyroBias)[i]  = (int16_t) gyroAvg[i];
    }
    // set back previous configs
    setSampleRate(prevSampleRate);
    setDigitalLowPassFilter(prevDLPF);
    setAccelFullScale(prevAccelFS);
    setGyroFullScale(prevGyroFS);
    setFIFOConfig(prevFIFOConfig);
    setFIFOEnabled(prevFIFOState);
    
}

}  // namespace mpud