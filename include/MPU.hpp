// =========================================================================
// This library is placed under the MIT License
// Copyright 2017-2018 Natanael Josue Rabello. All rights reserved.
// For the license information refer to LICENSE file in root directory.
// =========================================================================

/**
 * @file MPU.hpp
 * MPU library main file. Declare MPU class.
 *
 * @attention
 *  MPU library requires I2Cbus or SPIbus library.
 *  Select the communication protocol in `menuconfig`
 *  and include the corresponding library to project components.
 *
 * @note
 *  The following is taken in the code:
 *  - MPU9250 is the same as MPU6500 + AK8963
 *  - MPU9150 is the same as MPU6050 + AK8975
 *  - MPU6000 code equals MPU6050
 *  - MPU6555 code equals MPU6500
 *  - MPU9255 code equals MPU9250
 * */

#ifndef _MPU_HPP_
#define _MPU_HPP_

#include <stdint.h>
//#include "esp_err.h"
#include "config_mpu_lib.h"

#ifdef CONFIG_MPU_I2C
// #if !defined I2CBUS_COMPONENT_TRUE
// #error ''MPU component requires I2Cbus library. \
// Make sure the I2Cbus library is included in your components directory. \
// See MPUs README.md for more information.''
// #endif

#include <Adafruit_I2CDevice.h>

#elif defined CONFIG_MPU_SPI
// #if !defined SPIBUS_COMPONENT_TRUE
// #error ''MPU component requires SPIbus library. \
// Make sure the SPIbus library is included in your components directory. \
// See MPUs README.md for more information.''
// #endif
#include <Adafruit_SPIDevice.h>
#else
#error ''MPU communication protocol not specified''
#endif

#include "mpu/types.hpp"

/*! MPU Driver namespace */
namespace mpud
{
class MPU;
}

/*! Easy alias for MPU class */
typedef mpud::MPU MPU_t;

namespace mpud
{
/*! Motion Processing Unit */
class MPU
{
 public:
    //! \name Constructors / Destructor
    //! \{
    MPU();
    explicit MPU(mpu_bus_t* bus);
    #ifdef CONFIG_MPU_SPI
        MPU(mpu_bus_t* bus);
    #elif defined CONFIG_MPU_I2C
        MPU(mpu_bus_t* bus, mpu_addr_handle_t addr);
    #endif
    ~MPU();
    //! \}
    //! \name Basic
    //! \{
    MPU& setBus(mpu_bus_t* bus);
    // MPU& setAddr(mpu_addr_handle_t addr);
    mpu_bus_t* getBus();
    // mpu_addr_handle_t getAddr();
    int lastError();
    //! \}
    //! \name Setup
    //! \{
    int initialize();
    int reset();
    int setSleep(bool enable);
    int testConnection();
    int selfTest(selftest_t* result);
    int resetSignalPath();
    uint8_t whoAmI();
    bool getSleep();
    //! \}
    //! \name Main configurations
    //! \{
    int setSampleRate(uint16_t rate);
    int setClockSource(clock_src_t clockSrc);
    int setDigitalLowPassFilter(dlpf_t dlpf);
    uint16_t getSampleRate();
    clock_src_t getClockSource();
    dlpf_t getDigitalLowPassFilter();
    //! \}
    //! \name Power management
    //! \{
    int setLowPowerAccelMode(bool enable);
    int setLowPowerAccelRate(lp_accel_rate_t rate);
    lp_accel_rate_t getLowPowerAccelRate();
    bool getLowPowerAccelMode();
    int setStandbyMode(stby_en_t mask);
    stby_en_t getStandbyMode();
    //! \}
    //! \name Full-Scale Range
    //! \{
    int setGyroFullScale(gyro_fs_t fsr);
    int setAccelFullScale(accel_fs_t fsr);
    gyro_fs_t getGyroFullScale();
    accel_fs_t getAccelFullScale();
    //! \}
    //! \name Offset / Bias
    //! \{
    int setGyroOffset(raw_axes_t bias);
    int setAccelOffset(raw_axes_t bias);
    raw_axes_t getGyroOffset();
    raw_axes_t getAccelOffset();
    int computeOffsets(raw_axes_t* accel, raw_axes_t* gyro);
    //! \}
    //! \name Interrupt
    //! \{
    int setInterruptConfig(int_config_t config);
    int setInterruptEnabled(int_en_t mask);
    int_stat_t getInterruptStatus();
    int_config_t getInterruptConfig();
    int_en_t getInterruptEnabled();
    //! \}
    //! \name FIFO
    //! \{
    int setFIFOMode(fifo_mode_t mode);
    int setFIFOConfig(fifo_config_t config);
    int setFIFOEnabled(bool enable);
    int resetFIFO();
    uint16_t getFIFOCount();
    int readFIFO(size_t length, uint8_t* data);
    int writeFIFO(size_t length, const uint8_t* data);
    fifo_mode_t getFIFOMode();
    fifo_config_t getFIFOConfig();
    bool getFIFOEnabled();
    //! \}
    //! \name Auxiliary I2C Master
    //! \{
    int setAuxI2CConfig(const auxi2c_config_t& config);
    int setAuxI2CEnabled(bool enable);
    int setAuxI2CSlaveConfig(const auxi2c_slv_config_t& config);
    int setAuxI2CSlaveEnabled(auxi2c_slv_t slave, bool enable);
    int setAuxI2CBypass(bool enable);
    int readAuxI2CRxData(size_t length, uint8_t* data, size_t skip = 0);
    int restartAuxI2C();
    auxi2c_stat_t getAuxI2CStatus();
    auxi2c_config_t getAuxI2CConfig();
    auxi2c_slv_config_t getAuxI2CSlaveConfig(auxi2c_slv_t slave);
    bool getAuxI2CEnabled();
    bool getAuxI2CSlaveEnabled(auxi2c_slv_t slave);
    bool getAuxI2CBypass();
    int auxI2CWriteByte(uint8_t devAddr, uint8_t regAddr, uint8_t data);
    int auxI2CReadByte(uint8_t devAddr, uint8_t regAddr, uint8_t* data);
    //! \}
    //! \name Motion Detection Interrupt
    //! \{
    int setMotionDetectConfig(mot_config_t& config);
    mot_config_t getMotionDetectConfig();
    int setMotionFeatureEnabled(bool enable);
    bool getMotionFeatureEnabled();
#if defined CONFIG_MPU6000 || defined CONFIG_MPU6050 || defined CONFIG_MPU9150
    int setZeroMotionConfig(zrmot_config_t& config);
    zrmot_config_t getZeroMotionConfig();
    int setFreeFallConfig(ff_config_t& config);
    ff_config_t getFreeFallConfig();
    mot_stat_t getMotionDetectStatus();
#endif
    //! \}
    //! \name Compass | Magnetometer
    //! \{
#if defined CONFIG_MPU_AK89xx
    int compassInit();
    int compassTestConnection();
    int compassSetMode(mag_mode_t mode);
    int compassGetAdjustment(uint8_t* x, uint8_t* y, uint8_t* z);
    mag_mode_t compassGetMode();
    uint8_t compassWhoAmI();
    uint8_t compassGetInfo();
    int compassReadByte(uint8_t regAddr, uint8_t* data);
    int compassWriteByte(uint8_t regAddr, uint8_t data);
    bool compassSelfTest(raw_axes_t* result = nullptr);
#endif
#if defined CONFIG_MPU_AK8963
    int compassReset();
    int compassSetSensitivity(mag_sensy_t sensy);
    mag_sensy_t compassGetSensitivity();
#endif
    //! \}
    //! \name Miscellaneous
    //! \{
    int setFsyncConfig(int_lvl_t level);
    int setFsyncEnabled(bool enable);
    int_lvl_t getFsyncConfig();
    bool getFsyncEnabled();
#if defined CONFIG_MPU6500 || defined CONFIG_MPU9250
    int setFchoice(fchoice_t fchoice);
    fchoice_t getFchoice();
#endif
#if defined CONFIG_MPU9150 || (defined CONFIG_MPU6050 && !defined CONFIG_MPU6000)
    int setAuxVDDIOLevel(auxvddio_lvl_t level);
    auxvddio_lvl_t getAuxVDDIOLevel();
#endif
    //! \}
    //! \name Read / Write
    //! Functions to perform direct read or write operation(s) to registers.
    //! \{
    bool readBit(uint8_t regAddr, uint8_t bitNum, uint8_t* data);
    bool readBits(uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t* data);
    bool readByte(uint8_t regAddr, uint8_t* data);
    bool readBytes(uint8_t regAddr, size_t length, uint8_t* data);
    bool writeBit(uint8_t regAddr, uint8_t bitNum, uint8_t data);
    bool writeBits(uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t data);
    bool writeByte(uint8_t regAddr, uint8_t data);
    bool writeBytes(uint8_t regAddr, size_t length, uint8_t* data);
    bool registerDump(uint8_t start = 0x0, uint8_t end = 0x7F);
    //! \}
    //! \name Sensor readings
    //! \{
    int acceleration(raw_axes_t* accel);
    int acceleration(int16_t* x, int16_t* y, int16_t* z);
    int rotation(raw_axes_t* gyro);
    int rotation(int16_t* x, int16_t* y, int16_t* z);
    int temperature(int16_t* temp);
    int motion(raw_axes_t* accel, raw_axes_t* gyro);
#if defined CONFIG_MPU_AK89xx
    int heading(raw_axes_t* mag);
    int heading(int16_t* x, int16_t* y, int16_t* z);
    int motion(raw_axes_t* accel, raw_axes_t* gyro, raw_axes_t* mag);
#endif
    int sensors(raw_axes_t* accel, raw_axes_t* gyro, int16_t* temp);
    int sensors(sensors_t* sensors, size_t extsens_len = 0);
    //! \}

 protected:
    int accelSelfTest(raw_axes_t& regularBias, raw_axes_t& selfTestBias, uint8_t* result);
    int gyroSelfTest(raw_axes_t& regularBias, raw_axes_t& selfTestBias, uint8_t* result);
    int getBiases(accel_fs_t accelFS, gyro_fs_t gyroFS, raw_axes_t* accelBias, raw_axes_t* gyroBias,
                        bool selftest);

    mpu_bus_t* bus;         /*!< Communication bus pointer, I2C / SPI */
    //mpu_addr_handle_t addr; /*!< I2C address / SPI device handle */
    uint8_t buffer[16];     /*!< Commom buffer for temporary data */
    int err;          /*!< Holds last error code */
};

}  // namespace mpud

// ==============
// Inline methods
// ==============
namespace mpud
{
/*! Default Constructor. */
inline MPU::MPU() : MPU(){};
/**
 * @brief Contruct a MPU in the given communication bus.
 * @param bus Bus protocol object of type `I2Cbus` or `SPIbus`.
 */
inline MPU::MPU(mpu_bus_t* bus) : MPU(bus) {}
// inline MPU::MPU(mpu_bus_t& bus) : MPU(bus, MPU_DEFAULT_ADDR_HANDLE) {}
/**
 * @brief Construct a MPU in the given communication bus and address.
 * @param bus Bus protocol object of type `I2Cbus` or `SPIbus`.
 * @param addr I2C address (`mpu_i2caddr_t`) or SPI device handle (`spi_device_handle_t`).
 */
// inline MPU::MPU(mpu_bus_t& bus, mpu_addr_handle_t addr) : bus{&bus}, addr{addr}, buffer{0}, err{ESP_OK} {}
/** Default Destructor, does nothing. */
inline MPU::~MPU() = default;
/**
 * @brief Set communication bus.
 * @param bus Bus protocol object of type `I2Cbus` or `SPIbus`.
 */
inline MPU& MPU::setBus(mpu_bus_t* bus)
{
    this->bus = bus;
    return *this;
}
/**
 * @brief Return communication bus object.
 */
inline mpu_bus_t* MPU::getBus()
{
    return bus;
}
/**
 * @brief Set I2C address or SPI device handle.
 * @param addr I2C address (`mpu_i2caddr_t`) or SPI device handle (`spi_device_handle_t`).
 */
/*
inline MPU& MPU::setAddr(mpu_addr_handle_t addr)
{
    this->addr = addr;
    return *this;
}
*/
/**
 * @brief Return I2C address or SPI device handle.
 */
/*
inline mpu_addr_handle_t MPU::getAddr()
{
    return addr;
}
*/
/*! Return last error code. */

inline int MPU::lastError()
{
    return err;
}
/*! Read a single bit from a register*/
inline bool MPU::readBit(uint8_t regAddr, uint8_t bitNum, uint8_t* data)
{
    static bool _err = readBits(regAddr, bitNum, 1, data);
    return _err;
}
/*! Read a range of bits from a register */
inline bool MPU::readBits(uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t* data)
{
    uint8_t buffer;
    bool _err = readByte(regAddr, &buffer);
    if(!_err) {
        uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
        buffer &= mask;
        buffer >>= (bitStart - length + 1);
        *data = buffer;
    }
    return _err;    
}
/*! Read a single register */
inline bool MPU::readByte(uint8_t regAddr, uint8_t* data)
{
    static bool _err = readBytes(regAddr,1,data);
    return _err;
}
/*! Read data from sequence of registers */
inline bool MPU::readBytes(uint8_t regAddr, size_t length, uint8_t* data)
{
    static bool _err = false;

    #ifdef CONFIG_MPU_SPI
        _err = bus->read(data, length, regAddr);

        #if defined CONFIG_SPIBUS_LOG_READWRITES
            if (!_err) { 
                char str[length*5+1]; 
                for(size_t i = 0; i < length; i++) 
                sprintf(str+i*5, "0x%s%X ", (data[i] < 0x10 ? "0" : ""), data[i]);
                SPIBUS_LOG_RW("[%s, handle:0x%X] Read_ %d bytes from register 0x%X, data: %s", (host == 1 ? "HSPI" : "VSPI"), (uint32_t)handle, length, regAddr, str);
            }
        #endif
    #elif defined CONFIG_MPU_I2C

    #endif
    return _err;
}
/*! Write a single bit to a register */
inline bool MPU::writeBit(uint8_t regAddr, uint8_t bitNum, uint8_t data)
{
    // return err = bus->writeBit(addr, regAddr, bitNum, data);
}
/*! Write a range of bits to a register */
inline bool MPU::writeBits(uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t data)
{
    // return err = bus->writeBits(addr, regAddr, bitStart, length, data);
}
/*! Write a value to a register */
inline bool MPU::writeByte(uint8_t regAddr, uint8_t data)
{
    // return err = bus->writeByte(addr, regAddr, data);

}
/*! Write a sequence to data to a sequence of registers */
inline bool MPU::writeBytes(uint8_t regAddr, size_t length, uint8_t* data)
{
    static bool _err = false;

    #ifdef CONFIG_MPU_SPI
        _err = bus->write(data, length, &regAddr, 1);

        #if defined CONFIG_SPIBUS_LOG_READWRITES
            if (!_err) { 
                char str[length*5+1]; 
                for(size_t i = 0; i < length; i++) 
                sprintf(str+i*5, "0x%s%X ", (data[i] < 0x10 ? "0" : ""), data[i]);
                SPIBUS_LOG_RW("[%s, handle:0x%X] Read_ %d bytes from register 0x%X, data: %s", (host == 1 ? "HSPI" : "VSPI"), (uint32_t)handle, length, regAddr, str);
            }
        #endif
    #elif defined CONFIG_MPU_I2C

    #endif
    //return _err;

    // return err = bus->write(data, length, &regAddr, 1);
}

}  // namespace mpud

#endif /* end of include guard: _MPU_HPP_ */
