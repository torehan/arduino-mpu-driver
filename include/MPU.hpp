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
#include <Adafruit_I2CDevice.h>
#if !defined Adafruit_I2CDevice_h
#error ''MPU component requires Adafruit I2CDevice library. \
Make sure the library is existing in your include directory. \''
#endif


#elif defined CONFIG_MPU_SPI
#include <Adafruit_SPIDevice.h>
#if !defined Adafruit_SPIDevice_h
#error ''MPU component requires Adafruit SPIDevice library. \
Make sure the library is existing in your include directory. \''
#endif
#else
#error ''MPU communication protocol not specified''
#endif

#include "mpu/types.hpp"
#include "mpu/log.hpp"

#include <Wire.h>

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
    // explicit MPU(mpu_bus_t& bus);
    // MPU(mpu_bus_t& bus, mpu_addr_handle_t addr);
    MPU(mpu_device_specifier_t addr_or_cs);

    ~MPU();
    //! \}
    //! \name Basic
    //! \{
    // MPU& setBus(mpu_bus_t& bus);
    // MPU& setAddr(mpu_addr_handle_t addr);
    // mpu_bus_t& getBus();
    // mpu_addr_handle_t getAddr();
    //! \}
    //! \name Setup
    //! \{
    void initialize();
    void reset();
    void setSleep(bool enable);
    mpu_err_t testConnection();
    void selfTest(selftest_t* result);
    void resetSignalPath();
    uint8_t whoAmI();
    bool getSleep();
    //! \}
    //! \name Main configurations
    //! \{
    void setSampleRate(uint16_t rate);
    void setClockSource(clock_src_t clockSrc);
    void setDigitalLowPassFilter(dlpf_t dlpf);
    uint16_t getSampleRate();
    clock_src_t getClockSource();
    dlpf_t getDigitalLowPassFilter();
    //! \}
    //! \name Power management
    //! \{
    void setLowPowerAccelMode(bool enable);
    void setLowPowerAccelRate(lp_accel_rate_t rate);
    lp_accel_rate_t getLowPowerAccelRate();
    bool getLowPowerAccelMode();
    void setStandbyMode(stby_en_t mask);
    stby_en_t getStandbyMode();
    //! \}
    //! \name Full-Scale Range
    //! \{
    void setGyroFullScale(gyro_fs_t fsr);
    void setAccelFullScale(accel_fs_t fsr);
    gyro_fs_t getGyroFullScale();
    accel_fs_t getAccelFullScale();
    //! \}
    //! \name Offset / Bias
    //! \{
    void setGyroOffset(raw_axes_t bias);
    void setAccelOffset(raw_axes_t bias);
    raw_axes_t getGyroOffset();
    raw_axes_t getAccelOffset();
    void computeOffsets(raw_axes_t* accel, raw_axes_t* gyro);
    //! \}
    //! \name Interrupt
    //! \{
    void setInterruptConfig(int_config_t config);
    void setInterruptEnabled(int_en_t mask);
    int_stat_t getInterruptStatus();
    int_config_t getInterruptConfig();
    int_en_t getInterruptEnabled();
    //! \}
    //! \name FIFO
    //! \{
    void setFIFOMode(fifo_mode_t mode);
    void setFIFOConfig(fifo_config_t config);
    void setFIFOEnabled(bool enable);
    void resetFIFO();
    uint16_t getFIFOCount();
    void readFIFO(size_t length, uint8_t* data);
    void writeFIFO(size_t length, uint8_t* data);
    fifo_mode_t getFIFOMode();
    fifo_config_t getFIFOConfig();
    bool getFIFOEnabled();
    //! \}
    //! \name Auxiliary I2C Master
    //! \{
    void setAuxI2CConfig(const auxi2c_config_t& config);
    void setAuxI2CEnabled(bool enable);
    void setAuxI2CSlaveConfig(const auxi2c_slv_config_t& config);
    void setAuxI2CSlaveEnabled(auxi2c_slv_t slave, bool enable);
    void setAuxI2CBypass(bool enable);
    void readAuxI2CRxData(size_t length, uint8_t* data, size_t skip = 0);
    void restartAuxI2C();
    auxi2c_stat_t getAuxI2CStatus();
    auxi2c_config_t getAuxI2CConfig();
    auxi2c_slv_config_t getAuxI2CSlaveConfig(auxi2c_slv_t slave);
    bool getAuxI2CEnabled();
    bool getAuxI2CSlaveEnabled(auxi2c_slv_t slave);
    bool getAuxI2CBypass();
    void auxI2CWriteByte(uint8_t devAddr, uint8_t regAddr, uint8_t data);
    void auxI2CReadByte(uint8_t devAddr, uint8_t regAddr, uint8_t* data);
    //! \}
    //! \name Motion Detection Interrupt
    //! \{
    void setMotionDetectConfig(mot_config_t& config);
    mot_config_t getMotionDetectConfig();
    void setMotionFeatureEnabled(bool enable);
    bool getMotionFeatureEnabled();
#if defined CONFIG_MPU6000 || defined CONFIG_MPU6050 || defined CONFIG_MPU9150
    void setZeroMotionConfig(zrmot_config_t& config);
    zrmot_config_t getZeroMotionConfig();
    void setFreeFallConfig(ff_config_t& config);
    ff_config_t getFreeFallConfig();
    mot_stat_t getMotionDetectStatus();
#endif
    //! \}
    //! \name Compass | Magnetometer
    //! \{
#if defined CONFIG_MPU_AK89xx
    void compassInit();
    void compassTestConnection();
    void compassSetMode(mag_mode_t mode);
    void compassGetAdjustment(uint8_t* x, uint8_t* y, uint8_t* z);
    mag_mode_t compassGetMode();
    uint8_t compassWhoAmI();
    uint8_t compassGetInfo();
    void compassReadByte(uint8_t regAddr, uint8_t* data);
    void compassWriteByte(uint8_t regAddr, uint8_t data);
    bool compassSelfTest(raw_axes_t* result = nullptr);
#endif
#if defined CONFIG_MPU_AK8963
    void compassReset();
    void compassSetSensitivity(mag_sensy_t sensy);
    mag_sensy_t compassGetSensitivity();
#endif
    //! \}
    //! \name Miscellaneous
    //! \{
    void setFsyncConfig(int_lvl_t level);
    void setFsyncEnabled(bool enable);
    int_lvl_t getFsyncConfig();
    bool getFsyncEnabled();
#if defined CONFIG_MPU6500 || defined CONFIG_MPU9250
    void setFchoice(fchoice_t fchoice);
    fchoice_t getFchoice();
#endif
#if defined CONFIG_MPU9150 || (defined CONFIG_MPU6050 && !defined CONFIG_MPU6000)
    void setAuxVDDIOLevel(auxvddio_lvl_t level);
    auxvddio_lvl_t getAuxVDDIOLevel();
#endif
    //! \}
    //! \name Read / Write
    //! Functions to perform direct read or write operation(s) to registers.
    //! \{
    void readBit(uint8_t regAddr, uint8_t bitNum, uint8_t* data);
    void readBits(uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t* data);
    void readByte(uint8_t regAddr, uint8_t* data);
    void readBytes(uint8_t regAddr, size_t length, uint8_t* data);
    void writeBit(uint8_t regAddr, uint8_t bitNum, uint8_t data);
    void writeBits(uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t data);
    void writeByte(uint8_t regAddr, uint8_t data);
    void writeBytes(uint8_t regAddr, size_t length, uint8_t* data);
    void registerDump(uint8_t start = 0x0, uint8_t end = 0x7F);
    //! \}
    //! \name Sensor readings
    //! \{
    void acceleration(raw_axes_t* accel);
    void acceleration(int16_t* x, int16_t* y, int16_t* z);
    void rotation(raw_axes_t* gyro);
    void rotation(int16_t* x, int16_t* y, int16_t* z);
    void temperature(int16_t* temp);
    void motion(raw_axes_t* accel, raw_axes_t* gyro);
#if defined CONFIG_MPU_AK89xx
    void heading(raw_axes_t* mag);
    void heading(int16_t* x, int16_t* y, int16_t* z);
    void motion(raw_axes_t* accel, raw_axes_t* gyro, raw_axes_t* mag);
#endif
    void sensors(raw_axes_t* accel, raw_axes_t* gyro, int16_t* temp);
    void sensors(sensors_t* sensors, size_t extsens_len = 0);
    //! \}

 protected:
    void accelSelfTest(raw_axes_t& regularBias, raw_axes_t& selfTestBias, uint8_t* result);
    void gyroSelfTest(raw_axes_t& regularBias, raw_axes_t& selfTestBias, uint8_t* result);
    void getBiases(accel_fs_t accelFS, gyro_fs_t gyroFS, raw_axes_t* accelBias, raw_axes_t* gyroBias,
                        bool selftest);

    mpu_bus_t bus;         /*!< Communication bus, I2C / SPI */
    uint8_t buffer[16];     /*!< Commom buffer for temporary data */
};

}  // namespace mpud

// ==============
// Inline methods
// ==============
namespace mpud
{


/*! Default Constructor. */
// inline MPU::MPU() : MPU(MPU_DEFAULT_BUS){};
/**
//  * @brief Contruct a MPU in the given communication bus.
//  * @param bus Bus protocol object of type `I2Cbus` or `SPIbus`.
//  */
// inline MPU::MPU(mpu_bus_t& bus) : MPU(bus, MPU_DEFAULT_ADDR_HANDLE) {}
// /**
//  * @brief Construct a MPU in the given communication bus and address.
//  * @param bus Bus protocol object of type `I2Cbus` or `SPIbus`.
//  * @param addr I2C address (`mpu_i2caddr_t`) or SPI device handle (`spi_device_handle_t`).
//  */
inline MPU::MPU(mpu_device_specifier_t addr_or_cs) : bus(10, 1000000, SPI_BITORDER_MSBFIRST, SPI_MODE0){
    bus.begin();        
}
// inline MPU::MPU(mpu_bus_t& bus, mpu_addr_handle_t addr) : bus{&bus}, addr{addr}, buffer{0}, err{ESP_OK} {}
// /** Default Destructor, does nothing. */
inline MPU::~MPU() = default;
// /**
//  * @brief Set communication bus.
//  * @param bus Bus protocol object of type `I2Cbus` or `SPIbus`.
//  */
// inline MPU& MPU::setBus(mpu_bus_t* bus)
// {
//     this->bus = bus;
//     return *this;
// }
// /**
//  * @brief Return communication bus object.
//  */
// inline mpu_bus_t& MPU::getBus()
// {
//     return bus;
// }
// /**
//  * @brief Set I2C address or SPI device handle.
//  * @param addr I2C address (`mpu_i2caddr_t`) or SPI device handle (`spi_device_handle_t`).
//  */
// inline MPU& MPU::setAddr(mpu_addr_handle_t addr)
// {
//     this->addr = addr;
//     return *this;
// }
// /**
//  * @brief Return I2C address or SPI device handle.
//  */
// inline mpu_addr_handle_t MPU::getAddr()
// {
//     return addr;
// }


// /*! Read a single bit from a register*/

inline void MPU::readBit(uint8_t regAddr, uint8_t bitNum, uint8_t* data)
{
    readBits(regAddr, bitNum, 1, data);
}
/*! Read a range of bits from a register */
inline void MPU::readBits(uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t* data)
{
    uint8_t buffer;
    readByte(regAddr, &buffer);
    uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
    buffer &= mask;
    buffer >>= (bitStart - length + 1);
    *data = buffer;
        
}
/*! Read a single register */
inline void MPU::readByte(uint8_t regAddr, uint8_t* data)
{
    readBytes(regAddr,1,data);
}
/*! Read data from sequence of registers */
inline void MPU::readBytes(uint8_t regAddr, size_t length, uint8_t* data)
{
    uint8_t addr = regAddr | SPIBUS_READ;

    #ifdef CONFIG_MPU_SPI
        
        bus.write_then_read(&addr, 1, data, length);
        delayMicroseconds(1);

        #if defined CONFIG_SPIBUS_LOG_READWRITES
            for(size_t i = 0; i < length; i++){
                MPU_LOGV("[0x", DebugLogBase::HEX, (int)&bus,"] Read ", length, "bytes from register 0x", DebugLogBase::HEX, regAddr," data: 0x",DebugLogBase::HEX, data[i]);
            }
        #endif
    #elif defined CONFIG_MPU_I2C

    #endif
    
}
/*! Write a single bit to a register */
inline void MPU::writeBit(uint8_t regAddr, uint8_t bitNum, uint8_t data)
{
    uint8_t buffer;
    readByte(regAddr, &buffer);
    
    buffer = data ? (buffer | (1 << bitNum)) : (buffer & ~(1 << bitNum));
    writeByte(regAddr, buffer);
}
/*! Write a range of bits to a register */
inline void MPU::writeBits(uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t data)
{
    uint8_t buffer;
    readByte(regAddr, &buffer);
    
    uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
    data <<= (bitStart - length + 1);
    data &= mask;
    buffer &= ~mask;
    buffer |= data;
    writeByte(regAddr, buffer);

    // bus->writeBits(addr, regAddr, bitStart, length, data);
}
/*! Write a value to a register */
inline void MPU::writeByte(uint8_t regAddr, uint8_t data)
{
    // bus->writeByte(addr, regAddr, data);
    writeBytes(regAddr, 1, &data);
}
/*! Write a sequence to data to a sequence of registers */
inline void MPU::writeBytes(uint8_t regAddr, size_t length, uint8_t* data)
{   
    uint8_t _regAddr = regAddr & SPIBUS_WRITE;
    
    #ifdef CONFIG_MPU_SPI
        bus.write(data, length, &_regAddr, 1);
        delayMicroseconds(1);
        #if defined CONFIG_SPIBUS_LOG_READWRITES

                for(size_t i = 0; i < length; i++){
                    MPU_LOGV("[0x", DebugLogBase::HEX, (int)&bus,"] Read ", length, "bytes from register 0x", DebugLogBase::HEX, regAddr," data: 0x",DebugLogBase::HEX, data[i]);
                }

        #endif

    #elif defined CONFIG_MPU_I2C

    #endif
    

}

}  // namespace mpud

#endif /* end of include guard: _MPU_HPP_ */
