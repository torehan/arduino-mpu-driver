// =========================================================================
// This library is placed under the MIT License
// Copyright 2017-2018 Natanael Josue Rabello. All rights reserved.
// For the license information refer to LICENSE file in root directory.
// =========================================================================

/**
 * @file mpu/log.hpp
 * @brief MPU Log System
 * 
 * @attention
 *  This header is intended to be used ONLY inside the library itself
 *  Do not include this header in your application.
 * */

#ifndef _MPU_LOG_HPP_
#define _MPU_LOG_HPP_

// #include "esp_err.h"
// #include "esp_log.h"
#include "DebugLog.h"
#include "config_mpu_lib.h"
#include "types.hpp"

// Note: declare TAG before include this header
// Note: include only in .cpp files from this library

#define MPU_LOGE(format,...)  LOG_ERROR(format, ##__VA_ARGS__) 
#define MPU_LOGW(format,...)  LOG_WARN(format, ##__VA_ARGS__) 
#define MPU_LOGI(format,...)  LOG_INFO(format, ##__VA_ARGS__)
#define MPU_LOGD(format,...)  LOG_DEBUG(format, ##__VA_ARGS__) 
#define MPU_LOGV(format,...)  LOG_TRACE(format, ##__VA_ARGS__) 

/*! MPU Driver namespace */
namespace mpud
{
/*! Log namespace */
inline namespace log
{
/*! Messages namespace */
namespace msgs
{
static const char INVALID_ARG[]           = "Invalid Argument";
static const char INVALID_STATE[]         = "Invalid State";
static const char INVALID_LENGTH[]        = "Invalid length";
static const char INVALID_FIFO_RATE[]     = "Invalid FIFO rate";
static const char INVALID_SAMPLE_RATE[]   = "Invalid Sample rate";
static const char INVALID_TAP_THRESH[]    = "Invalid Tap threshold";
static const char DMP_LOAD_FAIL[]         = "Failed to load DMP firmware";
static const char DMP_NOT_LOADED[]        = "DMP firmware has not been loaded";
static const char UNKNOWN_DMP_CFG_STATE[] = "Unknown DMP config state";
static const char NO_AXIS_PASSED[]        = "No Axis passed";
static const char BANK_BOUNDARIES[]       = "Bank boundaries overpass";
static const char FIFO_CORRUPTION[]       = "FIFO Corruption. Quaternion data outside of the acceptable threshold";
static const char AUX_I2C_DISABLED[]      = "Auxiliary I2C is disabled";
static const char AUX_I2C_SLAVE_NACK[]    = "Auxiliary I2C Slave NACK";
static const char AUX_I2C_LOST_ARB[]      = "Auxiliary I2C Master loose abitraion of the bus";
static const char COMPASS_DISABLED[]      = "Compass is disabled";
static const char NOT_SUPPORTED[]         = "Not supported";
static const char TIMEOUT[]               = "Timeout";
static const char EMPTY[]                 = "";

}  // namespace msgs

}  // namespace log

}  // namespace mpud

#endif /* end of include guard: _MPU_LOG_HPP_ */
