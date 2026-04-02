/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>

#include <math.h>

#include "platform.h"

#ifdef USE_MAG_QMC5883P

#include "common/axis.h"
#include "common/maths.h"
#include "common/utils.h"

#include "drivers/bus.h"
#include "drivers/bus_i2c.h"
#include "drivers/bus_i2c_busdev.h"
#include "drivers/sensor.h"
#include "drivers/time.h"

#include "compass.h"
#include "compass_qmc5883p.h"

// QMC5883P I2C Address
#define QMC5883P_I2C_ADDRESS            0x2C

// Registers
#define QMC5883P_REG_CHIPID             0x00
#define QMC5883P_REG_XOUT_LSB           0x01
#define QMC5883P_REG_XOUT_MSB           0x02
#define QMC5883P_REG_YOUT_LSB           0x03
#define QMC5883P_REG_YOUT_MSB           0x04
#define QMC5883P_REG_ZOUT_LSB           0x05
#define QMC5883P_REG_ZOUT_MSB           0x06
#define QMC5883P_REG_STATUS             0x09
#define QMC5883P_REG_CONTROL1           0x0A
#define QMC5883P_REG_CONTROL2           0x0B

// Chip ID
#define QMC5883P_CHIP_ID                0x80

// Control Register 1 bits
#define QMC5883P_MODE_MASK              0x03
#define QMC5883P_ODR_MASK               (0x03 << 2)
#define QMC5883P_RNG_MASK               (0x03 << 4)
#define QMC5883P_OSR_MASK               (0x03 << 6)

// Control Register 2 bits
#define QMC5883P_SETRESET_MASK          0x03
#define QMC5883P_RESET_BIT              (1 << 7)
#define QMC5883P_TEST_BIT               (1 << 6)

// Status register bits
#define QMC5883P_STATUS_DRDY            (1 << 0)
#define QMC5883P_STATUS_OVL             (1 << 1)

// Operating modes
typedef enum {
    QMC5883P_MODE_STANDBY = 0x00,
    QMC5883P_MODE_CONTINUOUS = 0x01,
} qmc5883p_mode_t;

// Output data rates
typedef enum {
    QMC5883P_ODR_10HZ = (0x00 << 2),
    QMC5883P_ODR_50HZ = (0x01 << 2),
    QMC5883P_ODR_100HZ = (0x02 << 2),
    QMC5883P_ODR_200HZ = (0x03 << 2),
} qmc5883p_odr_t;

// Field ranges
typedef enum {
    QMC5883P_RANGE_2G = (0x00 << 4),
    QMC5883P_RANGE_8G = (0x01 << 4),
    QMC5883P_RANGE_12G = (0x02 << 4),
    QMC5883P_RANGE_30G = (0x03 << 4),
} qmc5883p_range_t;

// Over sample ratios
typedef enum {
    QMC5883P_OSR_512 = (0x00 << 6),
    QMC5883P_OSR_256 = (0x01 << 6),
    QMC5883P_OSR_128 = (0x02 << 6),
    QMC5883P_OSR_64 = (0x03 << 6),
} qmc5883p_osr_t;

// Set/Reset modes
typedef enum {
    QMC5883P_SETRESET_OFF = 0x00,
    QMC5883P_SETRESET_SETONLY = 0x01,
    QMC5883P_SETRESET_ON = 0x02,
} qmc5883p_setreset_t;

// Default configuration
#define QMC5883P_DEFAULT_MODE            QMC5883P_MODE_CONTINUOUS
#define QMC5883P_DEFAULT_ODR             QMC5883P_ODR_100HZ
#define QMC5883P_DEFAULT_RANGE           QMC5883P_RANGE_8G
#define QMC5883P_DEFAULT_OSR             QMC5883P_OSR_512
#define QMC5883P_DEFAULT_SETRESET        QMC5883P_SETRESET_ON

// Conversion factors (LSB per Gauss)
#define QMC5883P_LSB_PER_GAUSS_2G       15000.0f
#define QMC5883P_LSB_PER_GAUSS_8G       3750.0f
#define QMC5883P_LSB_PER_GAUSS_12G      2500.0f
#define QMC5883P_LSB_PER_GAUSS_30G      1000.0f

static bool qmc5883pInit(magDev_t *magDev)
{
    extDevice_t *dev = &magDev->dev;
    busDeviceRegister(dev);

    // Check chip ID
    uint8_t chipId;
    if (!busReadRegisterBuffer(dev, QMC5883P_REG_CHIPID, &chipId, 1) || chipId != QMC5883P_CHIP_ID) {
        return false;
    }

    // Configure Control Register 1
    uint8_t ctrl1 = QMC5883P_DEFAULT_MODE | QMC5883P_DEFAULT_ODR | QMC5883P_DEFAULT_RANGE | QMC5883P_DEFAULT_OSR;
    if (!busWriteRegister(dev, QMC5883P_REG_CONTROL1, ctrl1)) {
        return false;
    }

    // Configure Control Register 2
    uint8_t ctrl2 = QMC5883P_DEFAULT_SETRESET;
    if (!busWriteRegister(dev, QMC5883P_REG_CONTROL2, ctrl2)) {
        return false;
    }

    // Set ODR based on configuration
    switch (QMC5883P_DEFAULT_ODR) {
    case QMC5883P_ODR_10HZ:
        magDev->magOdrHz = 10;
        break;
    case QMC5883P_ODR_50HZ:
        magDev->magOdrHz = 50;
        break;
    case QMC5883P_ODR_100HZ:
        magDev->magOdrHz = 100;
        break;
    case QMC5883P_ODR_200HZ:
        magDev->magOdrHz = 200;
        break;
    default:
        magDev->magOdrHz = 100;
        break;
    }

    return true;
}

static bool qmc5883pRead(magDev_t *magDev, int16_t *magData)
{
    extDevice_t *dev = &magDev->dev;
    uint8_t buf[6];
    uint8_t status;

    // Check if data is ready
    if (!busReadRegisterBuffer(dev, QMC5883P_REG_STATUS, &status, 1)) {
        return false;
    }

    if (!(status & QMC5883P_STATUS_DRDY)) {
        return false;
    }

    // Read data
    if (!busReadRegisterBuffer(dev, QMC5883P_REG_XOUT_LSB, buf, sizeof(buf))) {
        return false;
    }

    // Convert to int16_t (2's complement)
    int16_t rawX = (int16_t)((buf[1] << 8) | buf[0]);
    int16_t rawY = (int16_t)((buf[3] << 8) | buf[2]);
    int16_t rawZ = (int16_t)((buf[5] << 8) | buf[4]);

    magData[X] = rawX;
    magData[Y] = rawY;
    magData[Z] = rawZ;

    return true;
}

bool qmc5883pDetect(magDev_t *magDev)
{
    extDevice_t *dev = &magDev->dev;

    if (dev->bus->busType == BUS_TYPE_I2C && dev->busType_u.i2c.address == 0) {
        dev->busType_u.i2c.address = QMC5883P_I2C_ADDRESS;
    }

    // Check chip ID
    uint8_t chipId;
    if (!busReadRegisterBuffer(dev, QMC5883P_REG_CHIPID, &chipId, 1) || chipId != QMC5883P_CHIP_ID) {
        return false;
    }

    magDev->init = qmc5883pInit;
    magDev->read = qmc5883pRead;

    return true;
}

#endif