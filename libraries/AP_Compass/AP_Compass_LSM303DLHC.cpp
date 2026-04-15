/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
 * AP_Compass_LSM303DLHC.cpp - Driver for LSM303DLHC I2C magnetometer
 * Supports: LSM303DLHC (0x1E)
 */
#include "AP_Compass_LSM303DLHC.h"

#if AP_COMPASS_LSM303DLHC_ENABLED

#include <utility>
#include <stdio.h>

#include <AP_Math/AP_Math.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/utility/sparse-endian.h>

extern const AP_HAL::HAL& hal;

/*
 * LSM303DLHC I2C Address: 0x1E
 */

/* Register addresses */
#define LSM303DLHC_MAG_CRA_REG_M    0x00
#define LSM303DLHC_MAG_CRB_REG_M    0x01
#define LSM303DLHC_MAG_MR_REG_M     0x02
#define LSM303DLHC_MAG_OUT_X_H_M    0x03
#define LSM303DLHC_MAG_OUT_X_L_M    0x04
#define LSM303DLHC_MAG_OUT_Z_H_M    0x05
#define LSM303DLHC_MAG_OUT_Z_L_M    0x06
#define LSM303DLHC_MAG_OUT_Y_H_M    0x07
#define LSM303DLHC_MAG_OUT_Y_L_M    0x08
#define LSM303DLHC_MAG_SR_REG_M     0x09
#define LSM303DLHC_MAG_IRA_REG_M    0x0A
#define LSM303DLHC_MAG_IRB_REG_M    0x0B
#define LSM303DLHC_MAG_IRC_REG_M    0x0C

/* CRA_REG_M bits */
#define LSM303DLHC_MAG_CRA_DO_0_75HZ    0x00
#define LSM303DLHC_MAG_CRA_DO_1_5HZ     0x04
#define LSM303DLHC_MAG_CRA_DO_3HZ       0x08
#define LSM303DLHC_MAG_CRA_DO_7_5HZ     0x0C
#define LSM303DLHC_MAG_CRA_DO_15HZ      0x10
#define LSM303DLHC_MAG_CRA_DO_30HZ      0x14
#define LSM303DLHC_MAG_CRA_DO_75HZ      0x18

/* CRB_REG_M bits - gain selection */
#define LSM303DLHC_MAG_CRB_GN_1_3GA     0x20
#define LSM303DLHC_MAG_CRB_GN_1_9GA     0x40
#define LSM303DLHC_MAG_CRB_GN_2_5GA     0x60
#define LSM303DLHC_MAG_CRB_GN_4_0GA     0x80
#define LSM303DLHC_MAG_CRB_GN_4_7GA     0xA0
#define LSM303DLHC_MAG_CRB_GN_5_6GA     0xC0
#define LSM303DLHC_MAG_CRB_GN_8_1GA     0xE0

/* MR_REG_M bits */
#define LSM303DLHC_MAG_MR_CONTINUOUS    0x00
#define LSM303DLHC_MAG_MR_SINGLE        0x01

/* SR_REG_M bits */
#define LSM303DLHC_MAG_SR_DRDY          0x01


AP_Compass_LSM303DLHC::AP_Compass_LSM303DLHC(AP_HAL::OwnPtr<AP_HAL::Device> dev, bool force_external, enum Rotation rotation)
    : _dev(std::move(dev))
    , _force_external(force_external)
    , _rotation(rotation)
    , _initialised(false)
{
}

AP_Compass_Backend *AP_Compass_LSM303DLHC::probe(AP_HAL::OwnPtr<AP_HAL::Device> dev,
                                                  enum Rotation rotation)
{
    return probe_i2c(std::move(dev), false, rotation);
}

AP_Compass_Backend *AP_Compass_LSM303DLHC::probe_i2c(AP_HAL::OwnPtr<AP_HAL::Device> dev,
                                                      bool force_external,
                                                      enum Rotation rotation)
{
    if (!dev) {
        return nullptr;
    }

    AP_Compass_LSM303DLHC *sensor = NEW_NOTHROW AP_Compass_LSM303DLHC(std::move(dev), force_external, rotation);
    if (!sensor || !sensor->init(rotation)) {
        delete sensor;
        return nullptr;
    }

    return sensor;
}

uint8_t AP_Compass_LSM303DLHC::_register_read(uint8_t reg)
{
    uint8_t val = 0;
    _dev->read_registers(reg, &val, 1);
    return val;
}

void AP_Compass_LSM303DLHC::_register_write(uint8_t reg, uint8_t val)
{
    _dev->write_register(reg, val);
}

void AP_Compass_LSM303DLHC::_register_modify(uint8_t reg, uint8_t clearbits, uint8_t setbits)
{
    uint8_t val = _register_read(reg);
    val &= ~clearbits;
    val |= setbits;
    _register_write(reg, val);
}

bool AP_Compass_LSM303DLHC::_hardware_init()
{
    _dev->get_semaphore()->take_blocking();

    // initially run the bus at low speed
    _dev->set_speed(AP_HAL::Device::SPEED_LOW);

    // Set CRA_REG_M: Temperature sensor enable (bit 7), Data output rate 75Hz (bits 4:2 = 0x18)
    // 0x98 = 0b10011000 (TEMP_EN=1, DO=75Hz)
    _register_write(LSM303DLHC_MAG_CRA_REG_M, 0x98);

    // Set CRB_REG_M: Gain 1.3 Gauss (default/full scale) - 0x20
    _register_write(LSM303DLHC_MAG_CRB_REG_M, LSM303DLHC_MAG_CRB_GN_1_3GA);

    // Set MR_REG_M: Continuous conversion mode
    _register_write(LSM303DLHC_MAG_MR_REG_M, LSM303DLHC_MAG_MR_CONTINUOUS);

    _dev->set_speed(AP_HAL::Device::SPEED_HIGH);
    _dev->get_semaphore()->give();

    return true;
}

bool AP_Compass_LSM303DLHC::init(enum Rotation rotation)
{
    if (!_dev) {
        return false;
    }

    _dev->get_semaphore()->take_blocking();

    // Test WHOAMI to verify device
    uint8_t whoami = _register_read(LSM303DLHC_MAG_IRA_REG_M);
    if (whoami != 0x48) {  // Expected value for LSM303DLHC
        _dev->get_semaphore()->give();
        return false;  // Sensor not found or not responding correctly
    }

    _dev->get_semaphore()->give();

    if (!_hardware_init()) {
        return false;
    }

    _initialised = true;

    // Register compass instance with bus ID
    _dev->set_device_type(DEVTYPE_LSM303D);
    if (!register_compass(_dev->get_bus_id())) {
        return false;
    }

    set_external(_force_external);

    // Set rotation
    set_rotation(rotation);

    // Register periodic read callback at ~91Hz (11ms period)
    _dev->register_periodic_callback(11000, FUNCTOR_BIND_MEMBER(&AP_Compass_LSM303DLHC::_update, void));

    return true;
}

bool AP_Compass_LSM303DLHC::_data_ready()
{
    uint8_t status = _register_read(LSM303DLHC_MAG_SR_REG_M);
    return (status & LSM303DLHC_MAG_SR_DRDY) != 0;
}

bool AP_Compass_LSM303DLHC::_read_sample()
{
    if (!_data_ready()) {
        return false;
    }

    uint8_t buffer[6];
    if (!_dev->read_registers(LSM303DLHC_MAG_OUT_X_H_M, buffer, 6)) {
        return false;
    }

    // Data layout: X_H, X_L, Z_H, Z_L, Y_H, Y_L
    int16_t x = (int16_t)(((uint16_t)buffer[0] << 8) | buffer[1]);
    int16_t z = (int16_t)(((uint16_t)buffer[2] << 8) | buffer[3]);
    int16_t y = (int16_t)(((uint16_t)buffer[4] << 8) | buffer[5]);

    // Store for read()
    _mag_x = x;
    _mag_y = y;
    _mag_z = z;

    return true;
}

void AP_Compass_LSM303DLHC::read()
{
    // Drain accumulated samples from periodic callback
    drain_accumulated_samples();
}

void AP_Compass_LSM303DLHC::_update()
{
    if (!_initialised) {
        return;
    }

    if (!_read_sample()) {
        return;
    }

    // Convert raw values to magnetic field (in mGauss)
    // LSM303DLHC at 1.3 Gauss range: XY=1100 LSB/Gauss, Z=980 LSB/Gauss
    Vector3f field(_mag_x / 1100.0f * 1000.0f,
                   _mag_y / 1100.0f * 1000.0f,
                   _mag_z / 980.0f * 1000.0f);

    // Apply rotation before accumulating
    field.rotate(_rotation);

    // Accumulate sample for averaging
    accumulate_sample(field, 10);
}

#endif  // AP_COMPASS_LSM303DLHC_ENABLED
