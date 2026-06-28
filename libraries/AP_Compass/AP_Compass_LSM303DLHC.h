#pragma once

#include "AP_Compass_config.h"

#if AP_COMPASS_LSM303DLHC_ENABLED

#define HAL_COMPASS_LSM303DLHC_I2C_ADDR 0x1E

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/Device.h>
#include <AP_Math/AP_Math.h>

#include "AP_Compass.h"
#include "AP_Compass_Backend.h"

class AP_Compass_LSM303DLHC : public AP_Compass_Backend
{
public:
    static AP_Compass_Backend *probe(AP_HAL::OwnPtr<AP_HAL::Device> dev,
                                     enum Rotation rotation);

    static AP_Compass_Backend *probe_i2c(AP_HAL::OwnPtr<AP_HAL::Device> dev,
                                         bool force_external,
                                         enum Rotation rotation);

    static constexpr const char *name = "LSM303DLHC";

    void read() override;

    virtual ~AP_Compass_LSM303DLHC() {}

private:
    AP_Compass_LSM303DLHC(AP_HAL::OwnPtr<AP_HAL::Device> dev, bool force_external);

    bool init(enum Rotation rotation);
    bool _read_sample();
    bool _hardware_init();
    void _update();
    bool _register_read(uint8_t reg, uint8_t &val);
    bool _register_write(uint8_t reg, uint8_t val);
    bool _register_modify(uint8_t reg, uint8_t clearbits, uint8_t setbits);

    AP_HAL::OwnPtr<AP_HAL::Device> _dev;
    bool _force_external;

    int16_t _mag_x;
    int16_t _mag_y;
    int16_t _mag_z;

    bool _initialised;
};

#endif  // AP_COMPASS_LSM303DLHC_ENABLED
