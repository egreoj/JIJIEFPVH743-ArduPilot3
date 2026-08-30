#pragma once

#include "AP_InertialSensor.h"
#include "AP_InertialSensor_Backend.h"

class AP_InertialSensor_LSM6DSK320X : public AP_InertialSensor_Backend
{
public:
    static AP_InertialSensor_Backend *probe(AP_InertialSensor &imu,
                                             AP_HAL::SPIDevice *dev,
                                             enum Rotation rotation);

    /* update accel and gyro instances */
    bool update() override;

    void start() override;

private:
    AP_InertialSensor_LSM6DSK320X(AP_InertialSensor &imu,
                                   AP_HAL::SPIDevice *dev,
                                   enum Rotation rotation);

    AP_HAL::SPIDevice *_dev;
    uint8_t _gyro_instance;
    uint8_t _accel_instance;

    bool _init_sensor(void);
    bool _read_data(void);
    void _configure_registers(void);

    static const uint8_t REG_ID = 0x0F;
    static const uint8_t ID_VAL = 0x6C;  // LSM6DSK320X device ID
};
