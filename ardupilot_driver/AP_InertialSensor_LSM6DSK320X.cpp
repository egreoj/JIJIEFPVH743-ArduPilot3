#include "AP_InertialSensor_LSM6DSK320X.h"
#include <AP_HAL/AP_HAL.h>
#include <stdio.h>

extern const AP_HAL::HAL& hal;

AP_InertialSensor_Backend *AP_InertialSensor_LSM6DSK320X::probe(AP_InertialSensor &imu,
                                                                  AP_HAL::SPIDevice *dev,
                                                                  enum Rotation rotation)
{
    if (dev == nullptr) {
        return nullptr;
    }

    AP_InertialSensor_LSM6DSK320X *sensor = new AP_InertialSensor_LSM6DSK320X(imu, dev, rotation);
    if (!sensor || !sensor->_init_sensor()) {
        delete sensor;
        return nullptr;
    }

    return sensor;
}

AP_InertialSensor_LSM6DSK320X::AP_InertialSensor_LSM6DSK320X(AP_InertialSensor &imu,
                                                              AP_HAL::SPIDevice *dev,
                                                              enum Rotation rotation)
    : AP_InertialSensor_Backend(imu, rotation),
      _dev(dev)
{
}

bool AP_InertialSensor_LSM6DSK320X::_init_sensor()
{
    if (_dev == nullptr) {
        return false;
    }

    WITH_SEMAPHORE(_dev->get_semaphore());

    uint8_t whoami = 0;
    if (!_dev->read_registers(REG_ID, &whoami, 1)) {
        return false;
    }

    if (whoami != ID_VAL) {
        return false;
    }

    _gyro_instance = _imu.register_gyro(800, _dev->get_bus_speed());
    _accel_instance = _imu.register_accel(800, _dev->get_bus_speed());

    _configure_registers();

    return true;
}

void AP_InertialSensor_LSM6DSK320X::_configure_registers()
{
    // Configure LSM6DSK320X with basic settings
    // CTRL1_XL: Accel config - ODR 800 Hz, ±16g range
    uint8_t reg = 0x80;  // ODR 800Hz, ±16g
    _dev->write_register(0x10, reg);

    // CTRL2_G: Gyro config - ODR 800 Hz, ±2000 dps
    reg = 0x80;  // ODR 800Hz, ±2000 dps
    _dev->write_register(0x11, reg);

    // CTRL3_C: Configure device mode
    reg = 0x40;  // Auto increment enabled
    _dev->write_register(0x12, reg);
}

void AP_InertialSensor_LSM6DSK320X::start()
{
    // Nothing needed here in this basic implementation
}

bool AP_InertialSensor_LSM6DSK320X::_read_data()
{
    uint8_t data[14];
    Vector3f accel, gyro;

    if (!_dev->read_registers(0x20, data, 14)) {
        return false;
    }

    // Parse accelerometer data (registers 0x28-0x2D)
    int16_t ax = (int16_t)(data[0] | (data[1] << 8));
    int16_t ay = (int16_t)(data[2] | (data[3] << 8));
    int16_t az = (int16_t)(data[4] | (data[5] << 8));

    // Parse gyroscope data (registers 0x22-0x27)
    int16_t gx = (int16_t)(data[6] | (data[7] << 8));
    int16_t gy = (int16_t)(data[8] | (data[9] << 8));
    int16_t gz = (int16_t)(data[10] | (data[11] << 8));

    // Convert to SI units
    accel = Vector3f(ax, ay, az) * (16.0f * 9.81f / 32768.0f);
    gyro = Vector3f(gx, gy, gz) * (2000.0f * M_PI / 180.0f / 32768.0f);

    _publish_gyro(_gyro_instance, gyro);
    _publish_accel(_accel_instance, accel);

    return true;
}

bool AP_InertialSensor_LSM6DSK320X::update()
{
    _read_data();
    return true;
}
