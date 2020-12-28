#ifndef ENUT_IMU_H
#define ENUT_IMU_H

#include "Shared/enut_ifaces.h"
#include "ModulesChain/Module.h"
#include "Shared/mpuXX50.h"

class Enut_imu : public ipm::modules::module, public enut::imu_iface
{
public:
    Enut_imu();
    enut::imu_iface::imu_data get_imu() override;

    bool opened(){return m_fd > 0;}

private:

    void loop();
    int m_fd;
    // Setup the MPU class
    struct i2c_device_t i2c_dev;
    MPUXX50 *mpuXX50;
    enut::imu_iface::imu_data m_imu_data;
};

#endif // ENUT_IMU_H
