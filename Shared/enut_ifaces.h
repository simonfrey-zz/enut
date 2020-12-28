#ifndef ENUT_IFACES_H
#define ENUT_IFACES_H

#include "Shared/enut_models.h"

namespace enut {

class imu_iface {
public:
    struct imu_data{
        double roll;
        double pitch;
    };
    virtual imu_data get_imu() = 0;
};

class angles_iface {
public:
    virtual Angles get_angles() = 0;
    virtual void set_angles( Angles, double speed ) = 0;
};

}

#endif // ENUT_IFACES_H
