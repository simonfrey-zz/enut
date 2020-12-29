#ifndef ENUT_SIMULATOR_H
#define ENUT_SIMULATOR_H

#include "ModulesChain/Module.h"
#include "IPM_Plot/plot3d_interface.h"
#include "enut_models.h"
#include "Shared/enut_ifaces.h"
#include <mutex>

class Enut_Simulator : public ipm::modules::module, public enut::imu_iface, public enut::angles_iface
{
public:
    Enut_Simulator( Plot3D_Interface::shared_t p3d );

    void set_ground_angles(double roll, double pitch , double yaw);

    void dbg_set_angles( double s_fl, double s_fr, double s_hl, double s_hr,
                     double l_fl, double l_fr, double l_hl, double l_hr,
                     double f_fl, double f_fr, double f_hl, double f_hr,
                     double head);

    enut::imu_iface::imu_data get_imu() override;
    enut::Angles get_angles( ) override;
    void set_angles( enut::Angles angles, double speed ) override;

    void turn_off() override {}
    void turn_on() override {}

private:
    Plot3D_Interface::shared_t m_p3d;

    void loop();
    void draw();
    void move_angles( double dt );
    enut::Body body;

    double m_speed;

    double m_ground_pitch;
    double m_ground_roll;
    double m_ground_yaw;

    enut::Angles m_soll_angles;
    enut::Angles m_ist_angles;

    double m_imu_roll;
    double m_imu_pitch;
    double m_imu_yaw;

    std::mutex m_mutex;

};

#endif // ENUT_SIMULATOR_H
