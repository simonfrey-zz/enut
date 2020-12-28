#ifndef ENUT_CONTROLLER_H
#define ENUT_CONTROLLER_H

#include "ModulesChain/Module.h"
#include "Shared/enut_ifaces.h"
#include "ceres/ceres.h"
#include "IPM_Control/IPM_PID.h"
#include "IPM_SCPI++/SCPIClassAdaptor.h"

class Enut_Controller: public ipm::modules::module, public SCPIClassAdaptor<Enut_Controller>
{
public:
    Enut_Controller( enut::imu_iface * imu, enut::angles_iface * angles );

    bool set_height( double height );
    std::pair<bool,double> get_height(){return {true,m_height}; }

    bool set_attitude( std::string a );
    std::pair<bool, std::string> get_attitude(){ return {true,enut::Attitude_to_string(m_attitude)}; }


private:
    enut::imu_iface * m_imu;
    enut::angles_iface * m_angles;
    double m_height;
    enut::Attitude m_attitude;
    enut::Body body;

    std::map<int, Eigen::Vector3d> m_foot_pose;
    std::map<int, enut::Enut_Pate_model*> m_pates_models;

    IPM_PID * m_pid_roll;
    IPM_PID * m_pid_pitch;

    void loop();

    void init_ceres();
    ceres::Problem::Options problem_options;
    ceres::Problem problem;
    std::map<int, double> foot_angles;
    std::map<int, double> leg_angles;
    std::map<int, double> shoulder_angles;
    double loss_size = 0.05;
    ceres::Solver::Options options;
    ceres::Solver::Summary summary;

};

#endif // ENUT_CONTROLLER_H
