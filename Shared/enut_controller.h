#ifndef ENUT_CONTROLLER_H
#define ENUT_CONTROLLER_H

#include "ModulesChain/Module.h"
#include "Shared/enut_ifaces.h"
#include "ceres/ceres.h"
#include "IPM_Control/IPM_PID.h"
#include "IPM_SCPI++/SCPIClassAdaptor.h"
#include "IPM_Parameter/IPM_SectionedParmFile.h"

class Enut_Controller: public ipm::modules::module, public SCPIClassAdaptor<Enut_Controller>
{
public:
    Enut_Controller(p3t::IPM_SectionedParmFile &config, enut::imu_iface * imu, enut::angles_iface * angles );

    bool set_height( double height );
    std::pair<bool,double> get_height(){return {true,m_height}; }

    bool set_attitude( std::string a );
    std::pair<bool, std::string> get_attitude(){ return {true,enut::Attitude_to_string(m_attitude)}; }

    bool set_body_rpy( double roll, double pitch, double yaw);

private:
    p3t::IPM_SectionedParmFile &m_db;
    enut::imu_iface * m_imu;
    enut::angles_iface * m_angles;
    double m_height;
    enut::Attitude m_attitude;
    enut::Body body;

    double m_body_roll;
    double m_body_pitch;
    double m_body_yaw;

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

    void reset_ceres_angles();

};

#endif // ENUT_CONTROLLER_H
