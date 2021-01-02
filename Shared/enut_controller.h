#ifndef ENUT_CONTROLLER_H
#define ENUT_CONTROLLER_H

#include "ModulesChain/Module.h"
#include "Shared/enut_ifaces.h"
#include "ceres/ceres.h"
#include "IPM_Control/IPM_PID.h"
#include "IPM_SCPI++/SCPIClassAdaptor.h"
#include "IPM_Parameter/IPM_SectionedParmFile.h"
#include <mutex>

class Enut_Controller: public ipm::modules::module, public SCPIClassAdaptor<Enut_Controller>
{
public:
    Enut_Controller(p3t::IPM_SectionedParmFile &config, enut::imu_iface * imu, enut::angles_iface * angles );

    bool set_height( double height );
    std::pair<bool,double> get_height(){return {true,m_height}; }

    bool set_attitude( std::string a );
    std::pair<bool, std::string> get_attitude(){ return {true,enut::Attitude_to_string(m_attitude)}; }

    bool set_body_rpy( double roll, double pitch, double yaw);
    bool set_gait_width( double width );
    bool set_gait_speed( double speed );

    bool set_foot_pose_calibration(unsigned id, double x, double y, double z );
    bool angle_mode_set_angle( unsigned id, double a );

    bool com_mode_legup( int leg );
    bool com_mode_shift( double x, double y );

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
    std::map<int, Eigen::Vector3d> m_shoulder_pose;
    std::map<int, enut::Enut_Pate_model*> m_pates_models;

    std::map<int, Eigen::Vector3d> m_foot_pose_calibration;

    IPM_PID * m_pid_roll;
    IPM_PID * m_pid_pitch;
    IPM_PID * m_pid_yaw;

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

    double m_gait_width;
    double m_gait_speed;

    std::mutex m_mutex;

    // angle mode
    enut::Angles m_angle_mode_angles;

    // com calib mode
    int m_com_mode_leg_up;
    Eigen::Vector3d m_com_mode_shift;

};

#endif // ENUT_CONTROLLER_H
