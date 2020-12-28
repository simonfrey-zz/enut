#ifndef ENUT_H
#define ENUT_H

#include "IPM_Plot/plot3d_interface.h"
#include "Shared/enut_models.h"

class Enut
{
public:
    Enut();

    void set_plot_iface( Plot3D_Interface::shared_t p3d ){
        m_p3d = p3d;
    }

    void draw();

    void set_angles( double s_fl, double s_fr, double s_hl, double s_hr,
                     double l_fl, double l_fr, double l_hl, double l_hr,
                     double f_fl, double f_fr, double f_hl, double f_hr,
                     double head);

    std::vector<float> get_angles();

    void set_height( double height );


    void set_attitude( int v );

    void set_imu( double roll, double pitch );

private:
    Plot3D_Interface::shared_t m_p3d;



    enut::Attitude m_attitude;
    enut::Body body;
    double m_height;
    std::map<int,Eigen::Vector3d> m_foot_pose;
    double m_imu_roll;
    double m_imu_pitch;

    bool find_angles();




};

#endif // ENUT_H
