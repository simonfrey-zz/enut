#include "enut_simulator.h"
#include "IPM_Calibration/ipm_ceres_geometry_fit.h"
Enut_Simulator::Enut_Simulator(Plot3D_Interface::shared_t p3d) :
    ipm::modules::module("simulator"),
    m_p3d( p3d )
{

    body.shoulders[HL].tr = {-0.038,-0.0685,0};
    body.shoulders[HR].tr = { 0.038,-0.0685,0};
    body.shoulders[FL].tr = {-0.038, 0.0685,0};
    body.shoulders[FR].tr = { 0.038, 0.0685,0};

    body.shoulders[HL].mirror = true;
    body.shoulders[HR].mirror = false;
    body.shoulders[FL].mirror = true;
    body.shoulders[FR].mirror = false;

    body.set_angles(m_ist_angles);

    m_ground_roll = 0;
    m_ground_pitch = 0;

    m_imu_roll = 0;
    m_imu_pitch = 0;

    m_speed = 1;

    start_helper_thread( &Enut_Simulator::draw, this );
    start_helper_thread( &Enut_Simulator::loop, this );
}

void Enut_Simulator::set_ground_angles(double roll, double pitch)
{
    m_ground_roll = roll;
    m_ground_pitch = pitch;
}

void Enut_Simulator::dbg_set_angles(double s_fl, double s_fr, double s_hl, double s_hr, double l_fl, double l_fr, double l_hl, double l_hr, double f_fl, double f_fr, double f_hl, double f_hr, double head)
{
    std::unique_lock<std::mutex> lock(m_mutex);

    m_soll_angles = {s_fl, s_fr, s_hl, s_hr,
                    l_fl, l_fr, l_hl, l_hr,
                     f_fl, f_fr, f_hl, f_hr, head};
}

enut::imu_iface::imu_data Enut_Simulator::get_imu()
{
    std::unique_lock<std::mutex> lock(m_mutex);
    enut::imu_iface::imu_data ret;
    ret.roll = m_imu_roll;
    ret.pitch = m_imu_pitch;
    return ret;
}

enut::Angles Enut_Simulator::get_angles()
{
    std::unique_lock<std::mutex> lock(m_mutex);
    return m_ist_angles;
}

void Enut_Simulator::set_angles(enut::Angles angles, double speed)
{
    std::unique_lock<std::mutex> lock(m_mutex);
    m_soll_angles = angles;
    m_speed = speed;
}

void Enut_Simulator::loop(){

    const double dt = 1.0/20.0;
    while (helper_in_normal_operation()) {
        {
            std::unique_lock<std::mutex> lock(m_mutex);

            move_angles( dt );

            // find for each legs the lowest joint
            std::vector<Eigen::Vector3d> lowest_points = {{0,0,110}, {0,0,110}, {0,0,110}, {0,0,110}};
            double min_z = 1000;
            auto update_lowest = []( Eigen::Vector3d &l, Eigen::Vector3d &p, double &min_z ) -> void {
                if( p[2] < l[2] ){
                    l = p;
                }
                if( l[2] < min_z )
                    min_z = l[2];
            };
            update_lowest(lowest_points[0], body.shoulders[FR].p, min_z);
            update_lowest(lowest_points[0], body.shoulders[FR].leg.p, min_z);
            update_lowest(lowest_points[0], body.shoulders[FR].leg.foot.p, min_z);

            update_lowest(lowest_points[1], body.shoulders[FL].p, min_z);
            update_lowest(lowest_points[1], body.shoulders[FL].leg.p, min_z);
            update_lowest(lowest_points[1], body.shoulders[FL].leg.foot.p, min_z);

            update_lowest(lowest_points[2], body.shoulders[HR].p, min_z);
            update_lowest(lowest_points[2], body.shoulders[HR].leg.p, min_z);
            update_lowest(lowest_points[2], body.shoulders[HR].leg.foot.p, min_z);

            update_lowest(lowest_points[3], body.shoulders[HL].p, min_z);
            update_lowest(lowest_points[3], body.shoulders[HL].leg.p, min_z);
            update_lowest(lowest_points[3], body.shoulders[HL].leg.foot.p, min_z);

            IPM_CeresPlaneFit fit_body(false);
            fit_body.add_points( lowest_points );
            fit_body.solve( true );

            // draw ground
            std::vector<Eigen::Vector3d> gp;
            gp.push_back( {-0.5,-0.5,0});
            gp.push_back( {0.5,-0.5,0});
            gp.push_back( {0.5,0.5,0});
            gp.push_back( {-0.5,0.5,0});

            Eigen::AngleAxisd aa_g_r( m_ground_roll, Eigen::Vector3d(0,1,0) );
            Eigen::AngleAxisd aa_g_p( m_ground_pitch, Eigen::Vector3d(1,0,0) );
            for( auto &p : gp ){
                p = aa_g_p._transformVector( aa_g_r._transformVector(p) );
            }

            IPM_CeresPlaneFit fit_gnd(false);
            fit_gnd.add_points( gp );
            fit_gnd.solve( true);

            body.q = fit_body.get_rotation() * fit_gnd.get_rotation();

            // translate body on plane
            Eigen::Vector3d p_offset(0,0,0);
            for( auto &p : lowest_points ){
                p_offset += body.q._transformVector(p);
            }
            body.tr = p_offset * -0.25;

            // imu
            Eigen::Vector3d roll = body.q._transformVector(Eigen::Vector3d(0,0,1));
            roll[1] = 0;
            roll.normalize();
            m_imu_roll = std::acos(roll.dot( Eigen::Vector3d(0,0,1) )) * 180.0 / M_PI;
            if( roll[0] < 0 )
                m_imu_roll = -m_imu_roll;

            Eigen::Vector3d pitch = body.q._transformVector(Eigen::Vector3d(0,0,1));
            pitch[0] = 0;
            pitch.normalize();
            m_imu_pitch = std::acos(pitch.dot( Eigen::Vector3d(0,0,1) )) * 180.0 / M_PI;
            if( pitch[1] > 0 )
                m_imu_pitch = -m_imu_pitch;

            PT_INFO( "roll " << m_imu_roll << ", pitch " << m_imu_pitch );

        }
        p3t::sleep(dt);
    }
}

void Enut_Simulator::draw()
{
    if(m_p3d.get()==nullptr)
        return;

    while( helper_in_normal_operation() ){

        {
            std::unique_lock<std::mutex> lock(m_mutex);
            m_p3d->clearAll();

            const Eigen::Vector3d draw_offset(0.5,0.5,0);
            //const Eigen::Vector3d draw_offset(0,0,0);

            // draw ground
            std::vector<Eigen::Vector3d> gp;
            gp.push_back( {-0.5,-0.5,0});
            gp.push_back( {0.5,-0.5,0});
            gp.push_back( {0.5,0.5,0});
            gp.push_back( {-0.5,0.5,0});

            Eigen::AngleAxisd aa_g_r( m_ground_roll, Eigen::Vector3d(0,1,0) );
            Eigen::AngleAxisd aa_g_p( m_ground_pitch, Eigen::Vector3d(1,0,0) );
            for( auto &p : gp ){
                p = aa_g_p._transformVector( aa_g_r._transformVector(p) ) + draw_offset;
            }
            m_p3d->addRect2(gp,PLOT3D_DARK_GRAY,0.5,"ground");

            m_p3d->addCoordinateSys(body.tr + draw_offset, body.q, 0.03, "body");

            Eigen::Vector3d ph = body.q._transformVector(body.head.tr) + body.tr + draw_offset;

            m_p3d->addLineFrom2Points( body.tr + draw_offset, ph, PLOT3D_GRAY, "joint");
            m_p3d->add3DPoint(ph, PLOT3D_RED, "shoulder");
            m_p3d->addCoordinateSys(ph, body.q*body.head.q, 0.05, "head");

            for( auto &it : body.shoulders ){
                auto &shoulder = it.second;
                Eigen::Vector3d p1 = body.q._transformVector(shoulder.tr) + body.tr + draw_offset;

                m_p3d->addLineFrom2Points( body.tr + draw_offset, p1, PLOT3D_GRAY, "joint");
                m_p3d->add3DPoint(p1, PLOT3D_RED, "shoulder");

                Eigen::Vector3d p2 = body.q._transformVector(shoulder.p) + body.tr + draw_offset;

                m_p3d->addLineFrom2Points( p1, p2, PLOT3D_GRAY, "joint");
                m_p3d->add3DPoint(p2, PLOT3D_RED, "shoulder");


                Eigen::Vector3d p3 = body.q._transformVector(shoulder.leg.p ) + body.tr + draw_offset;
                m_p3d->add3DPoint( p3, PLOT3D_RED, "leg" );
                m_p3d->addLineFrom2Points( p2, p3, PLOT3D_GRAY, "joint");

                Eigen::Vector3d p4 = body.q._transformVector(shoulder.leg.foot.p ) + body.tr + draw_offset;
                m_p3d->add3DPoint( p4, PLOT3D_MAGENTA, "foot" );
                m_p3d->addLineFrom2Points( p3, p4, PLOT3D_GRAY, "joint");

            }

            {
                Eigen::Vector3d p1 = body.q._transformVector(body.shoulders[FL].tr) + body.tr + draw_offset;
                Eigen::Vector3d p2 = body.q._transformVector(body.shoulders[FR].tr) + body.tr + draw_offset;
                m_p3d->addLineFrom2Points( p1, p2, PLOT3D_GRAY, "body");
            }
            {
                Eigen::Vector3d p1 = body.q._transformVector(body.shoulders[FL].tr) + body.tr + draw_offset;
                Eigen::Vector3d p2 = body.q._transformVector(body.shoulders[HL].tr) + body.tr + draw_offset;
                m_p3d->addLineFrom2Points( p1, p2, PLOT3D_GRAY, "body");
            }
            {
                Eigen::Vector3d p1 = body.q._transformVector(body.shoulders[FR].tr) + body.tr + draw_offset;
                Eigen::Vector3d p2 = body.q._transformVector(body.shoulders[HR].tr) + body.tr + draw_offset;
                m_p3d->addLineFrom2Points( p1, p2, PLOT3D_GRAY, "body");
            }
            {
                Eigen::Vector3d p1 = body.q._transformVector(body.shoulders[HL].tr) + body.tr + draw_offset;
                Eigen::Vector3d p2 = body.q._transformVector(body.shoulders[HR].tr) + body.tr + draw_offset;
                m_p3d->addLineFrom2Points( p1, p2, PLOT3D_GRAY, "body");
            }
        }
        p3t::sleep(0.1);

    }
}

void Enut_Simulator::move_angles(double dt)
{
    // 0.12 sec / 60 degree -> 500 deg/s
    const double max_angle_per_dt = m_speed * 400.0 * (M_PI/180.0) * dt;
    for( unsigned int i = 0; i < 13; i++ ){
        if( std::abs( m_soll_angles[i] - m_ist_angles[i] ) > max_angle_per_dt ){
            if( m_soll_angles[i] > m_ist_angles[i] )
                m_ist_angles[i] += max_angle_per_dt;
            else
                m_ist_angles[i] -= max_angle_per_dt;
        }
        else {
            m_ist_angles[i] = m_soll_angles[i];
        }
    }
    body.set_angles(m_ist_angles);
}
