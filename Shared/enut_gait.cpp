#include "enut_gait.h"

Enut_Gait::Enut_Gait()
{
    // 3/4
#if 0
    m_gait_points.push_back({0, 0.5000, 0});
    m_gait_points.push_back({0, 0.4166, 0});
    m_gait_points.push_back({0, 0.3333, 0});
    m_gait_points.push_back({0, 0.2500, 0});
    m_gait_points.push_back({0, 0.1666, 0});
    m_gait_points.push_back({0, 0.0833, 0});
    m_gait_points.push_back({0, 0.0000, 0});
    m_gait_points.push_back({0,-0.0833, 0});
    m_gait_points.push_back({0,-0.1666, 0});
    m_gait_points.push_back({0,-0.2500, 0});
    m_gait_points.push_back({0,-0.3333, 0});
    m_gait_points.push_back({0,-0.4166, 0});
    m_gait_points.push_back({0,-0.5000, 0});
    m_gait_points.push_back({0,-0.4000, 0.6});
    m_gait_points.push_back({0, 0.5000, 1.0});
    m_gait_points.push_back({0, 0.6000, 0.2});
#else
    // 1 / 16
    m_gait_points.push_back({0, 0.5000, 0});
    m_gait_points.push_back({0, 0.4285, 0});
    m_gait_points.push_back({0, 0.3571, 0});
    m_gait_points.push_back({0, 0.2857, 0});
    m_gait_points.push_back({0, 0.2142, 0});
    m_gait_points.push_back({0, 0.1428, 0});
    m_gait_points.push_back({0, 0.0714, 0});
    m_gait_points.push_back({0, 0.0000, 0});
    m_gait_points.push_back({0,-0.0714, 0});
    m_gait_points.push_back({0,-0.1428, 0});
    m_gait_points.push_back({0,-0.2142, 0});
    m_gait_points.push_back({0,-0.2857, 0});
    m_gait_points.push_back({0,-0.3571, 0});
    m_gait_points.push_back({0,-0.4285, 0});
    m_gait_points.push_back({0,-0.5000, 0});
    m_gait_points.push_back({0, 0.0, 1});
#endif
}

Eigen::Vector3d Enut_Gait::get(double step, double width, double height)
{
    double frac = std::fmod( step, 1.0/16.0 ) * 16.0;
    unsigned id = step * 16.0;

    Eigen::Vector3d p = (m_gait_points[ (id % 16) ] * ( 1.0 - frac ) + m_gait_points[ (id+1) % 16 ] * (frac));
    p[2] *= height;
    p[1] *= width;
    return p;
}

double Enut_Gait::touch(double step, double, double)
{
    double frac = std::fmod( step, 1.0/16.0 ) * 16.0;
    unsigned id = step * 16.0;

    Eigen::Vector3d p = (m_gait_points[ (id % 16) ] * ( 1.0 - frac ) + m_gait_points[ (id+1) % 16 ] * (frac));
    return p[2] > 0.001 ? 1 : 0;
}
