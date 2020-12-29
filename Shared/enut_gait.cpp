#include "enut_gait.h"

Enut_Gait::Enut_Gait()
{    
    m_gait_points.push_back({0,0.5,0});
    m_gait_points.push_back({0,0.409090909090909,	0});
    m_gait_points.push_back({0,0.318181818181818, 0});
    m_gait_points.push_back({0,0.227272727272727, 0});
    m_gait_points.push_back({0,0.136363636363636, 0});
    m_gait_points.push_back({0,0.045454545454545, 0});
    m_gait_points.push_back({0,-0.045454545454546,0});
    m_gait_points.push_back({0,-0.136363636363636,0});
    m_gait_points.push_back({0,-0.227272727272727,0});
    m_gait_points.push_back({0,-0.318181818181818,0});
    m_gait_points.push_back({0,-0.409090909090909,0});
    m_gait_points.push_back({0,-0.5,0});
    m_gait_points.push_back({0,-0.2,0.6});
    m_gait_points.push_back({0,0.5, 1});
    m_gait_points.push_back({0,0.6, 0.2});
    m_gait_points.push_back({0,0.5, 0});
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
