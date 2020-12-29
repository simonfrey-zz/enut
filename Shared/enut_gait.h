#ifndef ENUT_GAIT_H
#define ENUT_GAIT_H

#include <Eigen/Eigen>

class Enut_Gait
{
public:
    Enut_Gait();

    Eigen::Vector3d get( double step, double width, double height );

private:
    std::vector<Eigen::Vector3d> m_gait_points;
};

#endif // ENUT_GAIT_H
