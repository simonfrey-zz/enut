#include "enut.h"
#include <thread>

Enut::Enut()
{
    body.shoulders[HL].tr = {-0.038,-0.0685,0};
    body.shoulders[HR].tr = { 0.038,-0.0685,0};
    body.shoulders[FL].tr = {-0.038, 0.0685,0};
    body.shoulders[FR].tr = { 0.038, 0.0685,0};

    body.shoulders[HL].mirror = true;
    body.shoulders[HR].mirror = false;
    body.shoulders[FL].mirror = true;
    body.shoulders[FR].mirror = false;

    set_angles(M_PI_2,M_PI_2,M_PI_2,M_PI_2,M_PI_2,M_PI_2,M_PI_2,M_PI_2,M_PI_2,M_PI_2,M_PI_2,M_PI_2,M_PI_2);
    m_attitude = enut::relax;

    m_foot_pose[HL] = {0,0,0};
    m_foot_pose[FL] = {0,0,0};
    m_foot_pose[HR] = {0,0,0};
    m_foot_pose[FR] = {0,0,0};

    m_height = 0.12;
    m_imu_roll = 0;
    m_imu_pitch = 0;
}

void Enut::draw()
{

}

void Enut::set_angles(double s_fl, double s_fr, double s_hl, double s_hr,
                      double l_fl, double l_fr, double l_hl, double l_hr,
                      double f_fl, double f_fr, double f_hl, double f_hr, double head){
    enut::Angles a = {s_fl, s_fr, s_hl, s_hr,
                      l_fl, l_fr, l_hl, l_hr,
                      f_fl, f_fr, f_hl, f_hr, head};
    body.set_angles(a);
}

std::vector<float> Enut::get_angles()
{
    std::vector<float> ret;
    ret.push_back( body.shoulders[FL].leg.foot.angle );
    ret.push_back( body.shoulders[FL].leg.angle );
    ret.push_back( body.shoulders[FL].angle );

    ret.push_back( body.shoulders[HL].leg.foot.angle );
    ret.push_back( body.shoulders[HL].leg.angle );
    ret.push_back( body.shoulders[HL].angle );

    ret.push_back( body.shoulders[HR].leg.foot.angle );
    ret.push_back( body.shoulders[HR].leg.angle );
    ret.push_back( body.shoulders[HR].angle );

    ret.push_back( body.shoulders[FR].leg.foot.angle );
    ret.push_back( body.shoulders[FR].leg.angle );
    ret.push_back( body.shoulders[FR].angle );

    ret.push_back( body.head.angle );

    return ret;
}

void Enut::set_height(double height)
{
    m_height = height;
    find_angles();
}

void Enut::set_attitude(int v)
{
    m_attitude = (enut::Attitude)v;
    find_angles();
}

void Enut::set_imu(double roll, double pitch)
{
    m_imu_roll = roll * M_PI / 180.0;
    m_imu_pitch = pitch * M_PI / 180.0;

    find_angles();
}

bool Enut::find_angles()
{


    if( m_attitude == enut::relax ){
        set_angles(M_PI_2,M_PI_2,M_PI_2,M_PI_2,M_PI_2,M_PI_2,M_PI_2,M_PI_2,M_PI_2,M_PI_2,M_PI_2,M_PI_2,M_PI_2);
        draw();
        return true;
    }

    std::map<int, Eigen::Vector3d> foot_pose;

    if( m_attitude == enut::standing ){


        // put foot points
        foot_pose[FL] = {0.01,0,0};
        foot_pose[FR] = {0.01,0,0};
        foot_pose[HR] = {0.01,0,0};
        foot_pose[HL] = {0.01,0,0};

        foot_pose[FL] += body.shoulders[FL].tr;
        foot_pose[FR] += body.shoulders[FR].tr;
        foot_pose[HR] += body.shoulders[HR].tr;
        foot_pose[HL] += body.shoulders[HL].tr;

        Eigen::AngleAxisd aa_roll( m_imu_roll, Eigen::Vector3d(0,1,0) );
        foot_pose[FL] = aa_roll._transformVector( foot_pose[FL] );
        foot_pose[FR] = aa_roll._transformVector( foot_pose[FR] );
        foot_pose[HR] = aa_roll._transformVector( foot_pose[HR] );
        foot_pose[HL] = aa_roll._transformVector( foot_pose[HL] );

        Eigen::AngleAxisd aa_pitch( m_imu_pitch, Eigen::Vector3d(1,0,0) );
        foot_pose[FL] = aa_pitch._transformVector( foot_pose[FL] );
        foot_pose[FR] = aa_pitch._transformVector( foot_pose[FR] );
        foot_pose[HR] = aa_pitch._transformVector( foot_pose[HR] );
        foot_pose[HL] = aa_pitch._transformVector( foot_pose[HL] );

        foot_pose[FL] -= body.shoulders[FL].tr;
        foot_pose[FR] -= body.shoulders[FR].tr;
        foot_pose[HR] -= body.shoulders[HR].tr;
        foot_pose[HL] -= body.shoulders[HL].tr;

        foot_pose[FL][2] -= m_height;
        foot_pose[FR][2] -= m_height;
        foot_pose[HR][2] -= m_height;
        foot_pose[HL][2] -= m_height;

    }

    std::map<int, double> foot_angles;
    foot_angles[FL] = M_PI;
    foot_angles[FR] = M_PI;
    foot_angles[HL] = M_PI;
    foot_angles[HR] = M_PI;

    std::map<int, double> leg_angles;
    leg_angles[FL] = M_PI;
    leg_angles[FR] = M_PI;
    leg_angles[HL] = M_PI;
    leg_angles[HR] = M_PI;

    std::map<int, double> shoulder_angles;
    shoulder_angles[FL] = M_PI;
    shoulder_angles[FR] = M_PI;
    shoulder_angles[HL] = M_PI;
    shoulder_angles[HR] = M_PI;

    ceres::Problem::Options problem_options;
    ceres::Problem problem(problem_options);

    double loss_size = 0.05;

    problem.AddResidualBlock( new ceres::AutoDiffCostFunction<enut::Enut_Pate_model, 3, 1, 1, 1>(
                                  new enut::Enut_Pate_model( foot_pose[FL] ) ),
                              new ceres::CauchyLoss(loss_size),
                              &shoulder_angles[FL],
                              &leg_angles[FL],
                              &foot_angles[FL]
                              );

    problem.AddResidualBlock( new ceres::AutoDiffCostFunction<enut::Enut_Pate_model, 3, 1, 1, 1>(
                                  new enut::Enut_Pate_model( foot_pose[FR] ) ),
                              new ceres::CauchyLoss(loss_size),
                              &shoulder_angles[FR],
                              &leg_angles[FR],
                              &foot_angles[FR]
                              );

    problem.AddResidualBlock( new ceres::AutoDiffCostFunction<enut::Enut_Pate_model, 3, 1, 1, 1>(
                                  new enut::Enut_Pate_model( foot_pose[HL] ) ),
                              new ceres::CauchyLoss(loss_size),
                              &shoulder_angles[HL],
                              &leg_angles[HL],
                              &foot_angles[HL]
                              );

    problem.AddResidualBlock( new ceres::AutoDiffCostFunction<enut::Enut_Pate_model, 3, 1, 1, 1>(
                                  new enut::Enut_Pate_model( foot_pose[HR] ) ),
                              new ceres::CauchyLoss(loss_size),
                              &shoulder_angles[HR],
                              &leg_angles[HR],
                              &foot_angles[HR]
                              );

    // limits
    problem.SetParameterLowerBound( &shoulder_angles[HR], 0, 0 );
    problem.SetParameterUpperBound( &shoulder_angles[HR], 0, 105*M_PI/180. );

    problem.SetParameterLowerBound( &shoulder_angles[HL], 0, 0 );
    problem.SetParameterUpperBound( &shoulder_angles[HL], 0, 105*M_PI/180. );

    problem.SetParameterLowerBound( &shoulder_angles[FR], 0, 0 );
    problem.SetParameterUpperBound( &shoulder_angles[FR], 0, 105*M_PI/180. );

    problem.SetParameterLowerBound( &shoulder_angles[FL], 0, 0 );
    problem.SetParameterUpperBound( &shoulder_angles[FL], 0, 105*M_PI/180. );

    problem.SetParameterLowerBound( &leg_angles[HR], 0, 0 );
    problem.SetParameterUpperBound( &leg_angles[HR], 0, 180*M_PI/180. );

    problem.SetParameterLowerBound( &leg_angles[HL], 0, 0 );
    problem.SetParameterUpperBound( &leg_angles[HL], 0, 180*M_PI/180. );

    problem.SetParameterLowerBound( &leg_angles[FR], 0, 0 );
    problem.SetParameterUpperBound( &leg_angles[FR], 0, 180*M_PI/180. );

    problem.SetParameterLowerBound( &leg_angles[FL], 0, 0 );
    problem.SetParameterUpperBound( &leg_angles[FL], 0, 180*M_PI/180. );


    problem.SetParameterLowerBound( &foot_angles[HR], 0, 0 );
    problem.SetParameterUpperBound( &foot_angles[HR], 0, 180*M_PI/180. );

    problem.SetParameterLowerBound( &foot_angles[HL], 0, 0 );
    problem.SetParameterUpperBound( &foot_angles[HL], 0, 180*M_PI/180. );

    problem.SetParameterLowerBound( &foot_angles[FR], 0, 0 );
    problem.SetParameterUpperBound( &foot_angles[FR], 0, 180*M_PI/180. );

    problem.SetParameterLowerBound( &foot_angles[FL], 0, 0 );
    problem.SetParameterUpperBound( &foot_angles[FL], 0, 180*M_PI/180. );

    // Configure the solver.
    ceres::Solver::Options options;
    options.num_threads = std::thread::hardware_concurrency();
    options.linear_solver_type = ceres::DENSE_SCHUR;
    options.max_num_iterations = 20;
    options.max_solver_time_in_seconds=0.1;
    options.minimizer_progress_to_stdout = true;

    options.parameter_tolerance = 1e-16;
    options.gradient_tolerance = 1e-16;
    options.function_tolerance = 1e-16;

    // Solve!
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    std::cout << summary.FullReport() << std::endl;

    set_angles(shoulder_angles[FL], shoulder_angles[FR],shoulder_angles[HL],shoulder_angles[HR],
               leg_angles[FL], leg_angles[FR],leg_angles[HL],leg_angles[HR],
               foot_angles[FL], foot_angles[FR],foot_angles[HL],foot_angles[HR],
               body.head.angle
               );

    draw();
    return summary.IsSolutionUsable();

}
