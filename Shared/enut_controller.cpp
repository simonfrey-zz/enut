#include "enut_controller.h"
#include "Shared/enut_models.h"

Enut_Controller::Enut_Controller(enut::imu_iface * imu, enut::angles_iface * angles) :
    ipm::modules::module("control"),
    m_imu( imu ),
    m_angles( angles ),
    problem( problem_options )
{
    body.shoulders[HL].tr = {-0.038,-0.0685,0};
    body.shoulders[HR].tr = { 0.038,-0.0685,0};
    body.shoulders[FL].tr = {-0.038, 0.0685,0};
    body.shoulders[FR].tr = { 0.038, 0.0685,0};

    body.shoulders[HL].mirror = true;
    body.shoulders[HR].mirror = false;
    body.shoulders[FL].mirror = true;
    body.shoulders[FR].mirror = false;

    m_attitude = enut::relax;

    m_height = 0.08;

    init_ceres();

    start_helper_thread( &Enut_Controller::loop, this );
}

void Enut_Controller::set_height(double height)
{
    m_height = std::min( std::max( height, ENUT_MIN_HEIGHT), ENUT_MAX_HEIGHT);
}

void Enut_Controller::set_attitude(enut::Attitude a)
{
    m_attitude = a;
}

void Enut_Controller::loop()
{
    const double dt = 1.0/10.0;

    while (helper_in_normal_operation()) {

        p3t::sleep(dt);

        if( m_attitude == enut::relax ){
            enut::Angles def_angles;
            m_angles->set_angles( def_angles, 0.2 );
            continue;
        }
        else if( m_attitude == enut::standing ){

            auto imu = m_imu->get_imu();

            body.head.angle = (90-imu.pitch)*M_PI/180.0;

            // put foot points
            m_foot_pose[FL] = {0.01,0,0};
            m_foot_pose[FR] = {0.01,0,0};
            m_foot_pose[HR] = {0.01,0,0};
            m_foot_pose[HL] = {0.01,0,0};

            m_foot_pose[FL][2] -= m_height;
            m_foot_pose[FR][2] -= m_height;
            m_foot_pose[HR][2] -= m_height;
            m_foot_pose[HL][2] -= m_height;

            m_pates_models[FL]->set_foot_final( m_foot_pose[FL] );
            m_pates_models[FR]->set_foot_final( m_foot_pose[FR] );
            m_pates_models[HL]->set_foot_final( m_foot_pose[HL] );
            m_pates_models[HR]->set_foot_final( m_foot_pose[HR] );

            ceres::Solve(options, &problem, &summary);

            m_angles->set_angles( {shoulder_angles[FL], shoulder_angles[FR],shoulder_angles[HL],shoulder_angles[HR],
                                  leg_angles[FL], leg_angles[FR],leg_angles[HL],leg_angles[HR],
                                  foot_angles[FL], foot_angles[FR],foot_angles[HL],foot_angles[HR],
                                  body.head.angle}, 1 );
        }


    }
}

void Enut_Controller::init_ceres()
{
    foot_angles[FL] = M_PI;
    foot_angles[FR] = M_PI;
    foot_angles[HL] = M_PI;
    foot_angles[HR] = M_PI;

    leg_angles[FL] = M_PI;
    leg_angles[FR] = M_PI;
    leg_angles[HL] = M_PI;
    leg_angles[HR] = M_PI;

    shoulder_angles[FL] = M_PI;
    shoulder_angles[FR] = M_PI;
    shoulder_angles[HL] = M_PI;
    shoulder_angles[HR] = M_PI;

    m_pates_models[FL] = new enut::Enut_Pate_model( m_foot_pose[FL] );
    m_pates_models[FR] = new enut::Enut_Pate_model( m_foot_pose[FR] );
    m_pates_models[HL] = new enut::Enut_Pate_model( m_foot_pose[HL] );
    m_pates_models[HR] = new enut::Enut_Pate_model( m_foot_pose[HR] );

    problem.AddResidualBlock( new ceres::AutoDiffCostFunction<enut::Enut_Pate_model, 3, 1, 1, 1>(
                                  m_pates_models[FL] ),
                              new ceres::CauchyLoss(loss_size),
                              &shoulder_angles[FL],
                              &leg_angles[FL],
                              &foot_angles[FL]
                              );

    problem.AddResidualBlock( new ceres::AutoDiffCostFunction<enut::Enut_Pate_model, 3, 1, 1, 1>(
                                  m_pates_models[FR] ),
                              new ceres::CauchyLoss(loss_size),
                              &shoulder_angles[FR],
                              &leg_angles[FR],
                              &foot_angles[FR]
                              );

    problem.AddResidualBlock( new ceres::AutoDiffCostFunction<enut::Enut_Pate_model, 3, 1, 1, 1>(
                                  m_pates_models[HL] ),
                              new ceres::CauchyLoss(loss_size),
                              &shoulder_angles[HL],
                              &leg_angles[HL],
                              &foot_angles[HL]
                              );

    problem.AddResidualBlock( new ceres::AutoDiffCostFunction<enut::Enut_Pate_model, 3, 1, 1, 1>(
                                  m_pates_models[HR] ),
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
}
