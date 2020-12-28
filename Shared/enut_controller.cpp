#include "enut_controller.h"
#include "Shared/enut_models.h"

Enut_Controller::Enut_Controller(p3t::IPM_SectionedParmFile &config, enut::imu_iface * imu, enut::angles_iface * angles) :
    ipm::modules::module("control"),
    SCPIClassAdaptor<Enut_Controller>(this,"controller"),
    m_db( config ),
    m_imu( imu ),
    m_angles( angles ),
    problem( problem_options )
{
    body.shoulders[HL].tr = {-LEGS_SPAN_X*0.5,-LEGS_SPAN_Y,0.};
    body.shoulders[HR].tr = { LEGS_SPAN_X*0.5,-LEGS_SPAN_Y,0.};
    body.shoulders[FL].tr = {-LEGS_SPAN_X*0.5, LEGS_SPAN_Y,0.};
    body.shoulders[FR].tr = { LEGS_SPAN_X*0.5, LEGS_SPAN_Y,0.};

    body.shoulders[HL].mirror = true;
    body.shoulders[HR].mirror = false;
    body.shoulders[FL].mirror = true;
    body.shoulders[FR].mirror = false;

    m_attitude = enut::relax;

    m_height = 0.12;

    m_body_roll = 0;
    m_body_pitch = 0;
    m_body_yaw = 0;

    init_ceres();

    m_db.lock();
    const double kp = m_db.getd(".PID.KP", 0.0 );
    const double ki = m_db.getd(".PID.KI", 0.0 );
    const double kd = m_db.getd(".PID.KD", 0.0 );
    m_pid_roll = new IPM_PID( kp, ki, kd );
    m_pid_pitch = new IPM_PID( kp, ki, kd );
    m_db.unlock();

    addCommand( {"CTRL","HEIGHT"}, &Enut_Controller::set_height, &Enut_Controller::get_height);
    addCommand( {"CTRL","ATTITUDE"}, &Enut_Controller::set_attitude, &Enut_Controller::get_attitude);
    addCommandWriteOnly( {"CTRL", "RPY"}, &Enut_Controller::set_body_rpy );

    start_helper_thread( &Enut_Controller::loop, this );
}

bool Enut_Controller::set_height(double height)
{
    m_height = std::min( std::max( height, ENUT_MIN_HEIGHT), ENUT_MAX_HEIGHT);
    return true;
}

bool Enut_Controller::set_attitude(std::string a)
{
    PT_INFO("set attitude " << a );
    m_attitude = enut::Attitude_from_string(a);
    return true;
}

bool Enut_Controller::set_body_rpy(double roll, double pitch, double yaw)
{
    m_body_roll = roll;
    m_body_pitch = pitch;
    m_body_yaw = yaw;
    return true;
}


void Enut_Controller::loop()
{
    const double dt = 1.0/30.0;

    double speed = 0.2;

    while (helper_in_normal_operation()) {

        p3t::sleep(dt);
        if( speed > 1 )
            speed = 1;
        if( speed < 0.2 )
            speed = 0.2;

        auto imu = m_imu->get_imu();

        if( std::abs(imu.pitch) > 60 || std::abs(imu.roll) > 60 ){
            m_attitude = enut::relax;
        }

        if( m_attitude == enut::relax ){
            speed = 0.2;
            enut::Angles def_angles;
            m_angles->set_angles( def_angles, speed );
            reset_ceres_angles();
            continue;
        }
        else if( m_attitude == enut::standing ){

            auto imu = m_imu->get_imu();

            const double pid_roll = m_pid_roll->control( 0, sin(imu.roll*M_PI/180.0)*LEGS_SPAN_X*0.5 );
            const double pid_pitch = m_pid_pitch->control( 0, sin(imu.pitch*M_PI/180.0)*LEGS_SPAN_Y*0.5 );


            body.head.angle = (90-imu.pitch)*M_PI/180.0;

            // put foot points
            m_foot_pose[FL] = {0.02,0.03,0};
            m_foot_pose[FR] = {0.02,0.03,0};
            m_foot_pose[HR] = {0.02,0,0};
            m_foot_pose[HL] = {0.02,0,0};

            m_foot_pose[FL][2] -= m_height - pid_roll + pid_pitch;
            m_foot_pose[FR][2] -= m_height + pid_roll + pid_pitch;
            m_foot_pose[HR][2] -= m_height + pid_roll - pid_pitch;
            m_foot_pose[HL][2] -= m_height - pid_roll - pid_pitch;


            m_pates_models[FL]->set_foot_final( m_foot_pose[FL] );
            m_pates_models[FR]->set_foot_final( m_foot_pose[FR] );
            m_pates_models[HL]->set_foot_final( m_foot_pose[HL] );
            m_pates_models[HR]->set_foot_final( m_foot_pose[HR] );

            ceres::Solve(options, &problem, &summary);

            m_angles->set_angles( {shoulder_angles[FL], shoulder_angles[FR],shoulder_angles[HL],shoulder_angles[HR],
                                  leg_angles[FL], leg_angles[FR],leg_angles[HL],leg_angles[HR],
                                  foot_angles[FL], foot_angles[FR],foot_angles[HL],foot_angles[HR],
                                  body.head.angle}, speed );

            if( speed < 1 ){
                speed += 0.005;
            }
        }


    }

}

void Enut_Controller::reset_ceres_angles(){

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

}

void Enut_Controller::init_ceres()
{

    reset_ceres_angles();

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
