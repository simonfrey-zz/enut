#include "enut_controller.h"
#include "Shared/enut_models.h"
#include "Shared/enut_gait.h"

#define IMU_ROLL_OFFSET (2.5*M_PI/180.0)

Enut_Controller::Enut_Controller(p3t::IPM_SectionedParmFile &config, enut::imu_iface * imu, enut::angles_iface * angles, enut::sonar_iface * sonar) :
    ipm::modules::module("control"),
    SCPIClassAdaptor<Enut_Controller>(this,"controller"),
    m_db( config ),
    m_imu( imu ),
    m_angles( angles ),
    m_sonar( sonar ),
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

    m_body_roll = 0;
    m_body_pitch = 0;
    m_body_yaw = 0;

    m_gait_width = 0;
    m_gait_speed = 0;
    m_gait_direction = 0;
    m_gait_steering = 0;

    m_com_mode_leg_up = -1;
    m_com_mode_shift = Eigen::Vector3d(0,0,0);

    m_height = 0.12;

    m_db.lock();
    m_foot_pose_calibration[FL] = Eigen::Vector3d(m_db.getd("FOOT_FL_offset_x",0.0), m_db.getd("FOOT_FL_offset_y",0.0), m_db.getd("FOOT_FL_offset_z",0.0));
    m_foot_pose_calibration[FR] = Eigen::Vector3d(m_db.getd("FOOT_FR_offset_x",0.0), m_db.getd("FOOT_FR_offset_y",0.0), m_db.getd("FOOT_FR_offset_z",0.0));
    m_foot_pose_calibration[HL] = Eigen::Vector3d(m_db.getd("FOOT_HL_offset_x",0.0), m_db.getd("FOOT_HL_offset_y",0.0), m_db.getd("FOOT_HL_offset_z",0.0));
    m_foot_pose_calibration[HR] = Eigen::Vector3d(m_db.getd("FOOT_HR_offset_x",0.0), m_db.getd("FOOT_HR_offset_y",0.0), m_db.getd("FOOT_HR_offset_z",0.0));
    m_db.unlock();

    init_ceres();

    reset_pids();

    addCommand( {"CTRL","HEIGHT"}, &Enut_Controller::set_height, &Enut_Controller::get_height);
    addCommand( {"CTRL","ATTITUDE"}, &Enut_Controller::set_attitude, &Enut_Controller::get_attitude);
    addCommandWriteOnly( {"CTRL", "RPY"}, &Enut_Controller::set_body_rpy );

    addCommandWriteOnly( {"CTRL", "GAIT", "WIDTH"}, &Enut_Controller::set_gait_width);
    addCommandWriteOnly( {"CTRL", "GAIT", "SPEED"}, &Enut_Controller::set_gait_speed);
    addCommandWriteOnly( {"CTRL", "GAIT", "DIR"}, &Enut_Controller::set_gait_direction);
    addCommandWriteOnly( {"CTRL", "GAIT", "STEERING"}, &Enut_Controller::set_gait_steering);

    addCommandWriteOnly( {"CTRL", "CALIB"}, &Enut_Controller::set_foot_pose_calibration);

    addCommandWriteOnly( {"CTRL", "SET_ANGLE"}, &Enut_Controller::angle_mode_set_angle);

    addCommandWriteOnly( {"CTRL", "COM", "LEGUP"}, &Enut_Controller::com_mode_legup );
    addCommandWriteOnly( {"CTRL", "COM", "SHIFT"}, &Enut_Controller::com_mode_shift );

    start_helper_thread( &Enut_Controller::loop, this );
}

bool Enut_Controller::set_height(double height)
{
    std::unique_lock<std::mutex> lock(m_mutex);
    m_height = std::min( std::max( height, ENUT_MIN_HEIGHT), ENUT_MAX_HEIGHT);
    return true;
}

bool Enut_Controller::set_attitude(std::string a)
{
    std::unique_lock<std::mutex> lock(m_mutex);
    PT_INFO("set attitude " << a );
    m_attitude = enut::Attitude_from_string(a);
    return true;
}

bool Enut_Controller::set_body_rpy(double roll, double pitch, double yaw)
{
    std::unique_lock<std::mutex> lock(m_mutex);
    m_body_roll = roll;
    m_body_pitch = pitch;
    m_body_yaw = yaw;
    return true;
}

bool Enut_Controller::set_gait_width(double width)
{
    std::unique_lock<std::mutex> lock(m_mutex);
    m_gait_width = width;
    return true;
}

bool Enut_Controller::set_gait_speed(double speed)
{
    std::unique_lock<std::mutex> lock(m_mutex);
    m_gait_speed = speed;
    return true;
}

bool Enut_Controller::set_gait_direction(double angle)
{
    std::unique_lock<std::mutex> lock(m_mutex);
    m_gait_direction = angle;
    return true;
}

bool Enut_Controller::set_gait_steering(double angle)
{
    std::unique_lock<std::mutex> lock(m_mutex);
    m_gait_steering = angle;
    return true;
}

bool Enut_Controller::set_foot_pose_calibration(unsigned id, double x, double y, double z)
{
    std::unique_lock<std::mutex> lock(m_mutex);
    m_foot_pose_calibration[id] = Eigen::Vector3d(x,y,z);
    return true;
}

bool Enut_Controller::angle_mode_set_angle(unsigned id, double a)
{
    std::unique_lock<std::mutex> lock(m_mutex);
    if( id >= 13 )
        return false;
    m_angle_mode_angles[id] = a;
    PT_INFO("set " << id << " to " << a );
    return true;
}

bool Enut_Controller::com_mode_legup(int leg)
{
    std::unique_lock<std::mutex> lock(m_mutex);
    m_com_mode_leg_up = leg;
    return true;
}

bool Enut_Controller::com_mode_shift(double x, double y)
{
    std::unique_lock<std::mutex> lock(m_mutex);
    m_com_mode_shift = Eigen::Vector3d(x,y,0);
    return true;
}

void Enut_Controller::reset_pids(){

    delete m_pid_roll;
    delete m_pid_pitch;
    delete m_pid_yaw;

    m_db.lock();
    const double kp = m_db.getd(".PID.KP", 0.0 );
    const double ki = m_db.getd(".PID.KI", 0.0 );
    const double kd = m_db.getd(".PID.KD", 0.0 );
    m_pid_roll = new IPM_PID( kp, ki, kd );
    m_pid_pitch = new IPM_PID( kp, ki, kd );
    m_db.unlock();

    m_db.lock();
    m_pid_yaw = new IPM_PID( m_db.getd(".YAW_PID.KP", 0.0 ),
                             m_db.getd(".YAW_PID.KI", 0.3 ),
                             m_db.getd(".YAW_PID.KD", 0.0 )
                             );
    m_db.unlock();

}

void Enut_Controller::loop()
{
    const double dt = 1.0/30.0;

    double speed = 0.2;

    enut::Attitude prev_attitude = m_attitude;
    p3t::PTS_Timer<p3t::PTS_TSCount> timer;
    timer.start();

    while (helper_in_normal_operation() ) {


        p3t::sleep(dt);

        {
            std::unique_lock<std::mutex> lock(m_mutex);

            auto imu = m_imu->get_imu();

            if( prev_attitude != m_attitude ){

                PT_INFO("switch to state " << enut::Attitude_to_string(m_attitude) );

                timer.restart();

                if( prev_attitude == enut::relax ){
                    m_angles->turn_on();
                    enut::Angles def_angles;
                    m_angles->set_angles( def_angles, speed );
                    p3t::sleep(1);
                }

                if( m_attitude == enut::standing ){
                    reset_pids();
                }
            }
            prev_attitude = m_attitude;

            if( speed > 1 )
                speed = 1;
            if( speed < 0.001 )
                speed = 0.001;

            if( std::abs(imu.pitch) > 60 || std::abs(imu.roll) > 60 ){
                m_attitude = enut::relax;
                continue;
            }

            if( m_attitude == enut::relax ){
                speed = 0.1;

                reset_ceres_angles();

                if( timer.elapsed() > 3 ){
                    m_angles->turn_off();
                }
                else {
                    enut::Angles def_angles;
                    m_angles->set_angles( def_angles, speed );
                }

                continue;
            }
            else if( m_attitude == enut::standing || m_attitude == enut::walking ){

                double pid_roll = m_pid_roll->control( sin(m_body_roll + IMU_ROLL_OFFSET)*LEGS_SPAN_X*0.5,
                                                             sin(imu.roll*M_PI/180.0)*LEGS_SPAN_X*0.5 );

                double pid_pitch = m_pid_pitch->control( sin(m_body_pitch)*LEGS_SPAN_Y*0.5,
                                                               sin(imu.pitch*M_PI/180.0)*LEGS_SPAN_Y*0.5 );

                double pid_yaw = m_pid_yaw->control( 0, imu.yaw );

                // limit pid when walking
                if( m_attitude == enut::walking ){
                    pid_roll *= 0.0;//0.05;
                    pid_pitch *= 0.0;//0.05;
                    pid_yaw *= 0.0;//0.05;
                }

                body.head.angle = (90-imu.pitch)*M_PI/180.0;

                // obstacle
                double gait_speed_local = m_gait_speed;
                if( m_sonar->get_distance() < 0.3 ){
                    double p = m_sonar->get_distance() - 0.15;
                    if( p < 0 ){
                        gait_speed_local = p * 6;
                    }
                    else {
                        gait_speed_local = (p/0.15)  * m_gait_speed;
                    }
                }

                Enut_Gait gait;
                static double gait_step = 0;
                gait_step += 0.05*gait_speed_local;

                m_gait_width = std::min( 0.06, ((std::abs(gait_speed_local) / 0.3)*0.06));
                const double gait_height = std::min( 0.02, ((std::abs(gait_speed_local) / 0.2)*0.015));

                body.head.angle = (90-imu.pitch)*M_PI/180.0;

                const double step_height = (m_attitude == enut::walking) ? gait_height : 0;
                const double step_width = (m_attitude == enut::walking) ? m_gait_width : 0;

                // gait direction
                std::map<int, Eigen::Vector3d> m_foot_gait;
                Eigen::AngleAxisd aa_gait_dir = Eigen::AngleAxisd( m_gait_direction, Eigen::Vector3d(0,0,1));
                Eigen::AngleAxisd aa_gait_steering = Eigen::AngleAxisd( m_gait_steering, Eigen::Vector3d(0,0,1));
                //m_foot_gait[FL] = aa_gait_steering._transformVector( aa_gait_dir._transformVector(gait.get(gait_step+0.00, step_width, step_height)));
                //m_foot_gait[FR] = aa_gait_steering._transformVector( aa_gait_dir._transformVector(gait.get(gait_step+0.50, step_width, step_height)));
                //m_foot_gait[HL] = aa_gait_steering.inverse()._transformVector( aa_gait_dir._transformVector(gait.get(gait_step+0.25, step_width, step_height)));
                //m_foot_gait[HR] = aa_gait_steering.inverse()._transformVector( aa_gait_dir._transformVector(gait.get(gait_step+0.75, step_width, step_height)));

                m_foot_gait[FL] = aa_gait_steering._transformVector( aa_gait_dir._transformVector(gait.get(gait_step+0.00, step_width, step_height)));
                m_foot_gait[FR] = aa_gait_steering._transformVector( aa_gait_dir._transformVector(gait.get(gait_step+0.50, step_width, step_height)));
                m_foot_gait[HL] = aa_gait_steering.inverse()._transformVector( aa_gait_dir._transformVector(gait.get(gait_step+0.50, step_width, step_height)));
                m_foot_gait[HR] = aa_gait_steering.inverse()._transformVector( aa_gait_dir._transformVector(gait.get(gait_step+0.00, step_width, step_height)));

                // put foot points
                m_foot_pose[FL] = m_foot_pose_calibration[FL] + m_foot_gait[FL] + body.shoulders[FL].tr;
                m_foot_pose[FR] = m_foot_pose_calibration[FR] + m_foot_gait[FR] + body.shoulders[FR].tr;
                m_foot_pose[HL] = m_foot_pose_calibration[HL] + m_foot_gait[HL] + body.shoulders[HL].tr;
                m_foot_pose[HR] = m_foot_pose_calibration[HR] + m_foot_gait[HR] + body.shoulders[HR].tr;

                m_foot_pose[FL][2] +=  pid_roll - pid_pitch;
                m_foot_pose[FR][2] += -pid_roll - pid_pitch;
                m_foot_pose[HR][2] += -pid_roll + pid_pitch;
                m_foot_pose[HL][2] +=  pid_roll + pid_pitch;

                // shoulders
                Eigen::Vector3d com_shift(0,0,0);
                const double com_shift_mag = 0.0;
                com_shift += -com_shift_mag*gait.touch(gait_step+0.00, step_width, step_height) * body.shoulders[FL].tr;
                com_shift += -com_shift_mag*gait.touch(gait_step+0.50, step_width, step_height) * body.shoulders[FR].tr;
                com_shift += -com_shift_mag*gait.touch(gait_step+0.25, step_width, step_height) * body.shoulders[HL].tr;
                com_shift += -com_shift_mag*gait.touch(gait_step+0.75, step_width, step_height) * body.shoulders[HR].tr;

                Eigen::AngleAxisd aa_body_yaw( m_body_yaw + pid_yaw*M_PI/180.0 , Eigen::Vector3d(0,0,1) );

                m_shoulder_pose[FL] = aa_body_yaw._transformVector(com_shift + body.shoulders[FL].tr + Eigen::Vector3d(0,0,m_height));
                m_shoulder_pose[FR] = aa_body_yaw._transformVector(com_shift + body.shoulders[FR].tr + Eigen::Vector3d(0,0,m_height));
                m_shoulder_pose[HL] = aa_body_yaw._transformVector(com_shift + body.shoulders[HL].tr + Eigen::Vector3d(0,0,m_height));
                m_shoulder_pose[HR] = aa_body_yaw._transformVector(com_shift + body.shoulders[HR].tr + Eigen::Vector3d(0,0,m_height));


                m_pates_models[FL]->set_foot_final( m_foot_pose[FL], m_shoulder_pose[FL] );
                m_pates_models[FR]->set_foot_final( m_foot_pose[FR], m_shoulder_pose[FR] );
                m_pates_models[HL]->set_foot_final( m_foot_pose[HL], m_shoulder_pose[HL] );
                m_pates_models[HR]->set_foot_final( m_foot_pose[HR], m_shoulder_pose[HR] );

                ceres::Solve(options, &problem, &summary);

                m_angles->set_angles( {shoulder_angles[FL], shoulder_angles[FR],shoulder_angles[HL],shoulder_angles[HR],
                                       leg_angles[FL], leg_angles[FR],leg_angles[HL],leg_angles[HR],
                                       foot_angles[FL], foot_angles[FR],foot_angles[HL],foot_angles[HR],
                                       body.head.angle}, speed );

                if( speed < 1 ){
                    speed += 0.001;
                }
            }

            else if( m_attitude == enut::calibration ){

                // put foot points
                m_foot_pose[FL] = m_foot_pose_calibration[FL] + body.shoulders[FL].tr;
                m_foot_pose[FR] = m_foot_pose_calibration[FR] + body.shoulders[FR].tr;
                m_foot_pose[HL] = m_foot_pose_calibration[HL] + body.shoulders[HL].tr;
                m_foot_pose[HR] = m_foot_pose_calibration[HR] + body.shoulders[HR].tr;



                // shoulders
                m_shoulder_pose[FL] = body.shoulders[FL].tr + Eigen::Vector3d(0,0,m_height);
                m_shoulder_pose[FR] = body.shoulders[FR].tr + Eigen::Vector3d(0,0,m_height);
                m_shoulder_pose[HL] = body.shoulders[HL].tr + Eigen::Vector3d(0,0,m_height);
                m_shoulder_pose[HR] = body.shoulders[HR].tr + Eigen::Vector3d(0,0,m_height);


                m_pates_models[FL]->set_foot_final( m_foot_pose[FL], m_shoulder_pose[FL] );
                m_pates_models[FR]->set_foot_final( m_foot_pose[FR], m_shoulder_pose[FR] );
                m_pates_models[HL]->set_foot_final( m_foot_pose[HL], m_shoulder_pose[HL] );
                m_pates_models[HR]->set_foot_final( m_foot_pose[HR], m_shoulder_pose[HR] );

                ceres::Solve(options, &problem, &summary);

                m_angles->set_angles( {shoulder_angles[FL], shoulder_angles[FR],shoulder_angles[HL],shoulder_angles[HR],
                                       leg_angles[FL], leg_angles[FR],leg_angles[HL],leg_angles[HR],
                                       foot_angles[FL], foot_angles[FR],foot_angles[HL],foot_angles[HR],
                                       body.head.angle}, speed );

            }

            else if( m_attitude == enut::angles ){

                m_angles->set_angles( m_angle_mode_angles, 1 );

            }

            else if( m_attitude == enut::com_calib ){

                // put foot points
                m_foot_pose[FL] = m_foot_pose_calibration[FL] + body.shoulders[FL].tr;
                m_foot_pose[FR] = m_foot_pose_calibration[FR] + body.shoulders[FR].tr;
                m_foot_pose[HL] = m_foot_pose_calibration[HL] + body.shoulders[HL].tr;
                m_foot_pose[HR] = m_foot_pose_calibration[HR] + body.shoulders[HR].tr;

                if( m_com_mode_leg_up >= 0 ){
                    m_foot_pose[m_com_mode_leg_up][2] += 0.03;
                }


                // shoulders
                m_shoulder_pose[FL] = body.shoulders[FL].tr + Eigen::Vector3d(0,0,m_height) + m_com_mode_shift;
                m_shoulder_pose[FR] = body.shoulders[FR].tr + Eigen::Vector3d(0,0,m_height) + m_com_mode_shift;
                m_shoulder_pose[HL] = body.shoulders[HL].tr + Eigen::Vector3d(0,0,m_height) + m_com_mode_shift;
                m_shoulder_pose[HR] = body.shoulders[HR].tr + Eigen::Vector3d(0,0,m_height) + m_com_mode_shift;


                m_pates_models[FL]->set_foot_final( m_foot_pose[FL], m_shoulder_pose[FL] );
                m_pates_models[FR]->set_foot_final( m_foot_pose[FR], m_shoulder_pose[FR] );
                m_pates_models[HL]->set_foot_final( m_foot_pose[HL], m_shoulder_pose[HL] );
                m_pates_models[HR]->set_foot_final( m_foot_pose[HR], m_shoulder_pose[HR] );

                ceres::Solve(options, &problem, &summary);

                m_angles->set_angles( {shoulder_angles[FL], shoulder_angles[FR],shoulder_angles[HL],shoulder_angles[HR],
                                       leg_angles[FL], leg_angles[FR],leg_angles[HL],leg_angles[HR],
                                       foot_angles[FL], foot_angles[FR],foot_angles[HL],foot_angles[HR],
                                       body.head.angle}, speed );


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

    loss_size = 0.001;

    m_pates_models[FL] = new enut::Enut_Pate_model( true,  m_foot_pose[FL], body.shoulders[FL].tr );
    m_pates_models[FR] = new enut::Enut_Pate_model( false, m_foot_pose[FR], body.shoulders[FR].tr );
    m_pates_models[HL] = new enut::Enut_Pate_model( true,  m_foot_pose[HL], body.shoulders[HL].tr );
    m_pates_models[HR] = new enut::Enut_Pate_model( false, m_foot_pose[HR], body.shoulders[HR].tr );

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
    problem.SetParameterLowerBound( &shoulder_angles[HR], 0, 0.05 );
    problem.SetParameterUpperBound( &shoulder_angles[HR], 0, 105*M_PI/180. );

    problem.SetParameterLowerBound( &shoulder_angles[HL], 0, 0.05 );
    problem.SetParameterUpperBound( &shoulder_angles[HL], 0, 105*M_PI/180. );

    problem.SetParameterLowerBound( &shoulder_angles[FR], 0, 0.05 );
    problem.SetParameterUpperBound( &shoulder_angles[FR], 0, 105*M_PI/180. );

    problem.SetParameterLowerBound( &shoulder_angles[FL], 0, 0.05 );
    problem.SetParameterUpperBound( &shoulder_angles[FL], 0, 105*M_PI/180. );

    problem.SetParameterLowerBound( &leg_angles[HR], 0, 0.05 );
    problem.SetParameterUpperBound( &leg_angles[HR], 0, 180*M_PI/180. - 0.05 );

    problem.SetParameterLowerBound( &leg_angles[HL], 0, 0.05 );
    problem.SetParameterUpperBound( &leg_angles[HL], 0, 180*M_PI/180. - 0.05 );

    problem.SetParameterLowerBound( &leg_angles[FR], 0, 0.05 );
    problem.SetParameterUpperBound( &leg_angles[FR], 0, 180*M_PI/180. - 0.05 );

    problem.SetParameterLowerBound( &leg_angles[FL], 0, 0.05 );
    problem.SetParameterUpperBound( &leg_angles[FL], 0, 180*M_PI/180. - 0.05 );


    problem.SetParameterLowerBound( &foot_angles[HR], 0, 0.05 );
    problem.SetParameterUpperBound( &foot_angles[HR], 0, 180*M_PI/180. - 0.05 );

    problem.SetParameterLowerBound( &foot_angles[HL], 0, 0.05 );
    problem.SetParameterUpperBound( &foot_angles[HL], 0, 180*M_PI/180. - 0.05 );

    problem.SetParameterLowerBound( &foot_angles[FR], 0, 0.05 );
    problem.SetParameterUpperBound( &foot_angles[FR], 0, 180*M_PI/180. - 0.05 );

    problem.SetParameterLowerBound( &foot_angles[FL], 0, 0.05 );
    problem.SetParameterUpperBound( &foot_angles[FL], 0, 180*M_PI/180. - 0.05 );

    // Configure the solver.
    ceres::Solver::Options options;
    options.num_threads = std::thread::hardware_concurrency();
    options.linear_solver_type = ceres::DENSE_SCHUR;
    options.max_num_iterations = 40;
    options.max_solver_time_in_seconds=0.1;
    options.minimizer_progress_to_stdout = true;

    options.parameter_tolerance = 1e-16;
    options.gradient_tolerance = 1e-16;
    options.function_tolerance = 1e-16;
}
