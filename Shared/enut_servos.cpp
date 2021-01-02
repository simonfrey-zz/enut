#include "enut_servos.h"
#include <cmath>
#include "PTDebug.h"

Enut_Servos::Enut_Servos( p3t::IPM_SectionedParmFile &config ) :
    ipm::modules::module("servo"),
    PCA9685(1,0x40),
    SCPIClassAdaptor<Enut_Servos>(this, "SERVO"),
    m_db( config )
{
    for( int i = 0; i < 16; i++){
        m_joint[i] = {0,0,180,false,0,0,1};
    }

    auto init_joint = [this](int id, std::string name ) -> void {

        m_db.lock();
        float offset = m_db.getd( name + "_offset", 0.0 );
        float min = m_db.getd( name + "_min", 0.0 );
        float max = m_db.getd( name + "_max", 180.0 );
        bool inverted = m_db.getd( name + "_inverted", false );
        float scale = m_db.getd( name + "_scale", 1.0 );
        m_db.unlock();

        m_joint[id].offset = offset*M_PI/180.;
        m_joint[id].min = min*M_PI/180.f;
        m_joint[id].max = max*M_PI/180.f;
        m_joint[id].invert = inverted;
        m_joint[id].pwm_min = 100;
        m_joint[id].pwm_max = 512;
        m_joint[id].scale = scale;
    };

    init_joint( COUDE_FL, "COUDE_FL" );
    init_joint( COUDE_FR, "COUDE_FR" );
    init_joint( COUDE_HL, "COUDE_HL" );
    init_joint( COUDE_HR, "COUDE_HR" );

    init_joint( POIGNET_FL, "POIGNET_FL" );
    init_joint( POIGNET_FR, "POIGNET_FR" );
    init_joint( POIGNET_HL, "POIGNET_HL" );
    init_joint( POIGNET_HR, "POIGNET_HR" );

    init_joint( EPAULE_FL, "EPAULE_FL" );
    init_joint( EPAULE_FR, "EPAULE_FR" );
    init_joint( EPAULE_HL, "EPAULE_HL" );
    init_joint( EPAULE_HR, "EPAULE_HR" );

    init_joint( HEAD, "HEAD" );

    /*
    m_joint[COUDE_FL] = {0,0,180*M_PI/180.,false,110,467};
    m_joint[COUDE_HL] = {-2*M_PI/180.,0,180*M_PI/180.,false,107,475};
    m_joint[COUDE_HR] = {0,0,180*M_PI/180.,true,123,497};
    m_joint[COUDE_FR] = {0,0,180*M_PI/180.,true,118,498};

    m_joint[POIGNET_FL] = {5*M_PI/180.,-5*M_PI/180.,140*M_PI/180.,false,133,490};
    m_joint[POIGNET_HL] = {5*M_PI/180.,-5*M_PI/180.,140*M_PI/180.,false,105,470};
    m_joint[POIGNET_HR] = {5*M_PI/180.,-5*M_PI/180.,140*M_PI/180.,true,85,460};
    m_joint[POIGNET_FR] = {5*M_PI/180.,-5*M_PI/180.,140*M_PI/180.,true,40,455};

    m_joint[EPAULE_FL] = {0,0,100*M_PI/180.,false,140,515};
    m_joint[EPAULE_HL] = {0,0,100*M_PI/180.,false,150,500};
    m_joint[EPAULE_HR] = {0,0,100*M_PI/180.,true,120,510};
    m_joint[EPAULE_FR] = {0,0,100*M_PI/180.,true,120,505};

    m_joint[HEAD] = {0,30*M_PI/180.,180*M_PI/180.,false,60,510};
    */

    m_speed = 1;

    start_helper_thread(&Enut_Servos::loop, this);

    addCommand({"JOINT", "ANGLES"}, &Enut_Servos::set_angles_scpi, &Enut_Servos::get_angles_scpi );

    addCommandWriteOnly({"JOINT","PWM_MIN"}, &Enut_Servos::set_joint_pwm_min );
    addCommandWriteOnly({"JOINT","PWM_MAX"}, &Enut_Servos::set_joint_pwm_max );
}

void Enut_Servos::loop(){

    const double dt = 1.0/50.0;
    while (helper_in_normal_operation()) {

        p3t::sleep(dt);

        // 0.12 sec / 60 degree -> 500 deg/s
        const double max_angle_per_dt = m_speed * 600.0 * (M_PI/180.0) * dt;
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

        // update angles
        set_angle( EPAULE_FL, m_ist_angles[0] );
        set_angle( EPAULE_FR, m_ist_angles[1] );
        set_angle( EPAULE_HL, m_ist_angles[2] );
        set_angle( EPAULE_HR, m_ist_angles[3] );

        set_angle( COUDE_FL, m_ist_angles[4] );
        set_angle( COUDE_FR, m_ist_angles[5] );
        set_angle( COUDE_HL, m_ist_angles[6] );
        set_angle( COUDE_HR, m_ist_angles[7] );

        set_angle( POIGNET_FL, m_ist_angles[8] );
        set_angle( POIGNET_FR, m_ist_angles[9] );
        set_angle( POIGNET_HL, m_ist_angles[10] );
        set_angle( POIGNET_HR, m_ist_angles[11] );

        set_angle( HEAD, m_ist_angles[12] );

    }

}


void Enut_Servos::set_angle(int channel, float angle)
{
    angle = std::fmin( angle, m_joint[channel].max);
    angle = std::fmax( angle, m_joint[channel].min);

    if( m_joint[channel].invert ){
        angle = M_PI - angle;
    }

    const int v = map( channel, angle + m_joint[channel].offset );
    //PT_INFO(channel << "=" << v);
    setPWM( channel, v );
}

enut::Angles Enut_Servos::get_angles()
{
    return m_ist_angles;
}

void Enut_Servos::set_angles(enut::Angles a, double speed)
{
    m_soll_angles = a;
    m_speed = speed;
}

void Enut_Servos::turn_off()
{
    pca_sleep();
}

void Enut_Servos::turn_on()
{
    pca_init();
}

int Enut_Servos::map(int channel, float angle)
{
    const float a_min = 0;
    const float a_max = m_joint[channel].scale*M_PI;//180;
    const float v_min = m_joint[channel].pwm_min;//102;
    const float v_max = m_joint[channel].pwm_max;//512;
    return (v_max-v_min)*(angle-a_min) / (a_max-a_min) + v_min;
}

bool Enut_Servos::set_angles_scpi(std::vector<float> angles)
{
    if( angles.size() != 13 ){
        scpi_set_last_error("must have 13 angles");
        return false;
    }

    set_angle( POIGNET_FL, angles[0] );
    set_angle( COUDE_FL, angles[1] );
    set_angle( EPAULE_FL, angles[2] );

    set_angle( POIGNET_HL, angles[3] );
    set_angle( COUDE_HL, angles[4] );
    set_angle( EPAULE_HL, angles[5] );

    set_angle( POIGNET_HR, angles[6] );
    set_angle( COUDE_HR, angles[7] );
    set_angle( EPAULE_HR, angles[8] );

    set_angle( POIGNET_FR, angles[9] );
    set_angle( COUDE_FR, angles[10] );
    set_angle( EPAULE_FR, angles[11] );

    set_angle( HEAD, angles[12] );

    return true;
}

std::pair<bool, std::vector<float> > Enut_Servos::get_angles_scpi()
{
    std::vector<float> angles;
    /*
    angles.push_back( m_current_angle[POIGNET_FL] );
    angles.push_back( m_current_angle[COUDE_FL] );
    angles.push_back( m_current_angle[EPAULE_FL] );

    angles.push_back( m_current_angle[POIGNET_HL] );
    angles.push_back( m_current_angle[COUDE_HL] );
    angles.push_back( m_current_angle[EPAULE_HL] );

    angles.push_back( m_current_angle[POIGNET_HR] );
    angles.push_back( m_current_angle[COUDE_HR] );
    angles.push_back( m_current_angle[EPAULE_HR] );

    angles.push_back( m_current_angle[POIGNET_FR] );
    angles.push_back( m_current_angle[COUDE_FR] );
    angles.push_back( m_current_angle[EPAULE_FR] );

    angles.push_back( m_current_angle[HEAD] );
    */
    return {false, angles};
}

bool Enut_Servos::set_joint_pwm_min(int channel, int value)
{
    m_joint[channel].pwm_min = value;
    //set_angle( channel, m_current_angle[channel] );
    return true;
}

bool Enut_Servos::set_joint_pwm_max(int channel, int value)
{
    m_joint[channel].pwm_max = value;
    //set_angle( channel, m_current_angle[channel] );
    return true;
}
