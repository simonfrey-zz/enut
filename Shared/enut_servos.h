#ifndef ENUT_SERVOS_H
#define ENUT_SERVOS_H

#include "ModulesChain/Module.h"
#include "Shared/enut_ifaces.h"
#include "Shared/pca9685.h"
#include <map>

#define POIGNET_FL 2
#define COUDE_FL   3
#define EPAULE_FL  4

#define POIGNET_HL 5
#define COUDE_HL   6
#define EPAULE_HL  7

#define POIGNET_HR 10
#define COUDE_HR   9
#define EPAULE_HR  8

#define POIGNET_FR 13
#define COUDE_FR   12
#define EPAULE_FR  11

#define HEAD 15

#include "IPM_SCPI++/SCPIClassAdaptor.h"
#include "IPM_Parameter/IPM_SectionedParmFile.h"

class Enut_Servos : public ipm::modules::module, public PCA9685, public SCPIClassAdaptor<Enut_Servos>, public enut::angles_iface
{
public:
    Enut_Servos(p3t::IPM_SectionedParmFile &config);

    void set_angle( int, float );

    enut::Angles get_angles() override;
    void set_angles( enut::Angles, double speed ) override;
    void turn_off() override;
    void turn_on() override;


private:

    p3t::IPM_SectionedParmFile &m_db;

    struct Joint
    {
        float offset;
        float min;
        float max;
        bool  invert;
        int pwm_min;
        int pwm_max;
        float scale;
    };

    void loop();

    enut::Angles m_soll_angles;
    enut::Angles m_ist_angles;
    double m_speed;

    int map(int channel, float angle);
    std::map<int,Joint> m_joint;
    //std::map<int,float> m_current_angle;

    bool set_angles_scpi( std::vector<float> angles );
    std::pair<bool, std::vector<float>> get_angles_scpi();

    bool set_joint_pwm_min( int channel, int value );
    bool set_joint_pwm_max( int channel, int value );
};

#endif // ENUT_SERVOS_H
