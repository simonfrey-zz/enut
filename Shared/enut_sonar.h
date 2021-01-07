#ifndef ENUT_SONAR_H
#define ENUT_SONAR_H

#include "ModulesChain/Module.h"
#include "Shared/enut_ifaces.h"
#include "IPM_Parameter/IPM_SectionedParmFile.h"

class Enut_Sonar : public ipm::modules::module, public enut::sonar_iface
{
public:
    Enut_Sonar(p3t::IPM_SectionedParmFile &config);
    double get_distance() override;

private:
    p3t::IPM_SectionedParmFile &m_db;

    bool module_startup() override;

    void loop();
    double distance(int timeout);
    void recordPulseLength();

    const int trigger = 27;
    const int echo = 22;
    volatile long startTimeUsec;
    volatile long endTimeUsec;
    double distanceMeters;
    long travelTimeUsec;
    long now;

    double m_last_distance;
    std::mutex m_mutex;
};

#endif // ENUT_SONAR_H
