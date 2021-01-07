#include "enut_sonar.h"
#include <../../../../../usr/include/wiringPi.h>

Enut_Sonar::Enut_Sonar(p3t::IPM_SectionedParmFile &config) :
    ipm::modules::module("sonar"),
    m_db( config ),
    trigger(2), // 27
    echo(3) // 22
{

}

double Enut_Sonar::get_distance()
{
    std::unique_lock<std::mutex> lock(m_mutex);
    return m_last_distance;
}

bool Enut_Sonar::module_startup()
{
    if (wiringPiSetup() == -1){
        PT_SEVERE( "wiringPiSetup error" );
        return false;
    }
    start_helper_thread( &Enut_Sonar::loop, this );
    return true;
}

void Enut_Sonar::loop()
{
    // inits
    pinMode(trigger, OUTPUT);
    pinMode(echo, INPUT);
    digitalWrite(trigger, LOW);
    delay(500);

    PT_INFO("Sonor init ok");

    while( helper_in_normal_operation() ){

        {
            std::unique_lock<std::mutex> lock(m_mutex);
            m_last_distance = distance(30000);
        }
        PT_INFO( distance(30000) );
        p3t::sleep(0.1);
    }
}

double Enut_Sonar::distance(int timeout)
{
    delay(10);

    digitalWrite(trigger, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigger, LOW);

    now=micros();

    //PT_INFO("wait low");
    while (digitalRead(echo) == LOW && micros()-now<timeout && helper_in_normal_operation());

    //PT_INFO("while high");
    recordPulseLength();

    //PT_INFO("done");
    travelTimeUsec = endTimeUsec - startTimeUsec;
    distanceMeters = ((travelTimeUsec/1000000.0)*340.29)/2;

    return distanceMeters;
}

void Enut_Sonar::recordPulseLength()
{
    startTimeUsec = micros();
    while ( digitalRead(echo) == HIGH && helper_in_normal_operation() );
    endTimeUsec = micros();
}
