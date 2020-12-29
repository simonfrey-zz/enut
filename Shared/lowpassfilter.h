#ifndef LOWPASSFILTER_H
#define LOWPASSFILTER_H

#include <cmath>

class LowPassFilter
{
public:
    LowPassFilter( double CutOffFrequency, double DeltaTime );

    double update(double input);
    void set_output( double output ){
        m_output = output;
    }
private:
        double m_output;
        double m_ePow;
};

#endif // LOWPASSFILTER_H
