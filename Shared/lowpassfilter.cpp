#include "lowpassfilter.h"

LowPassFilter::LowPassFilter(double CutOffFrequency, double DeltaTime) :
    m_output(0),
    m_ePow(1-exp(-DeltaTime * 2 * M_PI * CutOffFrequency))
{

}

double LowPassFilter::update(double input){
    return m_output += (input - m_output) * m_ePow;
}
