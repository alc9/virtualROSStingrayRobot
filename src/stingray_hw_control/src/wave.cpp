/*
* Filename: wave.cpp
* Description: wave based functions
* Author: Alex Cunningham
* Start date: 14/06/2022
*/

#include "wave.h"

double waveGenerator(const double& freq, const double& time, const double& phaseDif, const double& actuatorIndex){
    return (M_PI/180.0)*(15.0)*sin((freq*2*M_PI*time)+(phaseDif*actuatorIndex));
}
