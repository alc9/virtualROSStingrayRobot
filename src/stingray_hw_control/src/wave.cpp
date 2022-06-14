/*
* Filename: wave.cpp
* Description: wave based functions
* Author: Alex Cunningham
* Start date: 14/06/2022
*/

#include "../include/wave.h"
//use - coro_t::pull_type seq(boost::coroutines2::fixedsize_stack(),waveGenerator)
//for (auto pair: seq){dosomething(pair)}
void waveGenerator(coro_t::push_type& yield, const float& freq, const float& time, const float& phaseDif,const int& actuatorIndex){
    yield((M_PI/180.0)*sin((freq*2*M_PI*time)+(phaseDif*actuatorIndex)));
}
