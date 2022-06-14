/*
* Filename: wave.cpp
* Description: wave based functions
* Author: Alex Cunningham
* Start date: 14/06/2022
*/

#include "wave.h"
//use - coro_t::pull_type seq(boost::coroutines2::fixedsize_stack(),waveGenerator)
//for (auto pair: seq){dosomething(pair)}
//typedef boost::coroutines2::coroutine<float> coro_t;
//void waveGenerator(coro_t::push_type& yield, const float& freq, const float& time, const float& phaseDif,const int& actuatorIndex){
//    yield((M_PI/180.0)*sin((freq*2*M_PI*time)+(phaseDif*actuatorIndex)));
//}
//coro_t::pull_type WaveGenerator_(boost::coroutines2::coroutine::fixedsize_stack(),std::bind(&waveGenerator,_1,_2,_3,_4,_5));
float waveGenerator(const float& freq, const float& time, const float& phaseDif,const int& actuatorIndex){
    return((M_PI/180.0)*sin((freq*2*M_PI*time)+(phaseDif*actuatorIndex)));
    }
