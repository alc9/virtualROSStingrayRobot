/*
* Filename: wave.h
* Description: wave based functions
* Author: Alex Cunningham
* Start date: 14/06/2022
*/
#pragma once
#include <std_msgs/Float64.h>
#include <memory>
#include <cmath>
#include <iostream>
//During periods of control oscillation there is no reason to cache wave
//position arrays, so instead use a generator
//typedef boost::coroutines2::coroutine<float> coro_t;

//void waveGenerator(coro_t::push_type& yield, const float& freq, const float& time, const float& phaseDif, const int& actuatorIndex);
double waveGenerator(const double& freq, const double& time, const double& phaseDif, const double& actuatorIndex);
