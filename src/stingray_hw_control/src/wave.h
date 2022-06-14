/*
* Filename: wave.h
* Description: wave based functions
* Author: Alex Cunningham
* Start date: 14/06/2022
*/
#pragma once
#include <std_msgs/Float64.h>
#include <memory>
#include <boost/coroutine2/all.hpp>
#include <functional>
#include <cmath>
//During periods of control oscillation there is no reason to cache wave
//position arrays, so instead use a generator
//typedef boost::coroutines2::coroutine<float> coro_t;

//void waveGenerator(coro_t::push_type& yield, const float& freq, const float& time, const float& phaseDif, const int& actuatorIndex);
float waveGenerator(const float& freq, const float& time, const float&
phaseDif, const int& actuatorIndex);
