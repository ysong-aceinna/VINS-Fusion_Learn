/*******************************************************
 * Copyright (C) 2019
 *
 * Author: SongYang (ysong@aceinna.com)
 *******************************************************/
#pragma once
#include <stdio.h>
#include <iostream>
#include <cstring>
#include <random>
#include <chrono>
#include "macro.h"
#include "struct.h"

using namespace std;

class CSimulator
{
public:
	CSimulator();
	~CSimulator();
    void GenerateNoiseOnGyroAccel();
    
    inline void SetNoiseType(unsigned int _idx)
    {
        idx = _idx;
    }

    inline double GetAccelNoise()
    {
        if(noise_count_accel >= NOISE_BUF_LEN) noise_count_accel = 0;
        return accel_noise[noise_count_accel++];
    }

    inline double GetGyroNoise()
    {
        if(noise_count_gyro >= NOISE_BUF_LEN) noise_count_gyro = 0;
        return gyro_noise[noise_count_gyro++];
    }

private:
    double* GenerateGaussianWhiteNoise(unsigned int buflen, double mean, double stddev);

private:
    static const unsigned int NOISE_BUF_LEN = 12000*3; //200hz, 1分钟，三轴
    double* accel_noise;
    double* gyro_noise;
    unsigned int idx;
    unsigned int noise_count_accel;
    unsigned int noise_count_gyro;

};
