
#pragma once
#include <stdio.h>
#include <iostream>
#include <cstring>
#include <random>
#include <chrono>

using namespace std;

#define real float
#define max(a,b) ((a) > (b) ? (a) : (b))
#define min(a,b) ((a) < (b) ? (a) : (b))
#define SAFEDELETE(p) if(p) {delete p; p = NULL;}
#define SAFEDELETEARRAY(p) if(p) {delete p[]; p = NULL;}

struct NoiseModel 
{
    real gyro_noise_mean;
    real gyro_noise_stddev;
    real accel_noise_mean;
    real accel_noise_stddev;
};

NoiseModel noise_model_array[] = {               //ARW:deg/√hr,  VRW:m/s/√hr
    {0.0, 0.106562, 0.0, 0.024037},              //ARW:0.8   VRW:0.15
    {0.0, 0.144222, 0.0, 0.0305505},             //ARW:0.9   VRW:0.17
    {0.0, 0.177075, 0.0, 0.03937},               //ARW:1.0   VRW:0.2
    {0.0, 0.31749, 0.0, 0.0657858},              //ARW:1.5   VRW:0.3
    {0.0, 0.444997, 0.0, 0.0906458},             //ARW:2.0   VRW:0.4
    {0.0, 1.1682, 0.0, 0.234272},                //ARW:5.0   VRW:1.0
    {0.0, 2.35188, 0.0, 0.706631},               //ARW:10.0  VRW:3.0
    {0.0, 11.7841, 0.0, 4.71397},                //ARW:50.0  VRW:20.0

    {1.0, 0.0, 0.0, 0.0},                       //gyro_bias:1.0 deg/s
    {2.0, 0.0, 0.0, 0.0},                       //gyro_bias:2.0 deg/s
    {3.0, 0.0, 0.0, 0.0},                       //gyro_bias:3.0 deg/s
    {0.0, 0.0, 0.3, 0.0},                       //accel_bias:0.3 m/s^2
    {0.0, 0.0, 1.5, 0.0},                       //accel_bias:1.5 m/s^2
    {0.0, 0.0, 3.0, 0.0},                       //accel_bias:3.0 m/s^2

    {0.0, 0, 0.0, 0}                //ARW:0.66  VRW:0.11 (ADIS14468)
};


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

    inline real GetAccelNoise()
    {
        if(noise_count_accel >= NOISE_BUF_LEN) noise_count_accel = 0;
        return accel_noise[noise_count_accel++];
    }

    inline real GetGyroNoise()
    {
        if(noise_count_gyro >= NOISE_BUF_LEN) noise_count_gyro = 0;
        return gyro_noise[noise_count_gyro++];
    }

private:
    real* GenerateGaussianWhiteNoise(unsigned int buflen, real mean, real stddev);

private:
    static const unsigned int NOISE_BUF_LEN = 12000*3; //200hz, 1分钟，三轴
    real* accel_noise;
    real* gyro_noise;
    unsigned int idx;
    unsigned int noise_count_accel;
    unsigned int noise_count_gyro;

};
