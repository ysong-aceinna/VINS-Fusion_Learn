
/*******************************************************
 * Copyright (C) 2019
 *
 * Author: SongYang (ysong@aceinna.com)
 *******************************************************/
#include "simulator.h"

NoiseModel noise_model_array[] = {               //ARW:deg/√hr,  VRW:m/s/√hr
    {0.0, 0.689783, 0.0, 0.114964},              //ARW:3.0   VRW:0.5
    {0.0, 2.35188, 0.0, 0.470691},               //ARW:10.0   VRW:2.0

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
    {0.0, 0.0, 0.5, 0.0},                       //accel_bias:0.5 m/s^2
    {0.0, 0.0, 1.0, 0.0},                       //accel_bias:1.0 m/s^2
    {0.0, 0.0, 1.2, 0.0},                       //accel_bias:1.2 m/s^2
    {0.0, 0.0, 1.5, 0.0},                       //accel_bias:1.5 m/s^2
    {0.0, 0.0, 3.0, 0.0},                       //accel_bias:3.0 m/s^2

    {0.0, 0, 0.0, 0}                //ARW:0.66  VRW:0.11 (ADIS14468)
};

CSimulator::CSimulator()
{
    accel_noise = NULL;
    gyro_noise = NULL;
    noise_count_accel = 0;
    noise_count_gyro = 0;
    idx = 0;
}

CSimulator::~CSimulator()
{
    // SAFEDELETEARRAY(accel_noise);
    // SAFEDELETEARRAY(gyro_noise);
}

//ref:https://stackoverflow.com/questions/32889309/adding-gaussian-noise
double* CSimulator::GenerateGaussianWhiteNoise(unsigned int buflen, double mean, double stddev)
{
    double* noise = new double[buflen];
    memset(noise, 0, buflen*sizeof(double));

    // construct a trivial random generator engine from a time-based seed:
    // unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
    unsigned seed = 1;  //for getting certain noise every time.
    std::default_random_engine generator (seed);
    std::normal_distribution<double> dist(mean, stddev);
    for (unsigned int i = 0; i < buflen; i++)
    {
        noise[i] = dist(generator);
    }
    
    return noise;
}

void CSimulator::GenerateNoiseOnGyroAccel()
{
    gyro_noise = GenerateGaussianWhiteNoise(NOISE_BUF_LEN, noise_model_array[idx].gyro_noise_mean, noise_model_array[idx].gyro_noise_stddev);

    cout << "Add gyro_noise, mean:" << noise_model_array[idx].gyro_noise_mean << "  stddev:" << noise_model_array[idx].gyro_noise_stddev << endl;
    for(int i = 0; i < MIN(100,NOISE_BUF_LEN); i++)
    {
        cout << gyro_noise[i] << endl;
    }

    cout << "!****************!" << endl;

    accel_noise = GenerateGaussianWhiteNoise(NOISE_BUF_LEN, noise_model_array[idx].accel_noise_mean, noise_model_array[idx].accel_noise_stddev);

    cout << "Add accel_noise, mean:" << noise_model_array[idx].accel_noise_mean << "  stddev:" << noise_model_array[idx].accel_noise_stddev << endl;
    for(int i = 0; i < MIN(100,NOISE_BUF_LEN); i++)
    {
        cout << accel_noise[i] << endl;
    }

}
