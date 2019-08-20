

#include "simulator.h"

CSimulator::CSimulator()
{
    accel_noise = NULL;
    gyro_noise = NULL;
    noise_count_accel = 0;
    noise_count_gyro = 0;
    idx = 4;
}

CSimulator::~CSimulator()
{
    // SAFEDELETEARRAY(accel_noise);
    // SAFEDELETEARRAY(gyro_noise);
}

//ref:https://stackoverflow.com/questions/32889309/adding-gaussian-noise
real* CSimulator::GenerateGaussianWhiteNoise(unsigned int buflen, real mean, real stddev)
{
    real* noise = new real[buflen];
    memset(noise, 0, buflen*sizeof(real));
    // construct a trivial random generator engine from a time-based seed:
    unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::default_random_engine generator (seed);
    std::normal_distribution<real> dist(mean, stddev);
    for (unsigned int i = 0; i < buflen; i++)
    {
        noise[i] = dist(generator);
    }
    
    cout << "mean:" << mean << "  stddev:" << stddev << endl;
    for(int i = 0; i < min(100,buflen); i++)
    {
        cout << noise[i] << endl;
    }

    return noise;
}

void CSimulator::GenerateNoiseOnGyroAccel()
{
    gyro_noise = GenerateGaussianWhiteNoise(NOISE_BUF_LEN, noise_model_array[idx].gyro_noise_mean, noise_model_array[idx].gyro_noise_stddev);
    cout << "!****************!" << endl;
    accel_noise = GenerateGaussianWhiteNoise(NOISE_BUF_LEN, noise_model_array[idx].accel_noise_mean, noise_model_array[idx].accel_noise_stddev);
}
