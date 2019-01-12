#ifndef SIMULATION_ANGLESENSOR_H_
#define SIMULATION_ANGLESENSOR_H

namespace Simulation
{
    /*
     *  The simulator does not provide an AngleX or AngleY sensor. Luckily,
     *  we can calculate this information ourselves using the gyroscope and
     *  accelerometer information. This class does this to provide an estimation
     *  of such a sensor.
     */
    class AngleSensor
    {
    public:
        AngleSensor(float initial_x, float initial_y)
            : x_(initial_x), y_(initial_y)
        { }

        AngleSensor()
            : x_(0.0f), y_(0.0f)
        { }

        void addMeasurement(float gyr_x, float gyr_y, float gyr_z,
                            float acc_x, float acc_y, float acc_z);
        void getAngles(float* x_out, float* y_out);

private:
        float x_;
        float y_;

    };
}

#endif // SIMULATION_ANGLESENSOR_H_
