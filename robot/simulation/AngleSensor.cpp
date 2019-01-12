#include "AngleSensor.hpp"

#include <utils/angles.hpp>

#include <cmath>

// TODO debug
#include <iomanip>

namespace Simulation
{

    void AngleSensor::addMeasurement(float gyr_x, float gyr_y, float gyr_z,
                                     float acc_x, float acc_y, float acc_z)
    {
        // I've had a lot of trouble trying to get this to work correctly.
        // Essentially, I'm using a complimentary filter to combine both the
        // gyroscope and accelerometer readings to estimate the robot's X/Y
        // angle. This is because the angle will drift over time using just
        // gyro and acc has gravity in its readings.
        //
        // The problem is that I've gotten this formula off the internet and
        // there are so many assumptions about the data, e.g.: should the
        // gyro and accelerometer be on the same axes (they're not), should the
        // accelerometer data be m/s^2 (it is), should acc_z sit at 0 or
        // -gravitiy (it's at -gravity), etc...
        //
        // So there are possibly bugs here. I just got it to the stage where it
        // works good enough. Sorry!
        //
        // The weightings of the gyro and acc could possibly be tweaked too.
        // The problem I was finding was that the acc seemed to lag a little
        // bit, so when the robot fell over, the acc would push it's angles
        // towards 0 for a few seconds before pushing it towards the correct
        // fallen angle. This could cause some getup problems, so I weighted
        // the acc a little less.

        // Add the gyro reading to angle X and Y
        x_ += gyr_x / 50;
        y_ += gyr_y / 50;

        // Convert acc data from m/s^2 to m/s
        float temp;
        temp = sqrt(fabs(acc_x));
        acc_x = (acc_x > 0 ? temp : -temp);
        temp = sqrt(fabs(acc_y));
        acc_y = (acc_y > 0 ? temp : -temp);
        temp = sqrt(fabs(acc_z));
        acc_z = (acc_z > 0 ? acc_z : -acc_z);

        // Incorporate X acceleration compliment
        float x_acc_comp = atan2f(acc_y, acc_z);

        // Incorporate Y acceleration compliment
        float y_acc_comp = atan2f(acc_x, acc_z);

        // Get new estimate
        x_ = x_ * 0.99 + x_acc_comp * 0.01;
        y_ = y_ * 0.99 + y_acc_comp * 0.01;
    }

    void AngleSensor::getAngles(float* x_out, float* y_out)
    {
        *x_out = x_;
        *y_out = y_;
    }

};
