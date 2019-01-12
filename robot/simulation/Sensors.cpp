#include "Sensors.hpp"

namespace Simulation
{
    bool Sensors::toSensorValues(SensorValues* s_out)
    {
        SensorValues& s = *s_out;
        int i;
        for (i = 0; i < Sensors::NUMBER_OF_SENSORS; ++i)
        {
            if (sptrs_[i] != &na_)
            {
                s.sensors[i] = *sptrs_[i];
            }
            else
            {
                s.sensors[i] = 0.0f;
            }
        }

        // ACCELEROMETER
        // Invert AccY (sims AccY is inverted)
        // TODO @jez debug DOUBLE CHECK SIM ACCY
        s.sensors[12] = -s.sensors[12];

        // Invert AccZ (sims AccZ is inverted)
        s.sensors[13] = -s.sensors[13];

        // Swap AccX and AccY (sim AccY is Nao AccX)
        float temp = s.sensors[11];
        s.sensors[11] = s.sensors[12];
        s.sensors[12] = temp;

        // Copy to old accelerometer
        s.sensors[0] = s.sensors[11];
        s.sensors[1] = s.sensors[12];
        s.sensors[2] = s.sensors[13];

        // GYROSCOPE
        // Convert gyro readings to radians
        for (int j = 8; j <= 10; ++j)
        {
           s.sensors[j] = DEG2RAD(s.sensors[j]);
        }

        // Invert GyroX (sims GyroX angle is inverted)
        s.sensors[8] = -s.sensors[8];

        // Invert GyroZ (sims GyroZ angle is possibly inverted)
        s.sensors[10] = -s.sensors[10];

        // Swap our GyroX and GyroY (opposite on robot / sim)
        temp = s.sensors[8];
        s.sensors[8] = s.sensors[9];
        s.sensors[9] = temp;
    
        // Copy to old gyro
        s.sensors[4] = s.sensors[8]; 
        s.sensors[5] = s.sensors[9];

        // SONAR
        // Add sonar readings (N/A so just use DISCARD value)
        //
        // NOTE: Do NOT use 0 here! Sonar readings are distance of object,
        // so 0 indicates that something is right infront of us! DISCARD
        // specifies the lower bound distance threshold to discard readings.
        for (i = 0; i < Sonar::NUMBER_OF_READINGS; ++i)
        {
            s.sonar[i] = Sonar::DISCARD;
        }

        return true;
    }

};
