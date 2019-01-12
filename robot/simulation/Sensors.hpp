#ifndef SIMULATION_SENSORS_H_
#define SIMULATION_SENSORS_H_

#include <types/SensorValues.hpp>

namespace Simulation
{

    /*
     * The Sensors class holds sensor information received from the server,
     * read from the simulated sensors on the simulated robot.
     *
     * The Sensors class can convert the simulated sensor information in to
     * a format consumable by rUNSWift, using the 'toSensorValues' method.
     * Unfortunately, there is not a one-to-one mapping of sensors on the
     * simulator to sensors on the NAO, so some sensors are omitted or their
     * values are estimations.
     *
     * NOTE: Joint perceptors are handled by the Joints class, not Sensors.
     * In rUNSWift, Joints are a subset of SensorValues. In the Simulation
     * module, PerceptorInfo holds a Joints object and a Sensors object.
     * The Joints object holds joint perceptor information, and the sensors
     * object holds other perceptor information.
     *
     */
    class Sensors
    {
    public:
        const static int NUMBER_OF_SENSORS = ::Sensors::NUMBER_OF_SENSORS;

        // GYRO COMPARISON
        // --- Nao ---
        // Gyro is in radians
        // GyroX is rotating left / right vertically
        //      - If Nao is facing you, falling left is positive
        //      - Falling right is negative
        // GyroY is rotating Nao back / forward
        //      - Forwards is positive angle
        //      - Backwards is negative angle
        // GyroZ is rotating left / right horizontally
        //      - Turning to the left is negative
        //      - Turning to the right is positive
        //
        // --- Sim ---
        // Gyro is in degrees
        // GyroX is rotating Nao back / forward
        //      - Forwards is negative angle
        //      - Backwards is positive angle
        // GyroY is (probably) rotating Nao left / right vertically
        //      - If Nao is facing you, falling left is positive
        //      - Falling right is negative
        // GyroZ is (probably) rotating left / right horizontally
        float gyr[3];

        // ACCELEROMETER COMPARISON
        // --- Nao ---
        // AccX is moving backwards / forwards
        //      - Forwards is positive
        //      - Backwards is negative
        // AccY is moving left / right (Nao facing you)
        //      - Left is positive
        //      - Right is negative
        // AccZ is moving up / down
        //      - Standing straight is negative
        //      - Upside down is positive
        //
        // --- Sim ---
        // AccY is moving backwards / forwards
        //      - Backwards is positive
        //      - Forwards is negative
        // AccX is moving left / right
        //      - Left is positive
        //      - Right is negative
        // AccZ is moving up / down
        //      - Standing straight is positive
        //      - Upside down is negative
        float acc[3];


        // FORCE PERCEPTOR COMPARISON
        // --- Nao ---
        // Four force peceptors on each foot
        //      - One in each corner
        //
        // --- Sim ---
        // One force perceptor on each foot
        //      - Perceptor provides location of force
        float lfpos[3];
        float lfval[3];
        float rfpos[3];
        float rfval[3];

        // X/Y ANGLE COMPARISON
        // --- Nao ---
        // Angles are in radians.
        // AngleX axis is rotating Nao left / right (vertical rotation)
        //      - If Nao is facing towards you, rotating it to the left is
        //            positive
        //      - Conversely, rotating it to the right is negative
        // AngleY axis is rotating Nao back / forward
        //      - Forward is positive angle
        //      - Backwards is negative angle
        //
        // --- Sim ---
        // Sim has no angle sensors
        // 
        // NOTE: Angle readings are simulated in SimulationThread using 
        // AngleSensor class.


        // SONAR COMPARISON
        // --- Nao ---
        // Two sonar senders and two receivers. One set on each side of chest.
        // Provides distance of object in meters.
        //
        // --- Sim ---
        // No sonars available.
        //
        // NOTE: Sonar readings are simulated in SimulationThread using 
        // SonarSensor class.

        Sensors()
        {
            memset((void*)&gyr, 0, 3);
            memset((void*)&acc, 0, 3);

            // Sensors
            // Mapping between simulator to libagent
            // TODO revisit descriptions
            int i=0;
                                         // Mapping descriptions sim -> libagent
            sptrs_[i++] = &na_;          // OLD SENSOR: N/A -> AccX
            sptrs_[i++] = &na_;          // OLD SENSOR: N/A -> AccY
            sptrs_[i++] = &na_;          // OLD SENSOR: N/A -> AccZ
            sptrs_[i++] = &na_;          // OLD SENSOR: N/A -> GyrX
            sptrs_[i++] = &na_;          // OLD SENSOR: N/A -> GyrY
            sptrs_[i++] = &na_;          // MISMATCH: N/A -> GyrRef
            sptrs_[i++] = &na_;          // MISMATCH: N/A -> AngleX
            sptrs_[i++] = &na_;          // MISMATCH: N/A -> AngleY

            sptrs_[i++] = &gyr[0];       // GyrX -> GyroscopeX
            sptrs_[i++] = &gyr[1];       // GyrY -> GyroscopeY
            sptrs_[i++] = &gyr[2];       // GyrZ -> GyroscopeZ
            sptrs_[i++] = &acc[0];       // AccX -> AccelerometerX
            sptrs_[i++] = &acc[1];       // AccY -> AccelerometerY
            sptrs_[i++] = &acc[2];       // AccZ -> AccelerometerZ

            sptrs_[i++] = &lfval[2];     // LeftFootZ -> LFFrontLeft
            sptrs_[i++] = &lfval[2];     // LeftFootZ -> LFFrontRight
            sptrs_[i++] = &lfval[2];     // LeftFootZ -> LFRearLeft
            sptrs_[i++] = &lfval[2];     // LeftFootZ -> LFRearRight
            sptrs_[i++] = &lfpos[0];     // LeftFootPosX -> LFCenterOfPressureX
            sptrs_[i++] = &lfpos[1];     // LeftFootPosY -> LFCenterOfPressure

            sptrs_[i++] = &rfval[2];     // RightFootZ -> RFFrontLeft
            sptrs_[i++] = &rfval[2];     // RightFootZ -> RFFrontRight
            sptrs_[i++] = &rfval[2];     // RightFootZ -> RFRearLeft
            sptrs_[i++] = &rfval[2];     // RightFootZ -> RFRearRight
            sptrs_[i++] = &rfpos[0];     // RightFootPosX -> RFCenterOfPressureX
            sptrs_[i++] = &rfpos[1];     // RightFootPosY -> RFCenterOfPressureY

            sptrs_[i++] = &na_;          // MISMATCH: N/A -> LFootBumperLeft
            sptrs_[i++] = &na_;          // MISMATCH: N/A -> LFootBumperRight
            sptrs_[i++] = &na_;          // MISMATCH: N/A -> RFootBumperLeft
            sptrs_[i++] = &na_;          // MISMATCH: N/A -> RFootBumperRight
            sptrs_[i++] = &na_;          // MISMATCH: N/A -> ChestBoardButton
            sptrs_[i++] = &na_;          // MISMATCH: N/A -> HeadTouchFront
            sptrs_[i++] = &na_;          // MISMATCH: N/A -> HeadTouchRear
            sptrs_[i++] = &na_;          // MISMATCH: N/A -> HeadTouchMiddle
            sptrs_[i++] = &na_;          // MISMATCH: N/A -> BatteryCharge
            sptrs_[i++] = &na_;          // MISMATCH: N/A -> BatteryCurrent
            sptrs_[i++] = &na_;          // MISMATCH: N/A -> US

            assert(i == ::Sensors::NUMBER_OF_SENSORS);
        }

        /*
         *  Writes the server sensor values to the provided SensorValues object,
         *  in the format that rUNSWift expects.
         *
         *  @param s_out The SensorValues object to write sensor information to.
         *  @return bool Returns true for success and false for failure.
         */
        bool toSensorValues(SensorValues* s_out);

private:
        /* Map to libagent order */
        float* sptrs_[::Sensors::NUMBER_OF_SENSORS];
        float na_;
    };
};


#endif // SIMULATION_SENSORS_H_
