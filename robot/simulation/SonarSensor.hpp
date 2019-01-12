#ifndef SIMULATION_SONARSENSOR_H_
#define SIMULATION_SONARSENSOR_H_

#include "simulation/PerceptorInfo.hpp"
#include "types/SensorValues.hpp"

#include <utility>
#include <vector>

namespace Simulation
{
    /* 
     *  The sonar sensor detects robots or goalposts that are infront of the
     *  robot.
     *
     *  This is just an estimation, and not necessarily mathematically correct.
     *  I'm just implementing it so the robots don't run in to each other or 
     *  goal posts.
     */
    class SonarSensor
    {
    public:
        const static float MAX_DISTANCE;
        const static float LEFT_LOWER_BOUND;
        const static float LEFT_UPPER_BOUND;
        const static float RIGHT_LOWER_BOUND;
        const static float RIGHT_UPPER_BOUND;
        const static int   READING_LIFETIME;
        const static unsigned int   MAX_READINGS;

        SonarSensor();
        void addMeasurement(const PerceptorInfo& perceived);
        void getSonar(float sonar[Sonar::NUMBER_OF_READINGS]);

    private:
        std::vector<std::pair<float,int> > left_readings_;
        std::vector<std::pair<float,int> > right_readings_;

        void checkLifetimes(std::vector<std::pair<float,int> >& readings);
    };

}

#endif // SIMULATION_SONARSENSOR_H_
