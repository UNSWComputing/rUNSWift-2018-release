#include "SonarSensor.hpp"
#include "simulation/Observation.hpp"
#include "simulation/SimVisionAdapter.hpp"

#include <climits>

namespace Simulation 
{
    const float SonarSensor::MAX_DISTANCE       = 0.80f;
    const float SonarSensor::RIGHT_LOWER_BOUND  = -DEG2RAD(55);
    const float SonarSensor::RIGHT_UPPER_BOUND  = DEG2RAD(5);
    const float SonarSensor::LEFT_LOWER_BOUND   = -DEG2RAD(5);
    const float SonarSensor::LEFT_UPPER_BOUND   = DEG2RAD(55);
    const int   SonarSensor::READING_LIFETIME   = 3;
    const unsigned int SonarSensor::MAX_READINGS       = 10;

    SonarSensor::SonarSensor()
        : left_readings_(), right_readings_()
    { }


    void SonarSensor::checkLifetimes(std::vector<std::pair<float,int> >& readings)
    {
        for (std::vector<std::pair<float, int> >::iterator itr = readings.begin();
            itr != readings.end(); )
        {
            --(itr->second);
            if (itr->second <= 0)
            {
                itr = readings.erase(itr);
            }
            else
            {
                ++itr;
            }
        }
    }


    void SonarSensor::addMeasurement(const PerceptorInfo& received)
    {
        // Here we have to check which sonars are detecting the object.
        // 
        // As an estimation, we pick some heading thresholds and tune
        // them through trial and error. Note that the left sonar
        // upper-bound and right sonar lower-bound overlap. This is
        // for objects infront of us.
        //
        // As we don't get observations every tick, we have to keep
        // sensor readings for N ticks or they will be ignored. It doesn't
        // really matter if we keep sensor readings that are no longer 
        // correct for a tick or two (worst case scenario we just strafe for
        // a little bit).

        // Remove old readings
        checkLifetimes(left_readings_);
        checkLifetimes(right_readings_);        

        // Add readings for each valid observation
        for (unsigned int i=0; i < received.observations.size(); ++i) 
        {
            // We only care about GOAL and PLAYER observations
            Observation o = received.observations[i];
            if (o.type != Observation::GOAL && o.type != Observation::PLAYER)
            {
                continue;
            }
            
            // Add readings for objects under the max distance        
            RRCoord rr = SimVisionAdapter::getRR(o, received.joints.hj1);
            float meters = rr.distance() / 1000; // RR is in mills
    
            if (meters < MAX_DISTANCE) 
            {
                std::pair<float, int> reading = std::make_pair(meters, READING_LIFETIME);
   
                // If the reading is within the valid heading bounds for left
                // sonar, add to left readings 
                float heading = rr.heading();
                if (heading >= LEFT_LOWER_BOUND && heading <= LEFT_UPPER_BOUND)
                {
                    left_readings_.push_back(reading);
                }

                // If the reading is within the bounds for right sonar, add to
                // right readings
                if (heading >= RIGHT_LOWER_BOUND && heading <= RIGHT_UPPER_BOUND)
                {
                    right_readings_.push_back(reading);
                }
            }
        }

        // Sort readings by lowest distance first
        std::sort(left_readings_.begin(), left_readings_.end());
        std::sort(right_readings_.begin(), right_readings_.end());
        if (left_readings_.size() >= MAX_READINGS)
        {
            left_readings_.resize(MAX_READINGS);
        }
        if (right_readings_.size() >= MAX_READINGS)
        {
            right_readings_.resize(MAX_READINGS);
        }
    } 

    void SonarSensor::getSonar(float sonar[Sonar::NUMBER_OF_READINGS])
    {
        int i = 0;
        for (std::vector<std::pair<float, int> >::iterator itr = left_readings_.begin();
            itr != left_readings_.end() && i < Sonar::Right0; ++itr)
        {
            sonar[i++] = itr->first;
        }
        while (i < Sonar::Right0)
        {
            sonar[i++] = Sonar::DISCARD;
        }

        for (std::vector<std::pair<float, int> >::iterator itr = right_readings_.begin();
            itr != right_readings_.end() && i < Sonar::NUMBER_OF_READINGS; ++itr)
        {
            sonar[i++] = itr->first;
        }
        while (i < Sonar::NUMBER_OF_READINGS)
        {
            sonar[i++] = Sonar::DISCARD;
        }      
    }

}
