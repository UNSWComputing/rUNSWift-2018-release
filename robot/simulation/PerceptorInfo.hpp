#ifndef SIMULATOR_PERCEPTORINFO_H_
#define SIMULATOR_PERCEPTORINFO_H_

#include "Joints.hpp"
#include "Observation.hpp"
#include "Sensors.hpp"

#include "types/SensorValues.hpp"
#include "utils/angles.hpp"
#include "utils/body.hpp"
#include <boost/lexical_cast.hpp>
#include <boost/shared_ptr.hpp>

#include <iostream>
#include <cmath>
#include <string>

// TODO write comments
// TODO remove trailing whitespace
// TODO extrapolate sensors to own class

namespace Simulation
{
    class PerceptorInfo
    {
    public:
        PerceptorInfo()
            : time(0)
            , sl(0)
            , sr(0)
            , t(0)
            , pm(NULL)
            , joints()
            , sensors()
        { }

        float time;

        /* Game state */
        int sl;
        int sr;
        float t;
        char* pm;

        /* Hinge joints */
        Joints joints;

        /* Sensors */
        Sensors sensors;

        /* Observations */
        std::vector<Observation> observations;

        bool fromString(const std::string& to_parse);
        bool toSensorValues(SensorValues* values_out);

private:
        bool parseString(const std::string& to_parse, size_t* pos_in_out, char** string_out);
        bool parseString(const std::string& to_parse, size_t* pos_in_out, std::string* string_out);
        bool parseInt(const std::string& to_parse, size_t* pos_in_out, int* int_out);
        bool parseFloat(const std::string& to_parse, size_t* pos_in_out, float* float_out);
        bool parseTime(const std::string& to_parse, size_t* pos_in_out);
        bool parseGameState(const std::string& to_parse, size_t* pos_in_out);
        bool parseGyroscope(const std::string& to_parse, size_t* pos_in_out);
        bool parseAccelerometer(const std::string& to_parse, size_t* pos_in_out);
        bool parseObservation(const std::string& to_parse, size_t* pos_in_out);


        /**
         *  Parses hinge joint value from server message and returns pos after the HJ
         *  to continue parsing.
         *
         *  @param to_parse The server message to parse
         *  @param pos The position to parse from. On success, contains the position
         *  where parsing finished.
         *  @return bool True indicated success. False indicates failure.
         */
        bool parseHingeJoint(const std::string& to_parse, size_t* pos_in_out);
        bool parseForcePerceptor(const std::string& to_parse, size_t* pos_in_out);

        template<typename T>
        bool parseNumericType(const std::string& to_parse, size_t* pos_in_out, T* t_out)
        {
            size_t pos = *pos_in_out;
            bool is_neg = (to_parse[pos] == '-');
            if (is_neg)
            {
                pos++;
            }

            // Find the end of the value
            size_t val_end = to_parse.find_first_of(") ", pos);
            if (val_end == std::string::npos)
            {
                 return false;
            }

            T val;
            try
            {
                val = boost::lexical_cast<T>(to_parse.substr(pos, val_end-pos));
            }
            catch (std::exception& e)
            {
                std::cerr << e.what() << "\n";
                return false;
            }
            val = is_neg ? -val : val;

            *pos_in_out = val_end;
            *t_out = val;
            return true;
        }

    };  // End class PerceptorInfo

} // End namespace Simulation

#endif  // SIMULATOR_PERCEPTORINFO_H_
