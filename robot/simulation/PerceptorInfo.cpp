#include "PerceptorInfo.hpp"

#include <boost/make_shared.hpp>

namespace Simulation
{
    bool PerceptorInfo::fromString(const std::string& to_parse)
    {
        // Essentially, we attempt to parse each kind of perceptor.
        // If we fail for a specific index, just skip to the next index.
        size_t pos = 0;
        while (pos < to_parse.size())
        {
            if (parseTime(to_parse, &pos))
            {
                continue;
            }

            if (parseGameState(to_parse, &pos))
            {
                continue;
            }

            if (parseGyroscope(to_parse, &pos))
            {
                continue;
            }

            if (parseAccelerometer(to_parse, &pos))
            {
                continue;
            }

            if (parseHingeJoint(to_parse, &pos))
            {
                continue;
            }

            if (parseForcePerceptor(to_parse, &pos))
            {
                continue;
            }

            if (parseObservation(to_parse, &pos))
            {
                continue;
            }

            // If we reach here, all our parsing attempts have failed.
            pos += 1;
            while(to_parse[pos] == ')' || to_parse[pos] == ' ')
            {
                pos++;
            }
        }
        return true;
    }

    bool PerceptorInfo::toSensorValues(SensorValues* values_out)
    {
        // Get the JointValues from our joints object
        if (!joints.toJointValues(&values_out->joints))
        {
            return false;
        }

        // Get the sensor values from our sensors object
        if (!sensors.toSensorValues(values_out))
        {
            return false;
        }

        return true;
    }

    // TODO review this
    // TODO fix string too long
    // TODO currently disabled due to mem leak
    bool PerceptorInfo::parseString(const std::string& to_parse, size_t* pos_in_out, char** string_out)
    {
        std::string temp;
        if (!parseString(to_parse, pos_in_out, &temp))
        {
            return false;
        }
        // TODO just return here for now
        return true;

        /*
        if (*string_out)
        {
            free(*string_out);
        }

        *string_out = (char*)malloc(sizeof(char)*temp.size());
        memcpy(*string_out, temp.c_str(), temp.size());
        return true;
        */
    }

    bool PerceptorInfo::parseString(const std::string& to_parse, size_t* pos_in_out, std::string* string_out)
    {
        size_t pos = *pos_in_out;
        size_t str_end = to_parse.find_first_of(") ", pos);
        if (str_end == std::string::npos)
        {
            return false;
        }
        *string_out = to_parse.substr(pos, str_end-pos);
        *pos_in_out = str_end;
        return true;
    }

    bool PerceptorInfo::parseInt(const std::string& to_parse, size_t* pos_in_out, int* int_out)
    {
        return parseNumericType<int>(to_parse, pos_in_out, int_out);
    }

    bool PerceptorInfo::parseFloat(const std::string& to_parse, size_t* pos_in_out, float* float_out)
    {
        return parseNumericType<float>(to_parse, pos_in_out, float_out);
    }

    bool PerceptorInfo::parseTime(const std::string& to_parse, size_t* pos_in_out)
    {
        size_t pos = *pos_in_out;
        if (to_parse.substr(pos, 11) != "(time (now ")
        {
            return false;
        }
        pos += 11;

        if (!parseFloat(to_parse, &pos, &time))
        {
            return false;
        }
        pos += 2;       // Consume parentheses

        *pos_in_out = pos;
        return true;
    }

    bool PerceptorInfo::parseGameState(const std::string& to_parse, size_t* pos_in_out)
    {
        size_t pos = *pos_in_out;
        if (to_parse.substr(pos, 4) != "(GS ")
        {
            return false;
        }
        pos += 4;

        while(pos < to_parse.size())
        {
            if (to_parse[pos] != '(')
            {
                return false;
            }
            pos += 1;

            std::string attr;
            if (!parseString(to_parse, &pos, &attr))
            {
                return false;
            }
            pos += 1;   // Skip space

            if (attr == "sl")
            {
                if (!parseInt(to_parse, &pos, &sl))
                {
                    return false;
                }
            }
            else if (attr == "sr")
            {
                if (!parseInt(to_parse, &pos, &sr))
                {
                    return false;
                }
            }
            else if (attr == "t")
            {
                if (!parseFloat(to_parse, &pos, &t))
                {
                    return false;
                }
            }
            else if (attr == "pm")
            {
                if (!parseString(to_parse, &pos, &pm))
                {
                    return false;
                }
                break;  // This should be the last field
            }
            pos += 2;      // Consume padding
        }
        pos += 2;
        *pos_in_out = pos;

        return true;
    }

    bool PerceptorInfo::parseGyroscope(const std::string& to_parse, size_t* pos_in_out)
    {
        size_t pos = *pos_in_out;
        if (to_parse.substr(pos,5) != "(GYR ")
        {
            return false;
        }
        pos += 5;

        // Skip name
        if (to_parse.substr(pos, 10) != "(n torso) ")
        {
            return false;
        }
        pos += 10;

        // Get values
        if (to_parse.substr(pos, 4) != "(rt ")
        {
            return false;
        }
        pos += 4;

        for (int i=0; i < 3; ++i)
        {
            if (!parseFloat(to_parse, &pos, &sensors.gyr[i]))
            {
                return false;
            }
            pos += 1;  // Consume space
        }
        pos += 1; // Consume parenthesis
        *pos_in_out = pos;

        return true;
    }


    bool PerceptorInfo::parseAccelerometer(const std::string& to_parse, size_t* pos_in_out)
    {
        size_t pos = *pos_in_out;
        if (to_parse.substr(pos,5) != "(ACC ")
        {
            return false;
        }
        pos += 5;

        // Skip name
        if (to_parse.substr(pos, 10) != "(n torso) ")
        {
            return false;
        }
        pos += 10;

        // Get values
        if (to_parse.substr(pos, 3) != "(a ")
        {
            return false;
        }
        pos += 3;

        for (int i=0; i < 3; ++i)
        {
            if (!parseFloat(to_parse, &pos, &sensors.acc[i]))
            {
                return false;
            }
            pos += 1;  // Consume space
        }
        pos += 1; // Consume parenthesis
        *pos_in_out = pos;

        return true;
    }

    bool PerceptorInfo::parseHingeJoint(const std::string& to_parse, size_t* pos_in_out)
    {
        size_t pos = *pos_in_out;
        if (to_parse.substr(pos,4) != "(HJ ")
        {
            return false;
        }

        // Skip to name
        pos += 4;
        if (to_parse.substr(pos,3) != "(n ")
        {
            return false;
        }

        // Skip to name value
        pos += 3;
        std::string name;
        if (!parseString(to_parse, &pos, &name))
        {
            return false;
        }

        // Skip to axis
        pos += 1;
        if (to_parse.substr(pos,5) != " (ax ")
        {
            return false;
        }

        // Skip to axis value
        pos += 5;
        float val;
        if (!parseFloat(to_parse, &pos, &val))
        {
            return false;
        }

        // Skip to closing parenthesis
        if (to_parse[pos] != ')')
        {
            return false;
        }
        pos += 1;

        // Set the value in the joints object
        if (!joints.setValue(name, val))
        {
            return false;
        }

        // Return new pos after parsed HJ
        *pos_in_out = pos;
        return true;
    }

    bool PerceptorInfo::parseForcePerceptor(const std::string& to_parse, size_t* pos_in_out)
    {
        size_t pos = *pos_in_out;
        if (to_parse.substr(pos,5) != "(FRP ")
        {
            return false;
        }
        pos += 5;

        // Get name
        if (to_parse.substr(pos, 3) != "(n ")
        {
            return false;
        }
        pos += 3;

        std::string name;
        if (!parseString(to_parse, &pos, &name))
        {
            return false;
        }
        pos += 1;   // Skip to values

        // Check for loc tag
        if (to_parse.substr(pos, 4) != " (c ")
        {
            return false;
        }
        pos += 4;

        // Get location
        float loc[3];
        for (int i=0; i < 3; ++i)
        {
            if (!parseFloat(to_parse, &pos, &loc[i]))
            {
                return false;
            }
            pos += 1;   // Consume whitespace
        }

        // Get for force tag
        if (to_parse.substr(pos, 4) != " (f ")
        {
            return false;
        }
        pos += 4;

        // Get force
        float force[3];
        for (int i=0; i < 3; ++i)
        {
            if (!parseFloat(to_parse, &pos, &force[i]))
            {
                return false;
            }
            pos += 1;   // Consume whitespace
        }

        if (to_parse[pos] != ')')
        {
            return false;
        }

        // Copy to left or right foot depending on name
        if (name == "rf")
        {
            memcpy(&sensors.rfpos, &loc, sizeof(float)*3);
            memcpy(&sensors.rfval, &force, sizeof(float)*3);
        }
        else if (name == "lf")
        {
            memcpy(&sensors.lfpos, &loc, sizeof(float)*3);
            memcpy(&sensors.lfval, &force, sizeof(float)*3);
        }
        else
        {
            return false;
        }
        *pos_in_out = pos;

        return true;
    }

    bool PerceptorInfo::parseObservation(const std::string& to_parse, size_t* pos_in_out)
    {
        // TODO @jez implement teammate / enemy observations
        size_t pos = *pos_in_out;

        std::string type;
        if (!parseString(to_parse, &pos, &type))
        {
            return false;
        }
        pos += 1;   // Skip to values

        // Check for 'team' first
        // (This indicates we're seeing a player)
        std::string team;
        int number;
        if (to_parse.substr(pos, 6) == "(team ")
        {
            pos += 6;
            // Get team name
            if (!parseString(to_parse, &pos, &team))
            {
                return false;
            }
            pos += 2;   // Skip parentheses and whitespace
            
            if (to_parse.substr(pos, 4) != "(id ")
            {
                return false;
            }
            pos += 4;
               
            if (!parseInt(to_parse, &pos, &number))
            {
                return false;
            }
            pos += 3;   // Skip parentheses and whitespace

            // Read body part
            // Just get the position of the first one we read.
            std::string body_part;  
            if (!parseString(to_parse, &pos, &body_part))
            {
                return false;
            }
            pos += 1;   // Skip whitespace

            // TODO fix this
            // For now, just look for head. If we see different body part,
            // it can be our own (lmao).
            if (body_part != "head")
            {
                return false;
            }
        }

        // Identifier for pos
        if (to_parse.substr(pos, 5) != "(pol ")
        {
            return false;
        }
        pos += 5;

        // Get pol
        float pol[3];
        for (int i=0; i < 3; ++i)
        {
            if (!parseFloat(to_parse, &pos, &pol[i]))
            {
                return false;
            }
            pos += 1;   // Consume whitespace
        }

        Observation o;
        if (type == "B")
        {
            o = Observation(Observation::BALL, Observation::NO_SUBTYPE, pol);
        }
        else if (type == "F1L")
        {
            o = Observation(Observation::FLAG, Observation::F1_L, pol);
        }
        else if (type == "F1R")
        {
            o = Observation(Observation::FLAG, Observation::F1_R, pol);
        }
        else if (type == "F2L")
        {
            o = Observation(Observation::FLAG, Observation::F2_L, pol);
        }
        else if (type == "F2R")
        {
            o = Observation(Observation::FLAG, Observation::F2_R, pol);
        }
        else if (type == "CTR")
        {
            o = Observation(Observation::FLAG, Observation::CTR, pol);
        }
        else if (type == "G1L")
        {
            o = Observation(Observation::GOAL, Observation::G1_L, pol);
        }
        else if (type == "G1R")
        {
            o = Observation(Observation::GOAL, Observation::G1_R, pol);
        }
        else if (type == "G2L")
        {
            o = Observation(Observation::GOAL, Observation::G2_L, pol);
        }
        else if (type == "G2R")
        {
            o = Observation(Observation::GOAL, Observation::G2_R, pol);
        }
        else if (type == "P")
        {
            // TODO handle teammate / enemy and number
            o = Observation(Observation::PLAYER, Observation::NO_SUBTYPE, pol); 
        }
        else
        {
            return false;
        }

        observations.push_back(o);
        *pos_in_out = pos;
        return true;
    }

}
