#ifndef SIMULATION_OBSERVATION_H_
#define SIMULATION_OBSERVATION_H_

#include <cstring>

namespace Simulation
{
    class Observation
    {
    public:
        enum Type
        {
            NO_TYPE,
            BALL,
            FLAG,
            GOAL,
            PLAYER
        };

        enum SubType
        {
            NO_SUBTYPE,
            F1_L,       // Top left corner (bird's eye view)
            F1_R,       // Top right corner
            F2_L,       // Bottom left corner
            F2_R,       // Bottom right corner
            G1_L,       // Top left goal
            G1_R,       // Top right goal
            G2_L,       // Bottom left goal
            G2_R,       // Bottom right goal
            CTR,        // Centre of the field
            TEAMMATE,   // Teammate robot       // TODO Implement this
            ENEMY       // Enemy robot          // TODO Implement this
        };

        Type type;
        SubType sub_type;
        float pol[3];


        Observation(Type t, SubType st, float pos[3])
            : type(t), sub_type(st)
        {
            memcpy(pol, pos, 3*sizeof(float));
        }

        Observation()
            : type(NO_TYPE), sub_type(NO_SUBTYPE)
        { }

    };
}

#endif   // SIMULATION_OBSERVATION_H_
