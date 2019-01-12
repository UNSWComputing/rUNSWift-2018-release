#include "Joints.hpp"

#include <utils/angles.hpp>

#include <iomanip>

namespace Simulation {

    bool Joints::fromJointValues(const JointValues& joints)
    {
        for (int i=0; i < ::Joints::NUMBER_OF_JOINTS; ++i)
        {
            // Sometimes we get NaN values for our joints - we want to
            // ignore these frames (TODO investigate further)
            if (isnan(joints.angles[i]) || jptrs[i] == &na)
            {
                *jptrs[i] = 0;
            }
            else
            {
                *jptrs[i] = RAD2DEG(joints.angles[i]);
                if (invertJoint(i))
                {
                    *jptrs[i] = -*jptrs[i];
                }
            }
        }
       return true;
    }

    bool Joints::toJointValues(JointValues* joints_out)
    {
        na = 0.0f;
        JointValues& joints = *joints_out;
        for (int i=0; i < ::Joints::NUMBER_OF_JOINTS; ++i)
        {
            joints.angles      [i] = DEG2RAD(*jptrs[i]);

            // Fix any angle inversions between rUNSWift and the simulator
            if (invertJoint(i))
            {
                joints.angles[i] = -joints.angles[i];
            }

            joints.temperatures[i] = 0;
            joints.currents    [i] = 0.5; // I believe this is legitimately the electircal current applied to the joint (stiffness)
        }
        return true;
    }

    bool Joints::invertJoint(int j)
    {
        switch(j)
        {
            case 1:     // HeadPitch
                return true;
            case 2:     // LShoulderPitch
                return true;
            case 10:    // LHipPitch
                return true;
            case 11:    // LKneePitch
                return true;
            case 12:    // LAnklePitch
                return true;
            case 15:    // RHipPitch
                return true;
            case 16:    // RKneePitch
                return true;
            case 17:    // RAnklePitch
                return true;
            case 19:    // RShoulderPitch
                return true;
            default:    // Don't invert other joints
                return false;
        }
    }


   bool Joints::setValue(const std::string& name, float value)
    {
        float* to_set = NULL;
        if (name == "hj1")
        {
            to_set = &hj1;
        }
        else if (name == "hj2")
        {
            to_set = &hj2;
        }
        else if (name == "raj1")
        {
            to_set = &raj1;
        }
        else if (name == "raj2")
        {
            to_set = &raj2;
        }
        else if (name == "raj3")
        {
            to_set = &raj3;
        }
        else if (name == "raj4")
        {
            to_set = &raj4;
        }
        else if (name == "laj1")
        {
            to_set = &laj1;
        }
        else if (name == "laj2")
        {
            to_set = &laj2;
        }
        else if (name == "laj3")
        {
            to_set = &laj3;
        }
        else if (name == "laj4")
        {
            to_set = &laj4;
        }
        else if (name == "rlj1")
        {
            to_set = &rlj1;
        }
        else if (name == "rlj2")
        {
            to_set = &rlj2;
        }
        else if (name == "rlj3")
        {
            to_set = &rlj3;
        }
        else if (name == "rlj4")
        {
            to_set = &rlj4;
        }
        else if (name == "rlj5")
        {
            to_set = &rlj5;
        }
        else if (name == "rlj6")
        {
            to_set = &rlj6;
        }
        else if (name == "llj1")
        {
            to_set = &llj1;
        }
        else if (name == "llj2")
        {
            to_set = &llj2;
        }
        else if (name == "llj3")
        {
            to_set = &llj3;
        }
        else if (name == "llj4")
        {
            to_set = &llj4;
        }
        else if (name == "llj5")
        {
            to_set = &llj5;
        }
        else if (name == "llj6")
        {
            to_set = &llj6;
        }

        if (!to_set)
        {
            return false;
        }
        *to_set = value;
        return true;
    }
}

std::ostream& operator<<(std::ostream& out, const Simulation::Joints& joints)
{
    out << std::fixed << std::setprecision(2);
    out << "(he1 " << joints.hj1 << ")";
    out << "(he2 " << joints.hj2 << ")";
    out << "(rae1 " << joints.raj1 << ")";
    out << "(rae2 " << joints.raj2 << ")";
    out << "(rae3 " << joints.raj3 << ")";
    out << "(rae4 " << joints.raj4 << ")";
    out << "(lae1 " << joints.laj1 << ")";
    out << "(lae2 " << joints.laj2 << ")";
    out << "(lae3 " << joints.laj3 << ")";
    out << "(lae4 " << joints.laj4 << ")";
    out << "(rle1 " << joints.rlj1 << ")";
    out << "(rle2 " << joints.rlj2 << ")";
    out << "(rle3 " << joints.rlj3 << ")";
    out << "(rle4 " << joints.rlj4 << ")";
    out << "(rle5 " << joints.rlj5 << ")";
    out << "(rle6 " << joints.rlj6 << ")";
    out << "(lle1 " << joints.llj1 << ")";
    out << "(lle2 " << joints.llj2 << ")";
    out << "(lle3 " << joints.llj3 << ")";
    out << "(lle4 " << joints.llj4 << ")";
    out << "(lle5 " << joints.llj5 << ")";
    out << "(lle6 " << joints.llj6 << ")";
    return out;
}




