#ifndef SIMULATION_JOINTS_H_
#define SIMULATION_JOINTS_H_

#include <types/JointValues.hpp>

#include <string>
#include <ostream>

namespace Simulation {

    /*
     *   The Joints class provides a representation of simulated joints.
     *   Joint information received from the server will be held in an
     *   instance of this class, and joint commands sent to the server
     *   should be held in an instance of this class.
     *
     *   Libagent uses 25 joints but the sim only provides 22. The relationship
     *   between these sets of joints is defined below.
     *
     *   Libagent joint order:
     *     1. HeadYaw
     *     2. HeadPitch
     *     3. LShoulderPitch
     *     4. LShoulderRoll
     *     5. LElbowYaw
     *     6. LElbowRoll
     *     7. LWristYaw
     *     8. LHand
     *     9. LHipYawPitch
     *     10. LHipRoll
     *     11. LHipPitch
     *     12. LKneePitch
     *     13. LAnklePitch
     *     14. LAnkleRoll
     *     15. RHipRoll
     *     16. RHipPitch
     *     17. RKneePitch
     *     18. RAnkleRoll
     *     19. RAnklePitch
     *     20. RShoulderPitch
     *     21. RShoulderRoll
     *     22. RElbowYaw
     *     23. RElbowYaw
     *     24. RWristYaw
     *     25. RHand
     *
     *  Interestingly enough, the left leg has a HipYawPitch but the
     *  right leg does not.
     *
     *   NOTE: This is because on the Nao V5, the HipYawPitch joints
     *   are fused, meaning moving one of the joints to some angle n
     *   will result in the other HipYawPitch joint moving to the same
     *   angle. To deal with this, libagent hides the right HipYawPitch
     *   and just provides an interface to the left. I imagine that it
     *   simply mirrors the LHipYawPitch angle to RHipYawPitch.
     *
     *
     *  Looks like we have the following joints from the sim (22):
     *     1. NeckYaw           (hj1)
     *     2. NeckPitch         (hj2)
     *     3. LShoulderPitch    (laj1)
     *     4. LShoulderYaw      (laj2)
     *     5. LArmYaw           (laj3)
     *     6. LArmRoll          (laj4)
     *     7. RShoulderPitch    (raj1)
     *     8. RShoulderYaw      (raj2)
     *     9. RArmYaw           (raj3)
     *     10. RArmRoll         (raj4)
     *     11. LHipYawPitch     (llj1)
     *     12. LHipRoll         (llj2)
     *     13. LHipPitch        (llj3)
     *     14. LKneePitch       (llj4)
     *     15. LFootPitch       (llj5)
     *     16. LFootRoll        (llj6)
     *     17. RHipYawPitch     (rlj1)
     *     18. RHipRoll         (rlj2)
     *     19. RHipPitch        (rlj3)
     *     20. RKneePitch       (rlj4)
     *     21. RFootPitch       (rlj5)
     *     22. RFootRoll        (rlj6)
     *
     *
     *  The mapping from rUNSWift to simulation joints will be as follows:
     *     1. HeadYaw -> NeckYaw
     *     2. HeadPitch -> NeckPitch
     *     3. LShoulderPitch -> LShoulderPitch
     *     4. LShoulderRoll -> LShoulderYaw
     *     5. LElbowYaw -> LArmRoll
     *     6. LElbowRoll ->  LArmYaw
     *     7. MISMATCH: LWristYaw -> N/A
     *     8. MISMATCH: LHand -> N/A
     *     9. LHipYawPitch -> LHipYawPitch
     *     10. LHipRoll -> LHipRoll
     *     11. LHipPitch -> LHipPitch
     *     12. LKneePitch -> LKneePitch
     *     13. LAnklePitch -> LFootPitch
     *     14. LAnkleRoll -> LFootRoll
     *     15. MISMATCH: N/A -> RHipYawPitch
     *     16. RHipRoll -> RHipRoll
     *     17. RHipPitch -> RHipPitch
     *     18. RKneePitch -> RKneePitch
     *     19. RAnklePitch -> RFootPitch
     *     20. RAnkleRoll -> RFootRoll
     *     21. RShoulderPitch -> RShoulderPitch
     *     22. RShoulderRoll -> RShoulderYaw
     *     23. RElbowYaw -> RArmRoll
     *     24. RElbowRoll -> RArmYaw
     *     25. MISMATCH: RWristYaw -> N/A
     *     26. MISMATCH: RHand -> N/A
     *
     */
    class Joints
    {
    public:
        /* Head */
        float hj1;
        float hj2;

        /* Right arm */
        float raj1;
        float raj2;
        float raj3;
        float raj4;

        /* Left arm */
        float laj1;
        float laj2;
        float laj3;
        float laj4;

        /* Right leg */
        float rlj1;
        float rlj2;
        float rlj3;
        float rlj4;
        float rlj5;
        float rlj6;

        /* Left leg */
        float llj1;
        float llj2;
        float llj3;
        float llj4;
        float llj5;
        float llj6;

        /* Unavailable joints */
        float na;

        Joints()
            : hj1(0)
            , hj2(0)
            , raj1(0)
            , raj2(0)
            , raj3(0)
            , raj4(0)
            , laj1(0)
            , laj2(0)
            , laj3(0)
            , laj4(0)
            , rlj1(0)
            , rlj2(0)
            , rlj3(0)
            , rlj4(0)
            , rlj5(0)
            , rlj6(0)
            , llj1(0)
            , llj2(0)
            , llj3(0)
            , llj4(0)
            , llj5(0)
            , llj6(0)
        {
            jptrs[0] = &hj1;      // HeadYaw -> NeckYaw
            jptrs[1] = &hj2;      // HeadPitch -> NeckPitch
            jptrs[2] = &laj1;     // LShoulderPitch -> LShoulderPitch
            jptrs[3] = &laj2;     // LShoulderRoll -> LShoulderYaw
            jptrs[4] = &laj3;     // LElbowYaw -> LArmRoll
            jptrs[5] = &laj4;     // LElbowRoll ->  LArmYaw
            jptrs[6] = &na;       // MISMATCH: LWristYaw -> N/A
            jptrs[7] = &na;       // MISMATCH: LHand -> N/A
            jptrs[8] = &llj1;     // LHipYawPitch -> LHipYawPitch
            jptrs[9] = &llj2;     // LHipRoll -> LHipRoll
            jptrs[10] = &llj3;     // LHipPitch -> LHipPitch
            jptrs[11] = &llj4;     // LKneePitch -> LKneePitch
            jptrs[12] = &llj5;     // LAnklePitch -> LFootPitch
            jptrs[13] = &llj6;     // LAnkleRoll -> LFootRoll
             //jptrs[i++] = &rlj1;  // MISMATCH: N/A -> RHipYawPitch
            jptrs[14] = &rlj2;     // RHipRoll -> RHipRoll
            jptrs[15] = &rlj3;     // RHipPitch -> RHipPitch
            jptrs[16] = &rlj4;     // RKneePitch -> RKneePitch
            jptrs[17] = &rlj5;     // RAnklePitch -> RFootPitch
            jptrs[18] = &rlj6;     // RAnkleRoll -> RFootRoll
            jptrs[19] = &raj1;     // RShoulderPitch -> RShoulderPitch
            jptrs[20] = &raj2;     // RShoulderRoll -> RShoulderYaw
            jptrs[21] = &raj3;     // RElbowYaw -> RArmRoll
            jptrs[22] = &raj4;     // RElbowRoll -> RArmYaw
            jptrs[23] = &na;       // MISMATCH: RWristYaw -> N/A
            jptrs[24] = &na;       // MISMATCH: RHand -> N/A
         }

        /*
         * Reads in rUNSWift JointValues. This essentially converts from
         * rUNSWift joint angles to simulator joint angles.
         *
         * @param joints The rUNSWift joints to read in.
         * @return bool Returns true if successful. False is unsuccessful.
         */
        bool fromJointValues(const JointValues& joints);

        /*
         * Writes the simulator joint angle to 'joints_out'. This essentially
         * converts from simulator joint angles to rUNSWift joint angles.
         *
         * @param joints_out The JointValues struct to write simulator joint
         * angles to.
         * @return bool True indicates success. False indicates failure.
         */
        bool toJointValues(JointValues* joints_out);

        /*
         * Sets the given angle of the joint identified by name.
         *
         * @param name The name of the joint to set the value of.
         * @param value The vale to set the joint
         * @return bool True indicates success. False indicates failure.
         */
        bool setValue(const std::string& name, float value);

    private:
        float* jptrs[Sensors::NUMBER_OF_SENSORS]; /**< Holds joint angles */
        bool invertJoint(int j); /**< Indicates whether or not the given joint
                                    angle should be inverted due to interface
                                    mismatch between rUNSWift and simulator. */

    }; // end class Joints

} // namespace Simulation

/*
 * Writes the joint names and their angle values to the outstream.
 *
 * @param out The outstream to write to.
 * @param joints The joint values to write.
 * @return std::ostream The ostream that was supplied, to allow operator
 * chaining.
 */
std::ostream& operator<<(std::ostream& out, const Simulation::Joints& joints);


#endif // SIMULATION_JOINTS_H_

