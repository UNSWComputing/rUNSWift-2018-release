#ifndef SIMULATION_SIMULATIONTHREAD_H_
#define SIMULATION_SIMULATIONTHREAD_H_

#include "AngleSensor.hpp"
#include "experiments/ExperimentControllerConnection.hpp"
#include "SimulationConnection.hpp"
#include "SimVisionAdapter.hpp"
#include "SonarSensor.hpp"
#include "utils/Timer.hpp"
#include "blackboard/Adapter.hpp"
#include "libagent/AgentData.hpp"

#include <semaphore.h>



using namespace Simulation;

/* Wrapper class for simulator thread */
class SimulationThread : Adapter {
public:
    /* Constructor */
    SimulationThread(Blackboard *bb);

    /* Destructor */
    ~SimulationThread();

    /* One cycle of this thread */
    void tick();

private:
    static const int THREAD_TICK_RATE  = 19000;
    static const float DEFAULT_VEL_PER_TICK = 1.22173; // TODO move this somewhere else

    int shared_fd_;
    sem_t* semaphore_;
    AgentData* shared_data_;
    SimulationConnection connection_;               /**< Handles TCP connection to simulation server */
    SimVisionAdapter vision_;

    JointValues last_simulator_perception_;
    JointValues last_simulator_actuation_;
    AngleSensor angle_;
    SonarSensor sonar_;

    float prev_rlj1_;
    int team_;                          // Team 
    int player_number_;                 // Player number

    ExperimentControllerConnection econt_;
    Timer sim_timer_;

    // Angle debugging
    float predictions_[Joints::NUMBER_OF_JOINTS];

};

#endif   // SIMULATION_SIMULATIONTHREAD_H_


