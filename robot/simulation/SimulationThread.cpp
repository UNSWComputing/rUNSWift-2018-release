#include "SimulationThread.hpp"
#include "types/AbsCoord.hpp"
#include "types/SensorValues.hpp"
#include "types/JointValues.hpp"


#include <sys/mman.h>        /* For shared memory */
#include <fcntl.h>           /* For O_* constants */
#include <arpa/inet.h>	     /* To convert size to network order */
#include <iostream>
#include <cmath>

// Boost
#include <boost/lexical_cast.hpp>
#include <boost/thread/thread.hpp>
#include <boost/pointer_cast.hpp>


SimulationThread::SimulationThread(Blackboard *bb)
    : Adapter(bb)
    , connection_()
    , vision_(bb)
    , angle_()
    , sonar_()
    , prev_rlj1_(0)
    , team_((bb->config)["player.team"].as<int>())
    , player_number_((bb->config)["player.number"].as<int>())
    , econt_(bb)

{
    // open shared memory as RW
    std::string mem_path (AGENT_MEMORY);

    // Append the team / player number to the shared memory path so we don't 
    // have multiple instances of sim build using the same shared memory (this
    // means multiple instances MUST use different player numbers).
    int mod = (team_ * MAX_NUM_PLAYERS) + player_number_;
    std::stringstream ss;
    ss << mod;
    mem_path += ss.str();
    
    shared_fd_ = shm_open(mem_path.c_str(), O_CREAT | O_RDWR, 0600);
    if (shared_fd_ < 0) {
        throw std::runtime_error("AgentEffector: shm_open() failed");
    }

    // map shared memory to process memory
    shared_data_ = (AgentData*) mmap(NULL, sizeof(AgentData),
                                    PROT_READ | PROT_WRITE,
                                    MAP_SHARED, shared_fd_, 0);

    if (shared_data_ == MAP_FAILED)
    {
        throw std::runtime_error("AgentEffector: mmap() failed");
    }

    if (ftruncate(shared_fd_, sizeof(AgentData)) == -1)
    {
        throw std::runtime_error("ftruncate() failed");
    }

    // create semaphore with permissions 600, value 0
    std::string sem_path (AGENT_SEMAPHORE);
    sem_path += ss.str();

    semaphore_ = sem_open(sem_path.c_str(), O_RDWR | O_CREAT, 0600, 0);
    if (semaphore_ < 0)
    {
        throw std::runtime_error("sem_open() failed");
    }

    shared_data_->init();

    SensorValues null_sensors;
    JointValues null_joints;
    ActionCommand::LED null_leds;

    int i;
    for (i = 0; i < Joints::NUMBER_OF_JOINTS; ++i) {
        null_joints.angles[i] = 0.0f;
        null_joints.stiffnesses[i] = 0.0f;
        null_joints.temperatures[i] = 0.0f;
        last_simulator_actuation_.angles[i] = 0.0f;
        last_simulator_perception_.angles[i] = 0.0f;
    }
    null_sensors.joints = null_joints;
    for (i = 0; i < Sensors::NUMBER_OF_SENSORS; ++i)
        null_sensors.sensors[i] = 0.0f;

    for (i = 0; i < Sonar::NUMBER_OF_READINGS; ++i)
        null_sensors.sonar[i] = Sonar::DISCARD;    

    for (i = 0; i < 3; ++i) {
        shared_data_->sensors[i] = null_sensors;
        shared_data_->joints[i] = null_joints;
        shared_data_->leds[i] = null_leds;
        shared_data_->sonar[i] = Sonar::Mode::NO_PING;
    }
    shared_data_->standing = true;

    srand(time(NULL));

    // Find player starting position
    // TODO tweak / optimise these
    // TODO move to separate function
    double starting_x = 0;
    double starting_y = 0;
    double starting_orientation = 0;
    switch (player_number_)
    {
        case 2:
            starting_x = -3.25;
            starting_y = 3;
            starting_orientation = -90;
            break;

        case 4:
            starting_x = -1.75;
            starting_y = 3;
            starting_orientation = -90;
            break;

        case 1:
            starting_x = -3;
            starting_y = -3;
            starting_orientation = 90;
            break;

        case 3:
            starting_x = -2;
            starting_y = -3;
            starting_orientation = 90;
            break;

        case 5:
            starting_x = -1;
            starting_y = -3;
            starting_orientation = 90;
            break;
    }

    std::string team_name = "runswift";
    if (team_ != 18)
    {
        std::stringstream ss;
        ss << "Team" << team_;
        team_name = ss.str();
    }

    //TODO configurable simulation endpoint
    if (!connection_.init("localhost", 3100, team_name, player_number_, starting_x, starting_y, starting_orientation))
    {
        throw std::runtime_error("Simulation connection failed");
    }

    if (!econt_.init("localhost", 3232))
    {
        std::cout << "Warning: could not connect to econtroller.\n";
    }

    // Joints debug
    for (int j=0; j < Joints::NUMBER_OF_JOINTS; ++j)
    {
        predictions_[j] = 0.0f;
    }
    
}

SimulationThread::~SimulationThread()
{

}

void SimulationThread::tick()
{
    sim_timer_.restart();
    // TODO revisit gyrX/Y and angleX/Y and 'GetupGenerator'

    // Receive perceptor info from server
    PerceptorInfo recv;
    // NOTE: Angles are converted from degrees to radians here
    if (!connection_.receivePerceptorInfo(&recv))
    {
        std::cerr << "Error receiving perceptor info!\n";
    }

    // Tick simulator vision
    vision_.tick(recv);

    // Here we need to write the info we received from the simulator to
    // the shared memory. This is effectively replacing the workflow of
    // libagent.
    SensorValues s;
    recv.toSensorValues(&s);
    JointValues simulator_perception = s.joints; // Store current joint angles (in radians)
                        // so we can map from absolute to relative


    // Find our AngleX and AngleY values (sim doesn't provide AngleX/Y sensors)
    angle_.addMeasurement(s.sensors[8], s.sensors[9], s.sensors[10],
                          s.sensors[11], s.sensors[12], s.sensors[13]);
    angle_.getAngles(&s.sensors[6], &s.sensors[7]);

    // Add sonar measurements (for closest object only)
    sonar_.addMeasurement(recv);
    sonar_.getSonar(s.sonar);

    // Find the right index in shared memory to write sim data to so rUNSWift
    // can access it
    int i;
    for (i = 0; i != shared_data_->sensors_latest &&
        i != shared_data_->sensors_read; ++i)
    {  }
    shared_data_->sensors[i] = s;
    shared_data_->sensors_latest = i;

    // Update the ExperimentController
    econt_.tick();

    // Post the semaphore twice so motion can double tick
    for (int j=0; j < 2; ++j) {
       sem_post(semaphore_);
    }

    // Read values from motion
    // Angles are in radians here
    shared_data_->actuators_read = shared_data_->actuators_latest;
    
    // This is our motion request
    JointValues motion_joint_request = shared_data_->joints[shared_data_->actuators_read];

    // This is our modified request (converted from absolute to relative)
    JointValues next_simulator_actuation = motion_joint_request;
    

    // rUNSWift deals with absolute radian angles, and the simulator takes
    // relative angles in degrees (which are maintained over ticks until
    // changed).
    //
    // Here we take the difference of the radian angle requested by rUNSWift
    // and the current radian angle of the joint provided by the simulator.
    //
    // We then find the difference in those angles, so we know which way to
    // move the joint. If the change is too big, we cap it with a max velocity
    // and if it is too small, we set it to 0.
    //
    // We then set the relative change in radians, which is converted to
    // degrees upon sending.
    float average_error = 0;
    const float lle1_request = motion_joint_request.angles[8];
    for (int i = 0; i < Joints::NUMBER_OF_JOINTS; ++i)
    {        
        // The simulator effector commands are delayed by one tick,
        // which means that the joint values that we received this tick
        // are essentially one tick out of sync. By the time our effector
        // commands are enacted, we will be working with different joint
        // values.
        //
        // This is a problem because the effector commands take motor velocity,
        // which we derive from taking the difference between the requested
        // joint angle and the current joint angle.
        //
        // A workaround for this is to predict the value of the joint when
        // the effector command will actually be enacted, in one ticks time.
        // Then, find the relative joint difference using this predicted value.
        
        // Warn for high joint prediction errors
        float prediction_difference = simulator_perception.angles[i] - predictions_[i];
        average_error += prediction_difference;
        if (fabs(prediction_difference) > 0.2)
        {
            std::cerr << "Warning: high joint prediction error " << prediction_difference << " in joint " << i << ". This may cause instability.\n";
        }

        // Predict the joint value for next tick. It appears that the joints
        // move approx 1.22173 * velocity per tick.
        float prediction = simulator_perception.angles[i] 
                           + (DEFAULT_VEL_PER_TICK * last_simulator_actuation_.angles[i]);
        predictions_[i] = prediction;

        // Find difference between requested position for this tick and the
        // predicted position for this tick.
        // (Converting from absolute to relative angle [but still radians])
        float difference = motion_joint_request.angles[i] - prediction;

        // The requested joint moves  ~1.22173 * velocity per tick
        // (this may vary).
        //
        // So we should set our velocity as follows:
        //      velocity = difference / vel_per_tick;
        //
        float velocity =  difference ? (difference / DEFAULT_VEL_PER_TICK) : 0;

        // Looks like the max speed for any joint is ~0.1, so lets cap our
        // speed there
        // TODO research MAX_SPEED for different joints
        const double MAX_SPEED = 0.1;
        if (fabs(velocity) > MAX_SPEED)
        {
            velocity = (difference > 0 ? MAX_SPEED : -MAX_SPEED);
        }

        // Change joint by speed
        next_simulator_actuation.angles[i] = velocity;
    }

    // Build effector command for server
    EffectorCommand cmd;
    if (cmd.joints.fromJointValues(next_simulator_actuation))
    {
        // rUNSWift does not interact with the RHipYawPitch, so it does not
        // provide an interface for doing so. As such, we need to have
        // additional code for setting the RHipYawPitch, which will use the
        // same desired joint angle as LHipYawPitch (on the robot, these joints
        // are fused).
        //
        // TODO move this hack for RHipYawPitch
        float prediction = DEG2RAD(recv.joints.rlj1) + (DEFAULT_VEL_PER_TICK * prev_rlj1_);
        // Use same angle as LHipYawPitch
        float difference = lle1_request - prediction;
        float velocity =  difference ? (difference / DEFAULT_VEL_PER_TICK) : 0;
        const double MAX_SPEED = 0.1;
        if (fabs(velocity) > MAX_SPEED)
        {
            velocity = (difference > 0 ? MAX_SPEED : -MAX_SPEED);
        }
        // Convert to degrees (EffectorCommand uses degrees)
        cmd.joints.rlj1 = RAD2DEG(velocity);
        prev_rlj1_ = velocity;

        // If it's valid, send!
        connection_.sendEffectorCommand(cmd);
    }
    last_simulator_actuation_ = next_simulator_actuation;
    last_simulator_perception_ = simulator_perception;

    // rUNSWift is on a 10ms tick but simulation server is 20ms
    // NOTE We shouldn't need to sleep because we should on the 
    // server, but let's sleep just incase
    double elapsed = sim_timer_.elapsed_us();
    if (elapsed < THREAD_TICK_RATE)
    {
        boost::this_thread::sleep(boost::posix_time::microseconds(THREAD_TICK_RATE - elapsed));
    }
    //std::cout << "Simulation thread: " << sim_timer_.elapsed_us() << "us\n";
}


