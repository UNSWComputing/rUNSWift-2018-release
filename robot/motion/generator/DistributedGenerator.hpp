#pragma once

#include <map>
#include <string>
#include "motion/generator/Generator.hpp"
#include "blackboard/Blackboard.hpp"

class DistributedGenerator : Generator {
   public:
      explicit DistributedGenerator(Blackboard *bb);
      ~DistributedGenerator();
      virtual JointValues makeJoints(ActionCommand::All* request,
                                     Odometry* odometry,
                                     const SensorValues &sensors,
                                     BodyModel &bodyModel,
                                     float ballX,
                                     float ballY);
      virtual bool isActive();
      void reset();
      void readOptions(const boost::program_options::variables_map &config);
      bool isStopping;

   private:
      Generator* bodyGenerators[ActionCommand::Body::NUM_ACTION_TYPES];
      Generator* headGenerator;
      ActionCommand::Body::ActionType current_generator;
      ActionCommand::Body::ActionType prev_generator;
      ActionCommand::Body::ActionType requestedDive;
};
