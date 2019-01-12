#pragma once

#include <Python.h>
#include <string>
#include "perception/vision/VisionAdapter.hpp"
#include "perception/localisation/LocalisationAdapter.hpp"
#include "perception/behaviour/BehaviourAdapter.hpp"
#include "perception/kinematics/KinematicsAdapter.hpp"
#include "perception/dumper/PerceptionDumper.hpp"
#include "perception/vision/camera/CombinedCamera.hpp"
#include "blackboard/Adapter.hpp"

#define THREAD_MAX_TIME 33666

#define TICK_MAX_TIME_VISION 30000
#define TICK_MAX_TIME_KINEMATICS 30000
#define TICK_MAX_TIME_LOCALISATION 30000
#define TICK_MAX_TIME_BEHAVIOUR 30000


/* Wrapper class for vision, localisation and behaviour threads */
class PerceptionThread : Adapter {
   public:
      /* Constructor */
      PerceptionThread(Blackboard *bb);
      /* Destructor */
      ~PerceptionThread();

      /* Read from global options after update */
      void readOptions(const boost::program_options::variables_map &config);

      /* One cycle of this thread */
      void tick();

   private:
      KinematicsAdapter kinematicsAdapter;
#ifndef SIMULATION
      VisionAdapter visionAdapter;
#endif
      //SonarAdapter sonarAdapter;
      LocalisationAdapter localisationAdapter;
      BehaviourAdapter behaviourAdapter;

      Blackboard* bb_;

      PerceptionDumper *dumper;
      Timer dump_timer;
      unsigned int dump_rate;
};

