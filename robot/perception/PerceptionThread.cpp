#include "perception/PerceptionThread.hpp"

#include <pthread.h>
#include <ctime>
#include <utility>
#include <boost/algorithm/string.hpp>
#include <boost/algorithm/string/classification.hpp>

#include "perception/dumper/PerceptionDumper.hpp"
#include "perception/behaviour/SafetySkill.hpp"
#include "soccer.hpp"
#include "blackboard/Blackboard.hpp"
#include "utils/Logger.hpp"
#include "thread/Thread.hpp"
#include "boost/lexical_cast.hpp"
#include <boost/bind.hpp>
#include <boost/thread/thread.hpp>

using namespace std;
using namespace boost;

PerceptionThread::PerceptionThread(Blackboard *bb)
   : Adapter(bb),
     kinematicsAdapter(bb),
#ifndef SIMULATION
     visionAdapter(bb),
#endif
     localisationAdapter(bb),
     behaviourAdapter(bb),
     bb_(bb)
{
   dumper = NULL;

   releaseLock(serialization);
   //uint8_t const* currentFrame = readFrom(vision, currentFrame);
   uint8_t const* topFrame = readFrom(vision, topFrame);
   uint8_t const* botFrame = readFrom(vision, botFrame);
   size_t ret;

   if (topFrame != NULL && botFrame != NULL) {
      string file = "/home/nao/crashframe-" +
                    boost::lexical_cast<string>(time(NULL)) + ".yuv";
      FILE *errorFrameFile = fopen(file.c_str(), "w");
      ret = fwrite(topFrame, 640 * 480 * 2, 1, errorFrameFile);
      ret = fwrite(botFrame, 640 * 480 * 2, 1, errorFrameFile);
      fclose(errorFrameFile);
      file = "/usr/bin/tail -n 200 /var/volatile/runswift/*/Perception > "
             + file + ".log";
      ret = std::system(file.c_str());
   }


   readOptions(bb->config);
   writeTo(thread, configCallbacks[Thread::name],
           boost::function<void(const boost::program_options::variables_map &)>
           (boost::bind(&PerceptionThread::readOptions, this, _1)));
}

PerceptionThread::~PerceptionThread() {
   llog(INFO) << __PRETTY_FUNCTION__ << endl;
   writeTo(thread, configCallbacks[Thread::name], boost::function<void(const boost::program_options::variables_map &)>());
}

void PerceptionThread::tick() {
   llog_open(VERBOSE) << "Perception Thread" << endl;

   Timer timer_thread;
   Timer timer_tick;

   /*
    * Kinematics Tick
    */
   llog_open(VERBOSE) << "Kinematics Tick" << endl;
   timer_tick.restart();
   kinematicsAdapter.tick();

   uint32_t kinematics_time = timer_tick.elapsed_us();
   if (kinematics_time > TICK_MAX_TIME_KINEMATICS) {
      llog_close(VERBOSE) << "Kinematics Tick: OK " << kinematics_time << " us" << endl;
   } else {
      llog_close(ERROR) << "Kinematics Tick: TOO LONG " << kinematics_time << " us" << endl;
   }

   /*
    * Vision Tick
    */
   llog_open(VERBOSE) << "Vision Tick" << endl;
   timer_tick.restart();
#ifndef SIMULATION
   visionAdapter.tick();
#endif

   uint32_t vision_time = timer_tick.elapsed_us();
   if (vision_time > TICK_MAX_TIME_VISION) {
      llog_close(VERBOSE) << "Vision Tick: OK " << vision_time << endl;
   } else {
      llog_close(ERROR) << "Vision Tick: TOO LONG " << vision_time << endl;
   }

   /*
    * Localisation Tick
    */
   llog_open(VERBOSE) << "Localisation Tick" << endl;
   timer_tick.restart();
   localisationAdapter.tick();

   uint32_t localisation_time = timer_tick.elapsed_us();
   if (localisation_time > TICK_MAX_TIME_LOCALISATION) {
      llog_close(VERBOSE) << "Localisation Tick: OK " << localisation_time << endl;
   } else {
      llog_close(ERROR) << "Localisation Tick: TOO LONG " << localisation_time << endl;
   }

   /*
    * Behaviour Tick
    */
   llog_open(VERBOSE) << "Behaviour Tick" << endl;
   timer_tick.restart();
   pthread_yield();

   if (time(NULL) - readFrom(remoteControl, time_received) < 60) {
     /* we have fresh remote control data, use it */
     int writeBuf = (readFrom(behaviour, readBuf) + 1) % 2;
     writeTo(behaviour, request[writeBuf], readFrom(remoteControl, request));
     writeTo(behaviour, readBuf, writeBuf);
   } else {
      behaviourAdapter.tick();
   }

   uint32_t behaviour_time = timer_tick.elapsed_us();
   if (behaviour_time > TICK_MAX_TIME_BEHAVIOUR) {
      llog_close(VERBOSE) << "Behaviour Tick (and perception yield): OK " << behaviour_time << endl;
   } else {
      llog_close(ERROR) << "Behaviour Tick (and perception yield): TOO LONG " << behaviour_time << endl;
   }

#ifdef SIMULATION
   // Introduce delay to componsate for vision processing
   // (so behaviours runs at the correct speed)
   boost::this_thread::sleep(boost::posix_time::milliseconds(32));
#endif

   /*
    * Finishing Perception
    */
   uint32_t perception_time = timer_thread.elapsed_us();
   if (perception_time > THREAD_MAX_TIME) {
      llog_close(VERBOSE) << "Perception Thread: OK " << perception_time << endl;
   } else {
      llog_close(ERROR) << "Perception Thread: TOO LONG " << perception_time << endl;
   }

   writeTo(perception, kinematics, kinematics_time);
   writeTo(perception, vision, vision_time);
   writeTo(perception, localisation, localisation_time);
   writeTo(perception, behaviour, behaviour_time);
   writeTo(perception, total, perception_time);

   if (dumper) {
      if (dump_timer.elapsed_us() > dump_rate) {
         dump_timer.restart();
         try {
            dumper->dump(bb_);
         } catch(const std::exception &e) {
            attemptingShutdown = true;
            cout << "Error: " << e.what() << endl;
         }
      }
   }

}

void PerceptionThread::readOptions(const boost::program_options::variables_map& config) {
#ifndef SIMULATION
   const string &e = config["vision.camera_controls"].as<string>();
   vector<string> vs;
   split(vs, e, is_any_of(",;"));
   for (vector<string>::const_iterator ci = vs.begin(); ci != vs.end(); ++ci) {
      vector<string> nv;
      split(nv, *ci, is_any_of(":"));
      if (nv.size() != 3)
         llog(ERROR) << "controls should be cam:control_id:value" << endl;
      else{
         Camera *currCamera;
         if(strtol(nv[0].c_str(),NULL,10) == 0){
            currCamera = visionAdapter.combined_camera_->getCameraBot();
         }else{
            currCamera = visionAdapter.combined_camera_->getCameraTop();
         }
         if(strtoul(nv[1].c_str(), NULL, 10) == 0)
            currCamera->setControl(22,1);
         if(strtoul(nv[1].c_str(), NULL, 10) == 17)
            currCamera->setControl(22,0);
         if(strtoul(nv[1].c_str(), NULL, 10) == 19)
            currCamera->setControl(22,0);
         currCamera->setControl(strtoul(nv[1].c_str(), NULL, 10),
                                strtol (nv[2].c_str(), NULL, 10));
      }
   }
#endif

   const string &dumpPath = config["debug.dump"].as<string>();
   dump_rate = config["vision.dumprate"].as<int>() * 1000;
   if (dumpPath == "") {
      delete dumper;
      dumper = NULL;
   } else {
      if (! dumper || dumper->getPath() != dumpPath) {
         delete dumper;
         dumper = new PerceptionDumper(dumpPath.c_str());
      }
   }

   OffNaoMask_t dumpMask = config["debug.mask"].as<int>();
   bb_->write(&(bb_->mask), dumpMask);
}

