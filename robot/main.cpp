#include <Python.h>

#include <boost/program_options.hpp>
#include <signal.h>
#include <errno.h>
#include <fcntl.h>
#include <vector>
#include <map>
#include <iostream>
#include <string>
#include <sys/file.h>

#include "soccer.hpp"

#include "utils/Logger.hpp"
#include "utils/options.hpp"
#include "utils/set_cloexec.hpp"

#include "perception/behaviour/python/RegisterConverters.hpp"

#include "thread/ThreadManager.hpp"
#include "motion/MotionAdapter.hpp"
#include "transmitter/OffNao.hpp"
#include "transmitter/Team.hpp"
#include "receiver/Team.hpp"
#include "receiver/RemoteControl.hpp"
#include "gamecontroller/GameController.hpp"
#include "perception/PerceptionThread.hpp"
#include "perception/vision/Vision.hpp"
#include "perception/vision/camera/NaoCamera.hpp"
#include "perception/vision/camera/NaoCameraV4.hpp"
#include "perception/vision/camera/CombinedCamera.hpp"

#include "simulation/SimulationThread.hpp"

#define handle_error_en(en, msg) \
   do { \
      errno = en; \
      perror(msg); \
      exit(1); \
   } while (0)

namespace po = boost::program_options;
using namespace std;
using namespace boost;

extern string git_version;

/**
 * A timer function.  After arg seconds, runswift will shut down.
 *
 * @param arg coerced to int.  seconds to shutdown
 * @returns NULL
 */
static void *shutdownTimer(void * arg) {
   int time = (int)arg;
   if (time > 5) {
      sleep(time - 5);
      SAY("shutting down");
      sleep(5);
   } else {
      sleep(time);
   }
   attemptingShutdown = true;
   return NULL;
}

/** Entry point for runswift application */
int main(int argc, char **argv) {

#ifndef SIMULATION
   // http://stackoverflow.com/questions/5339200/how-to-create-a-single-instance-application-in-c-or-c
   int pid_file = open("/var/volatile/runswift.pid", O_CREAT | O_RDWR, 0666);
   int rc = flock(pid_file, LOCK_EX | LOCK_NB);
   if (rc) {
      if (EWOULDBLOCK == errno) {
          // another instance is running
          std::cerr << "\033[31mAnother runswift is already running!\033[0m" << std::endl;
          std::cerr << "Things to try to get your runswift running: \n" <<
             "   \033[36m killall runswift \033[0m \n" <<
             "   \033[36m nao restart \033[0m" << std::endl;
          std::cerr << "If that fails, then try: \n" <<
             "   \033[36m lsof -n | grep runswift \033[0m \n" <<
             "The second number is runswift's <pid>, and then: \n" <<
             "   \033[36m kill -9 <pid> \033[0m" << std::endl;
          exit(1);
      }
   }
   else {
      // this is the first instance
      set_cloexec_flag(pid_file);
   }
#endif

   po::variables_map vm;
   try {
      po::options_description generic("Generic options");
      generic.add_options()
         ("help,h", "produce help message")
         ("version,v", "print version string");

      po::options_description cmdline_options =
         store_and_notify(argc, argv, vm, &generic);

      if (vm.count("help")) {
         cout << cmdline_options << endl;
         return 1;
      }

      if (vm.count("version")) {
         cout << "rUNSWift Nao soccer player " << git_version << endl;
         return 1;
      }

      cout << "rUNSWift V." << git_version << endl;

      options_print(vm);
   } catch (program_options::error& e) {
      cerr << "Error when parsing command line arguments: " << e.what() << endl;
      return 1;
   } catch (std::exception& e) {
      cerr << e.what() << endl;
      return 1;
   }

   offNao = false;
   attemptingShutdown = false;
   Thread::name = "main";

   // Generate date string in yyyy-mm-dd-hh-mm-ss format
   time_t s_since_epoch = time(NULL);
   const struct tm *now = localtime(&s_since_epoch);
   char timestr[20];
   strftime(timestr, 20, "%Y-%m-%d-%H-%M-%S", now);

   Logger::init(vm["debug.logpath"].as<string>() + string("/") + string(timestr),
                vm["debug.log"].as<string>(),
                vm["debug.log.motion"].as<bool>());

   Blackboard *blackboard = new Blackboard(vm);

   Camera *topCamera = NULL;
   Camera *botCamera = NULL;
#ifndef SIMULATION
  if (vm["debug.vision"].as<bool>()) {
      llog(INFO) << "Initialising v4 /dev/video0" << std::endl;
      topCamera = new NaoCameraV4(blackboard, "/dev/video0", "camera.top");

      llog(INFO) << "Initialising v4 /dev/video1" << std::endl;
      botCamera = new NaoCameraV4(blackboard, "/dev/video1", "camera.bot",
                                           IO_METHOD_MMAP,
                                           AL::kVGA);

      CombinedCamera::setCameraTop(topCamera);
      CombinedCamera::setCameraBot(botCamera);
   }
#endif

   llog(INFO) << "RUNSWIFT soccer library spinning up!" << endl;
#ifdef SIMULATION
    std::cout << "rUNSWift running in SIMULATION mode. DO NOT RUN THIS ON A NAO.\n";
#endif


   registerSignalHandlers();

   // create thread managers
   ThreadManager perception("Perception", 0); // as fast as the camera can provide as image
#ifndef SIMULATION
   ThreadManager motion("Motion", 0); // as fast as possible, waits on agent semaphore
#else
   ThreadManager motion("Motion-Sim", 0); // 'Motion' is scheduled differently which causes bugs in simulator mode - rename it to avoid this.
   ThreadManager simulation("Simulation", 0);
#endif
   ThreadManager gameController("GameController", 0); // as fast as possible, waits on udp read
   ThreadManager offnaoTransmitter("OffnaoTransmitter", 50000); // 20fps limit
   ThreadManager teamTransmitter("TeamTransmitter", 334000); // 3fps limit
   ThreadManager teamReceiver("TeamReceiver", 100000); // 10fps limit (Congested WiFi: Higher than TeamTransmitter)
   //ThreadManager remoteControlReceiver("RemoteControlReceiver", 200000); // 5 fps limit for remote-control updates

   // start threads
   if (vm["debug.perception"].as<bool>()) {
      perception.run<PerceptionThread>(blackboard);
      llog(INFO) << "Perception is running" << endl;
   }
   if (vm["debug.motion"].as<bool>()) {
      motion.run<MotionAdapter>(blackboard);
      llog(INFO) << "Motion is running" << endl;
   }

#ifdef SIMULATION
   simulation.run<SimulationThread>(blackboard);
   llog(INFO) << "Simulation is running" << endl;
#endif

   if (vm["debug.gamecontroller"].as<bool>()) {
      gameController.run<GameController>(blackboard);
      llog(INFO) << "GameController is running" << endl;
   }
   if (vm["debug.offnaotransmitter"].as<bool>()) {
      offnaoTransmitter.run<OffNaoTransmitter>(blackboard);
      llog(INFO) << "Off-Nao Transmitter is running" << endl;
   }
   if (vm["debug.naotransmitter"].as<bool>()) {
      teamTransmitter.run<TeamTransmitter>(blackboard);
      llog(INFO) << "Nao Transmitter is running" << endl;
   }
   if (vm["debug.naoreceiver"].as<bool>()) {
      teamReceiver.run<TeamReceiver>(blackboard);
      llog(INFO) << "Team Receiver is running" << endl;
   }
//   if (vm["debug.remotecontrol"].as<bool>()) {
//      //pthread_create(&remotecontrol, NULL, &safelyRun<RemoteControlReceiver>,
//      //               NULL);
//      remoteControlReceiver.run<RemoteControlReceiver>(blackboard);
//      llog(INFO) << "Remote Control Receiver is running" << endl;
//   }

   if (vm["debug.shutdowntime"].as<int>()) {
      pthread_t timer;
      pthread_create(&timer, NULL, &shutdownTimer,
                     (void*)vm["debug.shutdowntime"].as<int>());
      llog(INFO) << "Timer is running" << endl;
   }

   teamReceiver.join();
   teamTransmitter.join();
   offnaoTransmitter.join();
   gameController.join();
#ifdef SIMULATION
   simulation.join();
#endif
   motion.join();
   perception.join();

   delete blackboard;

   delete topCamera;
   delete botCamera;

   return 0;
}

