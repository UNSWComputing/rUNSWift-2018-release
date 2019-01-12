#include <boost/program_options.hpp>
#include <ext/slist>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>

#include "utils/options.hpp"
#include "transmitter/TransmitterDefs.hpp"

namespace po = boost::program_options;
using namespace std;
using namespace __gnu_cxx;

void populate_options(po::options_description &config_file_options) {
   po::options_description game_config("Game options");
   game_config.add_options()
      ("game.type,g", po::value<string>()->default_value("MATCH"),
      "Type of game/challenge (MATCH, DRIBBLE, OPEN, PASSING)");

   po::options_description player_config("Player options");
   player_config.add_options()
      ("player.number,n", po::value<int>()->default_value(2),
      "player number")
      ("player.team,T", po::value<int>()->default_value(19),
      "team number")
      ("player.bodyName", po::value<string>()->default_value(""),
      "player body name");

   po::options_description debug_config("Debugging options");
   debug_config.add_options()
      ("debug.log,l", po::value<string>()->default_value("SILENT"),
      "log level used by llog")
      ("debug.log.motion,m", po::value<bool>()->default_value(false),
      "allow llog from motion")
      ("debug.logpath",
      po::value<string>()->default_value("/var/volatile/runswift"),
      "where to store log files")
      ("debug.shutdowntime", po::value<int>()->default_value(0),
      "shutdown after arg seconds")
      ("debug.gamecontroller,G", po::value<bool>()->default_value(true),
      "enable GameController thread")
      ("debug.motion,M", po::value<bool>()->default_value(true),
      "enable Motion thread")
      ("debug.offnaotransmitter,O", po::value<bool>()->default_value(true),
      "enable OffNaoTransmitter thread")
      ("debug.perception,P", po::value<bool>()->default_value(true),
      "enable Perception thread")
      ("debug.vision,V", po::value<bool>()->default_value(true),
      "enable Vision module")
      ("debug.behaviour,B", po::value<bool>()->default_value(true),
      "enable Behaviour module")
      ("debug.naotransmitter,C", po::value<bool>()->default_value(true),
      "enable Nao Transmitter module")
      ("debug.naoreceiver,R", po::value<bool>()->default_value(true),
       "enable Nao Receiver module")
      ("debug.remotecontrol", po::value<bool>()->default_value(true),
       "enable Remote Control Receiver module")
      ("debug.act_on_whistle", po::value<bool>()->default_value(true),
       "Go to PLAYING state when in SET state and whistle heard")
      ("debug.dump,D", po::value<string>()->default_value(""),
      "Dump blackboard in .bbd format. Empty string disables.")
      ("debug.mask", po::value<int>()->default_value(INITIAL_MASK),
      "Blackboard mask determining what is serialised")
      ("debug.set_initial_pose", po::value<bool>()->default_value(false),
      "set intial pose")
      ("debug.initial_x", po::value<int>()->default_value(0),
      "initial x value (if debug.set_initial_pose == true)")
      ("debug.initial_y", po::value<int>()->default_value(0),
      "initial y value (if debug.set_initial_pose == true)")
      ("debug.initial_theta", po::value<int>()->default_value(0),
      "initial theta value (if debug.set_initial_pose == true)")
      ("debug.set_fallen", po::value<bool>()->default_value(false),
      "set fallen state")
      ("debug.getup_lost", po::value<bool>()->default_value(false),
      "set whether lost after getup");

   po::options_description behaviour_config("Behaviour options");
   behaviour_config.add_options()
      ("behaviour.skill,s",
      po::value<string>()->default_value("GameControllerV2"),
      "The desired top level Python skill class.")
      ("behaviour.path", po::value<string>()->default_value("/home/nao/data/behaviours/"),
      "path containing python behaviours.")
      ("default.body", po::value<string>()->default_value("REF_PICKUP"),
      "default body action type if behaviour isn't running")
      ("default.forward", po::value<int>()->default_value(0),
      "default forward parameter for the walk (mm/step)")
      ("default.left", po::value<int>()->default_value(0),
      "default left parameter for the walk (mm/step)")
      ("default.turn", po::value<float>()->default_value(0.0f),
      "default turn parameter for the walk (deg/step)")
      ("default.power", po::value<float>()->default_value(1.0f),
      "default power parameter for the kick (0.0-1.0)")
      ("default.speed", po::value<float>()->default_value(1.0f),
      "default speed parameter for the walk (0.0-1.0)")
      ("default.bend", po::value<float>()->default_value(15.0f),
      "default bend parameter for the walk (0.0-1.0)")
      ("default.foot", po::value<string>()->default_value("LEFT"),
      "default kick foot (LEFT/RIGHT)")
      ("default.kickDirection", po::value<float>()->default_value(0.0),
      "default kickDirection (degrees)")
      ("default.whichCamera", po::value<string>()->default_value("BEHAVIOUR"),
      "which camera to use")
      ("behaviour.remote_stiffen", po::value<bool>()->default_value(false),
      "Remotely stiffen (and standup) when a READY packet is received from gamecontroller.")
      ("behaviour.use_getups", po::value<bool>()->default_value(true),
      "Getups on/off (TRUE, FALSE)");

   po::options_description motion_config("Motion options");
   motion_config.add_options()
      ("motion.effector,e", po::value<string>()->default_value("Agent"),
      "effector to be used by motion")
      ("motion.touch,t", po::value<string>()->default_value("Agent"),
      "touch to be used by motion")
      ("motion.path",
      po::value<string>()->default_value("/home/nao/data/pos/"),
      "the path of .pos files for ActionGenerator")
      ("motion.individualConfigPath",
      po::value<string>()->default_value("/home/nao/data/pos/individualPoses"),
      "the path of .pos files for ActionGenerator, for individual robots")
      ("motion.v4", po::value<bool>()->default_value(false),
      "whether to use v3 version of getup instead")
      ("walk.f", po::value<float>()->default_value(0.5),
      "frequency of coronal plane rocking (Hz)")
      ("walk.st", po::value<float>()->default_value(1.0),
      "stiffness of leg joints (0.0 - 1.0)")
      ("walk.cs", po::value<bool>()->default_value(true),
      "coronal stabilisation on/off")
      ("walk.r", po::value<float>()->default_value(20.0),
      "amplitude of coronal plane rocking (degrees)")
      ("walk.s", po::value<float>()->default_value(5.0),
      "spread of legs when standing (degrees)")
      ("walk.l", po::value<float>()->default_value(20.0),
      "amplitude of leg lift (degrees)")
      ("walk.fs", po::value<float>()->default_value(5.0),
      "amplitude of forward step (degrees)")
      ("walk.ls", po::value<float>()->default_value(0.0),
      "amplitude of left step (degrees)")
      ("walk.ts", po::value<float>()->default_value(0.0),
      "amplitude of turn step (degrees)")
      ("walk.b", po::value<float>()->default_value(15.0),
      "bend of legs when standing upright (degrees)")
      ("walk.liftHeight", po::value<float>()->default_value(8.0f),
      "leg lift height.")
      ("walk.coronalAmplitude", po::value<float>()->default_value(0.0f),
      "Coronal rock amplitude.")
      ("walk.liftFrac", po::value<float>()->default_value(0.5f),
      "fraction of cycle in which leg lift is performed.")
      ("walk.moveFrac", po::value<float>()->default_value(0.4f),
      "fraction of cycle in which leg move is performed.")
      ("walk.m", po::value<float>()->default_value(0.0),
      "leg lift frequency multiplier (must be multiple of two)")
      ("motion.getup_speed", po::value<string>()->default_value("FAST"),
      "Speed of getups (FAST, MODERATE, SLOW)")
      ("motion.kick_speed", po::value<string>()->default_value("SLOW"),
      "Speed of kicks (FAST, SLOW)")
      ("motion.kick_lean_offset", po::value<float>()->default_value(0.0f),
      "Lean offset of normal kick (degrees)")
      ("motion.fast_kick_lean_offset", po::value<float>()->default_value(0.0f),
      "Lean offset of fast kicks (degrees)");

   po::options_description vision_config("Vision options");
   vision_config.add_options()
      ("vision.camera,c", po::value<string>()->default_value("Nao"),
      "camera to be used by vision")
      ("vision.camera_controls", po::value<string>()->default_value(""),
      "comma separated list of cid:value pairs of controls "
      "(cid offset from V4L2_CID_BASE)")
      ("vision.dumpframes,d", po::value<bool>()->default_value(false),
      "dump frames to disk")
      ("vision.dumprate,r", po::value<int>()->default_value(1000),
      "dump frames every arg milliseconds")
      ("vision.dumpfile,f", po::value<string>()->default_value("dump.yuv"),
      "file to store frames in")
      ("vision.run_colour_calibration", po::value<bool>()->default_value(false),
      "runs a calibration")
      ("vision.load_nnmc", po::value<bool>()->default_value(true),
      "loads /home/nao/data/green_yuv_classifier.nnmc to nnmc_")
      ("vision.save_nnmc", po::value<bool>()->default_value(false),
      "saves our calibration to an nnmc file");

   po::options_description camera_config("Camera options");
   camera_config.add_options()
      ("camera.top.hflip", po::value<int>()->default_value(1),
       "camera top hflip")
      ("camera.top.vflip", po::value<int>()->default_value(1),
       "camera top vflip")
      ("camera.top.brightness", po::value<int>()->default_value(20),
       "camera top brightness")
      ("camera.top.contrast", po::value<int>()->default_value(60),
       "camera top contrast")
      ("camera.top.saturation", po::value<int>()->default_value(210),
       "camera top saturation")
      ("camera.top.hue", po::value<int>()->default_value(0),
       "camera top hue")
      ("camera.top.sharpness", po::value<int>()->default_value(2),
       "camera top sharpness")
      ("camera.top.backlightcompensation", po::value<int>()->default_value(0x00),
       "camera top backlight compensation")
      ("camera.top.exposure", po::value<int>()->default_value(90),
       "camera top exposure")
      ("camera.top.gain", po::value<int>()->default_value(250),
       "camera top gain")
      ("camera.top.whitebalance", po::value<int>()->default_value(2700),
       "camera top whitebalance")
      ("camera.top.exposureauto", po::value<int>()->default_value(1),
       "camera top exposureauto")
      ("camera.top.autowhitebalance", po::value<int>()->default_value(1),
       "camera top autowhitebalance")
      ("camera.top.exposurealgorithm", po::value<int>()->default_value(3),
       "camera top exposurealgorithm")
      ("camera.top.aetargetavgluma", po::value<int>()->default_value(55),
       "camera top ae target avg luma")
      ("camera.top.aetargetavglumadark", po::value<int>()->default_value(27),
       "camera top ae target avg luma")
      ("camera.top.aetargetgain", po::value<int>()->default_value(128),
       "camera top ae target gain")
      ("camera.top.aeminvirtgain", po::value<int>()->default_value(32),
       "camera top ae min virt gain")
      ("camera.top.aemaxvirtgain", po::value<int>()->default_value(256),
       "camera top ae max virt gain")
      ("camera.top.aeminvirtagain", po::value<int>()->default_value(32),
       "camera top ae min virt again")
      ("camera.top.aemaxvirtagain", po::value<int>()->default_value(256),
       "camera top ae max virt again")


      ("camera.bot.hflip", po::value<int>()->default_value(0),
       "camera bot hflip")
      ("camera.bot.vflip", po::value<int>()->default_value(0),
       "camera bot vflip")
      ("camera.bot.brightness", po::value<int>()->default_value(20),
       "camera bot brightness")
      ("camera.bot.contrast", po::value<int>()->default_value(60),
       "camera bot contrast")
      ("camera.bot.saturation", po::value<int>()->default_value(180),
       "camera bot saturation")
      ("camera.bot.hue", po::value<int>()->default_value(0),
       "camera bot hue")
      ("camera.bot.sharpness", po::value<int>()->default_value(2),
       "camera bot sharpness")
      ("camera.bot.backlightcompensation", po::value<int>()->default_value(0x00),
       "camera bot backlight compensation")
      ("camera.bot.exposure", po::value<int>()->default_value(13),
       "camera bot exposure")
      ("camera.bot.gain", po::value<int>()->default_value(250),
       "camera bot gain")
      ("camera.bot.whitebalance", po::value<int>()->default_value(2700),
       "camera bot whitebalance")
      ("camera.bot.exposureauto", po::value<int>()->default_value(1),
       "camera bot exposureauto")
      ("camera.bot.autowhitebalance", po::value<int>()->default_value(1),
       "camera bot autowhitebalance")
      ("camera.bot.exposurealgorithm", po::value<int>()->default_value(3),
       "camera bot exposurealgorithm")
      ("camera.bot.aetargetavgluma", po::value<int>()->default_value(55),
       "camera bot ae target avg luma")
      ("camera.bot.aetargetavglumadark", po::value<int>()->default_value(27),
       "camera bot ae target avg luma dark")
      ("camera.bot.aetargetgain", po::value<int>()->default_value(128),
       "camera bot ae target gain")
      ("camera.bot.aeminvirtgain", po::value<int>()->default_value(32),
       "camera bot ae min virt gain")
      ("camera.bot.aemaxvirtgain", po::value<int>()->default_value(256),
       "camera bot ae max virt gain")
      ("camera.bot.aeminvirtagain", po::value<int>()->default_value(32),
       "camera bot ae min virt again")
      ("camera.bot.aemaxvirtagain", po::value<int>()->default_value(256),
       "camera bot ae max virt again");


   po::options_description kinematics_config("Kinematics options");
   kinematics_config.add_options()
      ("kinematics.isCalibrating", po::value<bool>()->default_value(false),
      "If set to true run kinematics calibration.")
      ("kinematics.bodyPitch", po::value<float>()->default_value(0.0),
      "accounts for imperfections in all motors in the robots body.")
      ("kinematics.cameraYawTop", po::value<float>()->default_value(0.0),
      "difference between real camera Yaw compared to aldebaran specs")
      ("kinematics.cameraRollTop", po::value<float>()->default_value(0.0),
      "difference between real camera Roll compared to aldebaran specs")
      ("kinematics.cameraRollBottom", po::value<float>()->default_value(0.0),
      "difference between real camera Roll compared to aldebaran specs")
      ("kinematics.cameraPitchTop", po::value<float>()->default_value(0.0),
      "difference between real camera Pitch compared to aldebaran specs")
      ("kinematics.cameraYawBottom", po::value<float>()->default_value(0.0),
      "difference between real camera Yaw compared to aldebaran specs")
      ("kinematics.cameraPitchBottom", po::value<float>()->default_value(0.0),
      "difference between real camera Pitch compared to aldebaran specs");

   po::options_description touch_config("Touch options");
   touch_config.add_options()
      ("touch.gyrXOffset", po::value<float>()->default_value(0.0),
      "offset on gyrX.")
      ("touch.gyrYOffset", po::value<float>()->default_value(0.0),
      "offset on gyrY.")
      ("touch.angleXOffset", po::value<float>()->default_value(0.0),
      "offset on angleX")
      ("touch.angleYOffset", po::value<float>()->default_value(0.0),
      "offset on angleY");

   po::options_description gamecontroller_config("GameController options");
   gamecontroller_config.add_options()
      ("gamecontroller.connect", po::value<bool>()->default_value(true),
      "whether the GameController should try to connect")
      ("gamecontroller.state", po::value<string>()->default_value("INITIAL"),
      "game state if gamecontroller not connected, can be: "
      "INITIAL, READY, SET, PLAYING, FINISHED")
      ("gamecontroller.gamephase",
      po::value<string>()->default_value("NORMAL"),
      "secondary game phase if gamecontroller not connected, can be: "
      "NORMAL, PENALTYSHOOT")
      ("gamecontroller.setplay", po::value<string>()->default_value("NONE"),
      "set play if gamecontroller not connected, can be: "
      "NONE, GOAL_FREE_KICK, PUSHING_FREE_KICK")
      ("gamecontroller.ourcolour", po::value<string>()->default_value("blue"),
      "our team colour if gamecontroller not connected")
      ("gamecontroller.opponentteam", po::value<int>()->default_value(1),
      "opponent team number if gamecontroller not connected")
      ("gamecontroller.ourscore", po::value<int>()->default_value(0),
      "our team's score if gamecontroller not connected")
      ("gamecontroller.opponentscore", po::value<int>()->default_value(0),
      "opponent team's score if gamecontroller not connected")
      ("gamecontroller.firsthalf", po::value<bool>()->default_value(true),
      "whether we're in the first half if gamecontroller not connected")
      ("gamecontroller.kickingteam", po::value<string>()->default_value("red"),
      "which team kicks off if gamecontroller not connected")
      ("gamecontroller.secsremaining", po::value<int>()->default_value(600),
      "seconds left in the half if gamecontroller not connected");

   po::options_description network_config("Networking options");
   network_config.add_options()
      ("network.ssid", po::value<string>()->default_value
         ("runswift"), "WiFi network name (SSID) to connect to")
      ("network.transmitter_address", po::value<string>()->default_value
         ("192.168.255.255"), "address to broadcast to")
      ("network.transmitter_base_port", po::value<int>()->default_value(10000),
      "port to which we add team number, and then broadcast on");

   po::options_description remote_control_config("Remote Control options");
   remote_control_config.add_options()
      ("remotecontrol.port", po::value<int>()->default_value(4000),
       "port to receive on");

   config_file_options.add(game_config).add(player_config)
   .add(gamecontroller_config).add(debug_config).add(behaviour_config)
   .add(motion_config).add(vision_config).add(camera_config).add(kinematics_config)
   .add(network_config).add(touch_config);
}

po::options_description store_and_notify(int argc, char **argv,
                                         po::variables_map &vm,
                                         po::options_description* generic) {
   vector<string> vs(argv + 1, argv + argc);
   // for(int arg = 1; arg < argc; ++arg)
   //    vs.push_back(argv[arg]);
   return store_and_notify(vs, vm, generic);
}


po::options_description store_and_notify(vector<string> argv,
                                         po::variables_map &vm,
                                         po::options_description* generic) {
   po::options_description config_file_options;
   populate_options(config_file_options);

   po::options_description cmdline_options;
   if (generic)
      cmdline_options.add(*generic);
   cmdline_options.add(config_file_options);

   /** Config hierarchy:
    *  - first,  command line arguments
    *  - second, /home/nao/data/configs/$hostname.cfg
    *            if a hostname config is not found, then it will use __default__.cfg
    *  - third, /home/nap/data/configs/body/$hostname.cfg (for body specific configs)
    *  - fourth,  /home/nao/data/runswift.cfg
    *  - fifth, `pwd`/runswift.cfg
    **/
   // arguments from this call
   store(po::command_line_parser(argv).options(cmdline_options).run(), vm);

   // arguments from previous calls
   static slist<vector<string> > argvs;
   for (slist<vector<string> >::const_iterator argv_ci = argvs.begin();
        argv_ci != argvs.end(); ++argv_ci)
      store(po::command_line_parser(*argv_ci).options(cmdline_options).run(),
            vm);

   ifstream ifs;
   string hostname;
   string bodyname;

   ifs.open("/etc/hostname");
   ifs >> hostname;
   ifs.close();

   ifs.open(string("/home/nao/data/configs/" + hostname + ".cfg").c_str());
   bool noHostCofig = ifs.fail();
   store(parse_config_file(ifs, config_file_options), vm);
   ifs.close();

   if (noHostCofig) {
      ifs.open("image/home/nao/data/configs/__default__.cfg");
      store(parse_config_file(ifs, config_file_options), vm);
      ifs.close();
   }

   bodyname = vm["player.bodyName"].as<string>();
   ifs.open(string("/home/nao/data/configs/body/" + bodyname + ".cfg").c_str());
   store(parse_config_file(ifs, config_file_options), vm);
   ifs.close();

   ifs.open("/home/nao/data/runswift.cfg");
   store(parse_config_file(ifs, config_file_options), vm);
   ifs.close();

   ifs.open("runswift.cfg");
   store(parse_config_file(ifs, config_file_options), vm);
   ifs.close();


   /** Doesn't do anything right now, but will 'notify' any variables
    * we try to set via program_options */
   po::notify(vm);

   // assuming all options were valid and no exception was thrown, save argv for
   // repeated use
   argvs.push_front(argv);

   return cmdline_options;
}

/** A little struct used to help output program options */
struct type_info_compare {
   bool operator()(const type_info* lhs,
                   const type_info* rhs) const {
      return lhs->before(*rhs);
   }
};

/** Wrapper for outputting a boost::program_options::varaible_value */
template <typename T>
struct output_value {
   void operator()(const po::variable_value& v) const {
      cout << v.as< T >() << endl;
   }
};

void options_print(boost::program_options::variables_map &vm) {
   /** Populate this map with actions that will correctly
    * print each type of program option */
   map<const type_info*,
       boost::function<void(const po::variable_value&)>,
       type_info_compare> action_map;
   action_map[&typeid(string)] = output_value<string>();
   action_map[&typeid(int)] = output_value<int>();
   action_map[&typeid(bool)] = output_value<bool>();
   action_map[&typeid(float)] = output_value<float>();

   po::variables_map::iterator it;
   cout << endl;
   for (it = vm.begin(); it != vm.end(); ++it) {
      // cout << setw(20) << left << it->first << ": ";
      const po::variable_value& v = it->second;
      if (!v.empty()) {
         cout << setw(20) << left << it->first << ": ";
         action_map[&v.value().type()](v);
      }
   }
}
