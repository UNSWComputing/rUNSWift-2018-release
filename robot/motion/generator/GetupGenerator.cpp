#include <fstream>
#include <limits>
#include <cctype>
#include <unistd.h>
#include <limits.h>
#include <sstream>
#include "motion/generator/GetupGenerator.hpp"
#include "utils/Logger.hpp"
#include "utils/angles.hpp"
#include "utils/speech.hpp"

using namespace std;
using boost::program_options::variables_map;

GetupGenerator::GetupGenerator(std::string falldirection) : fall_direction(falldirection) {
   max_iter = 0;
   fallen = false;
   num_times_fallen = 0;
   current_time = NOT_RUNNING;
};

GetupGenerator::~GetupGenerator() {
   llog(INFO) << "GetupGenerator destroyed" << std::endl;
};

bool GetupGenerator::isActive() {
   return current_time != NOT_RUNNING;
};

void GetupGenerator::reset() {
   current_time = 0;
}

void GetupGenerator::stop(){
}

JointValues GetupGenerator::makeJoints(ActionCommand::All* request,
                                        Odometry* odometry,
                                        const SensorValues &sensors,
                                        BodyModel &bodyModel,
                                        float ballX,
                                        float ballY) {

   float ang[2] = {RAD2DEG(sensors.sensors[Sensors::InertialSensor_AngleX]),
                   RAD2DEG(sensors.sensors[Sensors::InertialSensor_AngleY])};
   JointValues j;
   if (current_time == NOT_RUNNING) {
      //current_time = 0;
      active = request->body;
      j = joints[joints.size() - 1];
      num_times_fallen = 0;
   } else {
      JointValues newJoints = sensors.joints;
      for (int i = 0; i < Joints::NUMBER_OF_JOINTS; i++) {
         newJoints.stiffnesses[i] = 1.0f;
      }
      if (current_time == 0) interpolate(newJoints);
      //Run Check to see if we have fallen
      if (ang[1] < -FALLEN_ANG) {
         if (current_time > 4 && !fallen) {
            fallen = true;
            num_times_fallen ++;
            fall_direction = "BACK";

            reset();
            joints.clear();
            readOptions(configuration);
         }
      } else if (ang[1] > FALLEN_ANG) {
         if (current_time > 4 && !fallen) {
            fallen= true;
            num_times_fallen ++;
            fall_direction = "FRONT";

            reset();
            joints.clear();
            readOptions(configuration);
         }
      } else {
         fallen = false;
      }
      j = joints[current_time++];
      if (current_time == (signed int)joints.size())  // if we just did last action
         current_time = NOT_RUNNING;
   }

   // If we have fallen more than 3 times in a row, then
   // limp joints and lie there, waiting for ref
   // Say something to indicate this.
    if (num_times_fallen >= CONSECUTIVE_FALLS_ALLOWED) {
        //SAY("I fell over. Please help me.");
        //current_time = NOT_RUNNING;
        //for (uint8_t i = 0; i < Joints::NUMBER_OF_JOINTS; ++i)
        //    j.stiffnesses[i] = -1.0f;
    }

   return j;
};

void GetupGenerator::interpolate(JointValues newJoint, int duration) {
   if (joints.empty()) {
      max_iter = duration / 10;
      // Reserve space for the interpolation when the generator
      // first called
      for (int i = 0; i < max_iter; i++) {
         joints.push_back(newJoint);
      }
      joints.push_back(newJoint);
   } else {
      int inTime = 0;
      float offset[Joints::NUMBER_OF_JOINTS];

      if (duration != 0) {
         inTime = duration / 10;
         JointValues currentJoint = joints.back();

         // Calculate the difference between new joint and the previous joint
         for (int i = 0; i < Joints::NUMBER_OF_JOINTS; i++) {
            offset[i] = (newJoint.angles[i] - currentJoint.angles[i]) / inTime;
         }

         for (int i = 0; i < inTime; i++) {
            JointValues inJoint;
            for (int j = 0; j < Joints::NUMBER_OF_JOINTS; j++) {
               inJoint.angles[j] = joints.back().angles[j] + offset[j];
               inJoint.stiffnesses[j] = newJoint.stiffnesses[j];
            }
            joints.push_back(inJoint);
         }
      } else {

         JointValues firstJoint = joints.at(max_iter);
         // Calculate the difference between the joint at MAX_ITER position
         // with the new joint
         for (int i = 0; i < Joints::NUMBER_OF_JOINTS; i++) {
            offset[i] = (firstJoint.angles[i] - newJoint.angles[i]) / max_iter;
         }

         joints[0] = newJoint;
         for (int i = 1; i < max_iter; i++) {
            for (int j = 0; j < Joints::NUMBER_OF_JOINTS; j++) {
               joints[i].angles[j] = joints[i - 1].angles[j] + offset[j];
               joints[i].stiffnesses[j] = firstJoint.stiffnesses[j];
            }
         }
      }
   }
}

bool GetupGenerator::individualPathExists(std::string individualPath, std::string bodyName) {
   ifstream ifs;
   ifs.open(string(individualPath + "/" + bodyName + "_" + file_name + ".pos").c_str());

   if (ifs == NULL) {
      return false;
   } else {
      llog(INFO) << "GetupGenerator: Individual file path found." << endl;
      ifs.close();
      return true;
   }
}

void GetupGenerator::constructPose(std::string path, std::string individualPath, std::string bodyName) {

   /*
    * Check for individual pos files, if they exist.
    * Getups require the entire body of the nao, so individual pos fles exist
    * to account for multiple joint offsets. If no file exists for them,
    * then simply use the default pos file.
    */


   bool useIndividalPath = individualPathExists(individualPath, bodyName);

   ifstream in;
   if (useIndividalPath) {
      in.open(string(individualPath + "/" + bodyName + "_" + file_name + ".pos").c_str());
   } else {
      in.open(string(path + "/" + file_name + ".pos").c_str());
   }

   llog(INFO) << "GetupGenerator(" << file_name << ") creating" << endl;

   if (!in.is_open()) {
      llog(FATAL) << "GetupGenerator cannot open " << file_name << endl;
   } else {
      int duration = 0;
      float stiffness = 1.0;
      float angles = 0.0;
      while (isspace(in.peek())) {
         in.ignore();
      }
      while (!in.eof()) {
         // Ignore comments, newlines and ensure not eof
         if (in.peek() == '#' || in.peek() == '\n' || in.peek() == EOF) {
            in.ignore(std::numeric_limits<int>::max(), '\n');
            continue;
         }
         JointValues newJoint;
         // Read the angles in the file to create a new JointValue
         for (int i = 0; i < Joints::NUMBER_OF_JOINTS; i++) {
            while (isspace(in.peek())) {
               in.ignore();
            }
            if (in.peek() == '#' || in.peek() == '$' || in.peek() == '\n' || in.peek() == EOF) {
               std::cout << "You're missing a joint value in " << file_name << ".pos" << std::endl;
               exit(1);
            }
            in >> angles;

            // Convert degree to radian because the values in the file
            // are in degree
            newJoint.angles[i] = DEG2RAD(angles);
            newJoint.stiffnesses[i] = 1.0;
         }

         // read in the duration
         while (isspace(in.peek())) {
            in.ignore();
         }
         if (in.peek() == '#' || in.peek() == '$' || in.peek() == '\n' || in.peek() == EOF) {
            std::cout << "You're missing a duration in " << file_name << ".pos" << std::endl;
            exit(1);
         }
         in >> duration;

         // Ignore comments, newlines and ensure not eof
         while (isspace(in.peek())) {
            in.ignore();
         }
         while (in.peek() == '#') {
            in.ignore(std::numeric_limits<int>::max(), '\n');
         }
         while (isspace(in.peek())) {
            in.ignore();
         }

         // Stiffnesses are specified by a line beginning with "$"
         if (in.peek() == '$') {
            in.ignore(std::numeric_limits<int>::max(), '$');
            for (int i = 0; i < Joints::NUMBER_OF_JOINTS; i++) {
               while (isspace(in.peek())) {
                  in.ignore();
               }
               if (in.peek() == '#' || in.peek() == '\n' || in.peek() == EOF) {
                  std::cout << "You're missing a stiffness value in " << file_name << ".pos" << std::endl;
                  exit(1);
               }
               in >> stiffness;
               newJoint.stiffnesses[i] = stiffness;
            }
            // Ignore comments, newlines and ensure not eof
            while (isspace(in.peek())) {
               in.ignore();
            }
            while (in.peek() == '#') {
               in.ignore(std::numeric_limits<int>::max(), '\n');
            }
            while (isspace(in.peek())) {
               in.ignore();
            }
         }
         interpolate(newJoint, duration);
         while (isspace(in.peek())) {
            in.ignore();
         }
      }
      in.close();
   }
   llog(INFO) << "GetupGenerator(" << file_name << ") created" << endl;
}

void GetupGenerator::chooseGetup(){
   bool isFrontGetup = (fall_direction == "FRONT");
   bool isGoalie = (configuration["player.number"].as<int>() == 1);
   std::string getup_speed = configuration["motion.getup_speed"].as<std::string>();

   if (isFrontGetup){
      if (isGoalie){
         if (getup_speed == "FAST"){
            file_name = "getupFrontFast"; // we don't want goalie to do a fast getup ... or do we?
         } else if (getup_speed == "MODERATE"){
            file_name = "getupFront";
         } else if (getup_speed == "SLOW"){
            file_name = "getupFrontSlow";
         }
      } else {
         if (getup_speed == "FAST"){
            file_name = "getupFrontFast";
         } else if (getup_speed == "MODERATE"){
            file_name = "getupFront";
         } else if (getup_speed == "SLOW"){
            file_name = "getupFrontSlow";
         }
      }
   } else {
      if (isGoalie){
         if (getup_speed == "FAST"){
            file_name = "getupBack";
         } else if (getup_speed == "MODERATE"){
            file_name = "getupBack";
         } else if (getup_speed == "SLOW"){
            file_name = "getupBackSlow";
         }
      } else {
         if (getup_speed == "FAST"){
            file_name = "getupBack"; // we don't have a fast back getup
         } else if (getup_speed == "MODERATE"){
            file_name = "getupBack";
         } else if (getup_speed == "SLOW"){
            file_name = "getupBackSlow";
         }
      }
   }
}

void GetupGenerator::readOptions(const boost::program_options::variables_map &config) {
   configuration = config;
   std::string path = config["motion.path"].as<std::string>();
   std::string individualPath = config["motion.individualConfigPath"].as<std::string>();
   std::string bodyName = config["player.bodyName"].as<std::string>();
   chooseGetup();
   constructPose(path, individualPath, bodyName);
}