#include "KinematicsAdapter.hpp"
#include "blackboard/Blackboard.hpp"
#include "utils/Logger.hpp"
#include "perception/vision/camera/Camera.hpp"

using namespace std;

KinematicsAdapter::KinematicsAdapter(Blackboard *bb) : Adapter(bb) {}

KinematicsAdapter::~KinematicsAdapter() {}

void KinematicsAdapter::tick() {
   llog(VERBOSE) << "Kinematics.. ticking away" << endl;
   t.restart();
   llog(VERBOSE) << "Kinematics took: " << t.elapsed_us() << " us" << endl;

   t.restart();
   sonarFilter.update(readFrom(motion, sonarWindow));
   writeTo(kinematics, sonarFiltered, sonarFilter.sonarFiltered);
   llog(VERBOSE) << "Sonar Filter took: " << t.elapsed_us() << " us" << endl;
}

