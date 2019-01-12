#include "variableView.hpp"
#include <QTreeWidgetItem>
#include <QStringList>
#include <QString>
#include <string>
#include <utility>
#include <algorithm>
#include "types/ActionCommand.hpp"
#include "blackboard/Blackboard.hpp"
#include "perception/vision/Vision.hpp"

#include <boost/lexical_cast.hpp>


using namespace std;

VariableView::VariableView() {
   perceptionHeading = new QTreeWidgetItem(this, QStringList(QString("Perception")), 1);
   perceptionHeading->setExpanded(true);
   perceptionAverageFPS  = new QTreeWidgetItem(perceptionHeading,
         QStringList(QString("Framerate: ")), 1);
   perceptionKinematicsTime = new QTreeWidgetItem(perceptionHeading,
         QStringList(QString("Kinematics time: ")), 1);
   perceptionVisionTime = new QTreeWidgetItem(perceptionHeading,
         QStringList(QString("Vision time: ")), 1);
   perceptionLocalisationTime = new QTreeWidgetItem(perceptionHeading,
         QStringList(QString("Localisation time: ")), 1);
   perceptionBehaviourTime = new QTreeWidgetItem(perceptionHeading,
         QStringList(QString("Behaviour time: ")), 1);
   perceptionTotalTime = new QTreeWidgetItem(perceptionHeading,
         QStringList(QString("Total time: ")), 1);

   visionHeading = new QTreeWidgetItem(this, QStringList(QString("Vision")), 1);
   visionHeading->setExpanded(true);

   visionTimestamp = new QTreeWidgetItem(visionHeading,
         QStringList(QString("Vision Timestamp = ")), 1);
   visionDxdy = new QTreeWidgetItem(visionHeading,
         QStringList(QString("Vision DxDy = ")), 1);
   visionFrames = new QTreeWidgetItem(visionHeading,
         QStringList(QString("Surf Missed Frames = ")), 1);
   visionGoalProb = new QTreeWidgetItem(visionHeading,
         QStringList(QString("Away Goal Prob = ")), 1);
   visionHomeMapSize = new QTreeWidgetItem(visionHeading,
         QStringList(QString("Home Map Size = ")), 1);
   visionAwayMapSize = new QTreeWidgetItem(visionHeading,
         QStringList(QString("Away Map Size = ")), 1);
   visionGoal = new QTreeWidgetItem(visionHeading,
         QStringList(QString("Goal Area = ")), 1);
   visionNumBalls = new QTreeWidgetItem(visionHeading,
         QStringList(QString("# Balls = ")), 1);
   visionBallPos = new QTreeWidgetItem(visionHeading,
         QStringList(QString("Ball Pos = ")), 1);
   visionBallPosRobotRelative  = new QTreeWidgetItem(visionHeading,
         QStringList(QString("Robot relative Ball Pos = ")), 1);
   visionPostType1 = new QTreeWidgetItem(visionHeading,
         QStringList(QString("Post Type 1 = ")), 1);
   visionPost1 = new QTreeWidgetItem(visionHeading,
         QStringList(QString("Post 1 @ ")), 1);
   visionPostInfo1 = new QTreeWidgetItem(visionHeading,
         QStringList(QString("Post 1 - ")), 1);
   visionPostType2 = new QTreeWidgetItem(visionHeading,
         QStringList(QString("Post Type 2 = ")), 1);
   visionPost2 = new QTreeWidgetItem(visionHeading,
         QStringList(QString("Post 2 @ ")), 1);
   visionPostInfo2 = new QTreeWidgetItem(visionHeading,
         QStringList(QString("Post 2 - ")), 1);
   visionNumFeet = new QTreeWidgetItem(visionHeading,
         QStringList(QString("NumFeet = ")), 1);
   visionFoot1 = new QTreeWidgetItem(visionHeading,
         QStringList(QString("Foot1 = ")), 1);
   visionFoot2 = new QTreeWidgetItem(visionHeading,
         QStringList(QString("Foot2 = ")), 1);

   lastSecond = new QTreeWidgetItem(this,
         QStringList(QString("In the last second")), 1);
   lastSecond->setExpanded(true);
   visionframesProcessed = new QTreeWidgetItem(lastSecond,
         QStringList(QString("Frames processed: ")), 1);

   cornersHeading = new QTreeWidgetItem(lastSecond,
         QStringList(QString("Corners: ")), 1);
   cornersHeading->setExpanded(true);

   cornersObserved = new QTreeWidgetItem(cornersHeading,
         QStringList(QString("observed: ")), 1);
   cornersDistance = new QTreeWidgetItem(cornersHeading,
         QStringList(QString("distance: ")), 1);

   tJunctionsHeading = new QTreeWidgetItem(lastSecond,
         QStringList(QString("TJunctions: ")), 1);
   tJunctionsHeading->setExpanded(true);
   tJunctionsObserved = new QTreeWidgetItem(tJunctionsHeading,
         QStringList(QString("observed: ")), 1);
   tJunctionsDistance = new QTreeWidgetItem(tJunctionsHeading,
         QStringList(QString("distance: ")), 1);

   postsHeading = new QTreeWidgetItem(lastSecond,
         QStringList(QString("Posts: ")), 1);
   postsHeading->setExpanded(true);
   postsObserved = new QTreeWidgetItem(postsHeading,
         QStringList(QString("observed: ")), 1);
   postskDistance = new QTreeWidgetItem(postsHeading,
         QStringList(QString("kDistance: ")), 1);
   postswDistance = new QTreeWidgetItem(postsHeading,
         QStringList(QString("wDistance: ")), 1);

   ballsHeading = new QTreeWidgetItem(lastSecond,
         QStringList(QString("Balls: ")), 1);
   ballsHeading->setExpanded(true);
   ballsObserved = new QTreeWidgetItem(ballsHeading,
         QStringList(QString("observed: ")), 1);
   ballsDistance = new QTreeWidgetItem(ballsHeading,
         QStringList(QString("distance: ")), 1);

   linesHeading = new QTreeWidgetItem(lastSecond,
         QStringList(QString("Lines: ")), 1);
   linesHeading->setExpanded(true);
   linesObserved = new QTreeWidgetItem(linesHeading,
         QStringList(QString("observed: ")), 1);

   penaltySpotsHeading = new QTreeWidgetItem(lastSecond,
         QStringList(QString("Penalty Spots: ")), 1);
   penaltySpotsHeading->setExpanded(true);
   penaltySpotsObserved = new QTreeWidgetItem(penaltySpotsHeading,
         QStringList(QString("observed: ")), 1);

   centreCirclesHeading = new QTreeWidgetItem(lastSecond,
         QStringList(QString("Centre Circles: ")), 1);
   centreCirclesHeading->setExpanded(true);
   centreCirclesObserved = new QTreeWidgetItem(centreCirclesHeading,
         QStringList(QString("observed: ")), 1);

   fieldLinePointsHeading = new QTreeWidgetItem(lastSecond,
         QStringList(QString("Field Line Points: ")), 1);
   fieldLinePointsHeading->setExpanded(true);
   fieldLinePointsObserved = new QTreeWidgetItem(fieldLinePointsHeading,
         QStringList(QString("observed: ")), 1);

   robotsHeading = new QTreeWidgetItem(lastSecond,
         QStringList(QString("Robots: ")), 1);
   robotsHeading->setExpanded(true);
   robotsObserved = new QTreeWidgetItem(robotsHeading,
         QStringList(QString("observed: ")), 1);

   this->setHeaderLabel(QString("State variables"));

   localisationHeading = new QTreeWidgetItem(this,
         QStringList(QString("Localisation")), 1);
   localisationHeading->setExpanded(true);
   localisationRobotPosx = new QTreeWidgetItem(localisationHeading,
         QStringList(QString("Robot x = ")), 1);
   localisationRobotPosy = new QTreeWidgetItem(localisationHeading,
         QStringList(QString("Robot y = ")), 1);
   localisationRobotHeading = new QTreeWidgetItem(localisationHeading,
         QStringList(QString("Robot heading = ")), 1);
   localisationRobotTopWeight = new QTreeWidgetItem(localisationHeading,
         QStringList(QString("Robot top weight = ")), 1);
   localisationRobotHypoth = new QTreeWidgetItem(localisationHeading,
         QStringList(QString("Robot num hypoth = ")), 1);
   localisationBallLost = new QTreeWidgetItem(localisationHeading,
         QStringList(QString("Ball lost = ")), 1);
   localisationBallSeen = new QTreeWidgetItem(localisationHeading,
         QStringList(QString("Ball seen = ")), 1);
   localisationBallx = new QTreeWidgetItem(localisationHeading,
         QStringList(QString("Ball x = ")), 1);
   localisationBally = new QTreeWidgetItem(localisationHeading,
         QStringList(QString("Ball y = ")), 1);
    localisationBallVelx = new QTreeWidgetItem(localisationHeading,
         QStringList(QString("Ball dx = ")), 1);
   localisationBallVely = new QTreeWidgetItem(localisationHeading,
         QStringList(QString("Ball dy = ")), 1);

   motionHeading = new QTreeWidgetItem(this, QStringList(QString("Motion")), 1);
   motionHeading->setExpanded(true);

   motionHeadYaw = new QTreeWidgetItem(motionHeading,
         QStringList(QString("HeadYaw = ")), 1);
   kinematicsHeadYaw = new QTreeWidgetItem(motionHeading,
         QStringList(QString("HeadYawK = ")), 1);

   behaviourHeading = new QTreeWidgetItem(this,
                                          QStringList(QString("Behaviour")), 1);
   behaviourBodyRequest = new QTreeWidgetItem(behaviourHeading,
                     QStringList(QString("request.body.actionType = ")), 1);
   behaviourHierarchy = new QTreeWidgetItem(behaviourHeading,
                     QStringList(QString("Behaviour Hierarchy:")), 1);
   behaviourHeading->setExpanded(true);

   gameControllerHeading = new QTreeWidgetItem(this,
                                          QStringList(QString("GameController")), 1);
   gameControllerTeam = new QTreeWidgetItem(gameControllerHeading,
                     QStringList(QString("gameController.team_red = ")), 1);
   gameControllerHeading->setExpanded(true);

   this->addTopLevelItem(visionHeading);
   this->addTopLevelItem(localisationHeading);
   this->addTopLevelItem(motionHeading);
   this->addTopLevelItem(behaviourHeading);
}

void VariableView::redraw(NaoData *naoData) {

   Frame frame = naoData->getCurrentFrame();
   Blackboard *blackboard = (frame.blackboard);
   if(!blackboard) return;

   // struct RobotPos wrapperPos[4];
   // readArray(localisation, robot, wrapperPos);
   AbsCoord pos = readFrom(localisation, robotPos);

   uint32_t ballLostCount = readFrom(localisation, ballLostCount);
   uint32_t ballSeenCount = readFrom(localisation, ballSeenCount);
   AbsCoord ballPos = readFrom(localisation, ballPos);
   AbsCoord ballVelRRC = readFrom(localisation, ballVelRRC);
   std::vector < AbsCoord > allrobotPos = readFrom(localisation, allrobotPos);

   stringstream sposx;
   sposx << "Robot x: " << pos.x() << " stdvar: (" << std::sqrt(pos.var(0,0)) << ")" << endl;

   stringstream sposy;
   sposy << "Robot y: " << pos.y() << " stdvar: (" << std::sqrt(pos.var(1,1)) << ")" << endl;

   stringstream sheading;
   sheading << "Robot heading: " << pos.theta() << " (" << RAD2DEG(pos.theta()) << " deg) stdvar: (" << std::sqrt(pos.var(2, 2)) << endl;

   stringstream sposNumHypoth;
   sposNumHypoth << "Number of hypotheses: " << allrobotPos.size() << endl;

   stringstream sposWeight;
   sposWeight << "Weight of top Gaussian: " << pos.weight << endl;

   stringstream sballLost;
   sballLost << "Ball lost count: " << ballLostCount << endl;

   stringstream sballSeen;
   sballSeen << "Ball seen count: " << ballSeenCount << endl;

   stringstream sballposx;
   sballposx << "Ball x: " << ballPos.x() << " stdvar: (" << std::sqrt(ballPos.var(0,0)) << ")" << endl;

   stringstream sballposy;
   sballposy << "Ball y: " << ballPos.y() << " stdvar: (" << std::sqrt(ballPos.var(1,1)) << ")" << endl;

   stringstream sballvelx;
   sballposx << "Ball vel x: " << ballVelRRC.x() << " stdvar: (" << std::sqrt(ballVelRRC.var(0,0)) << ")" << endl;

   stringstream sballvely;
   sballposy << "Ball vel y: " << ballVelRRC.y() << " stdvar: (" << std::sqrt(ballVelRRC.var(1,1)) << ")" << endl;


   localisationRobotPosx->setText(0, sposx.str().c_str());
   localisationRobotPosy->setText(0, sposy.str().c_str());
   localisationRobotHeading->setText(0, sheading.str().c_str());
   localisationRobotTopWeight->setText(0, sposWeight.str().c_str());
   localisationRobotHypoth->setText(0, sposNumHypoth.str().c_str());
   localisationBallLost->setText(0, sballLost.str().c_str());
   localisationBallSeen->setText(0, sballSeen.str().c_str());
   localisationBallx->setText(0, sballposx.str().c_str());
   localisationBally->setText(0, sballposy.str().c_str());
   localisationBallVelx->setText(0, sballvelx.str().c_str());
   localisationBallVely->setText(0, sballvely.str().c_str());

   updateVision(naoData);
   updateBehaviour(naoData);
   stringstream steam;
   steam << "gamecontroller.team_red = " << readFrom(gameController, team_red) << endl;
   gameControllerTeam->setText(0, steam.str().c_str());
}

template <class T>
const char* VariableView::createSufPref(string pref, T t, string suff) {
   stringstream s;
   s << pref << t << suff;
   return s.str().c_str();
}

void VariableView::updateBehaviour(NaoData *naoData) {
   Blackboard *blackboard = (naoData->getCurrentFrame().blackboard);
   if (!blackboard) return;

   string actionName;
   switch (readFrom(motion, active).body.actionType) {
   case ActionCommand::Body::NONE: actionName = "NONE"; break;
   case ActionCommand::Body::STAND: actionName = "STAND"; break;
   case ActionCommand::Body::WALK: actionName = "WALK"; break;
   case ActionCommand::Body::DRIBBLE: actionName = "DRIBBLE"; break;
   case ActionCommand::Body::TURN_DRIBBLE: actionName = "TURN_DRIBBLE"; break;
   case ActionCommand::Body::GETUP_FRONT: actionName = "GETUP_FRONT"; break;
   case ActionCommand::Body::GETUP_BACK: actionName = "GETUP_BACK"; break;
   case ActionCommand::Body::TIP_OVER: actionName = "TIP_OVER"; break;
   case ActionCommand::Body::KICK: actionName = "KICK"; break;
   case ActionCommand::Body::INITIAL: actionName = "INITIAL"; break;
   case ActionCommand::Body::DEAD: actionName = "DEAD"; break;
   case ActionCommand::Body::REF_PICKUP: actionName = "REF_PICKUP"; break;
   case ActionCommand::Body::GOALIE_SIT: actionName = "GOALIE_SIT"; break;
   case ActionCommand::Body::GOALIE_DIVE_RIGHT: actionName = "GOALIE_DIVE_RIGHT"; break;
   case ActionCommand::Body::GOALIE_DIVE_LEFT: actionName = "GOALIE_DIVE_LEFT"; break;
   case ActionCommand::Body::GOALIE_CENTRE: actionName = "GOALIE_CENTRE"; break;
   case ActionCommand::Body::GOALIE_UNCENTRE: actionName = "GOALIE_UNCENTRE"; break;
   case ActionCommand::Body::GOALIE_INITIAL: actionName = "GOALIE_INITIAL"; break;
   case ActionCommand::Body::GOALIE_AFTERSIT_INITIAL: actionName = "GOALIE_AFTERSIT_INITIAL"; break;

   default: actionName = "Other"; break;
   }
   behaviourBodyRequest->setText(0, createSufPref("request.body.actionType = ",
                                                  actionName, ""));

   string hierarchy = readFrom(behaviour,request)[0].behaviourHierarchy;
   std::replace(hierarchy.begin(), hierarchy.end(), '.', '\n');
   behaviourHierarchy->setText(0, createSufPref("Behaviour Hierarchy: \n",
                                                  hierarchy, ""));
}

void VariableView::updateVision(NaoData *naoData) {

   Blackboard *blackboard = (naoData->getCurrentFrame().blackboard);
   if (!blackboard) return;

   std::vector<FootInfo>        feet          = readFrom (vision, feet_boxes);
   std::vector<BallInfo>         balls         = readFrom (vision, balls);
   BallHint                      ballHint      = readFrom (vision, ballHint);
   std::vector<PostInfo>         posts         = readFrom (vision, posts);
   std::vector<RobotInfo>        robots        = readFrom (vision, robots);
   std::vector<FieldBoundaryInfo>    fieldBoundaries    = readFrom (vision, fieldBoundaries);
   std::vector<FieldFeatureInfo> fieldFeatures = readFrom (vision, fieldFeatures);
   std::pair<int, int>           dxdy          = readFrom (vision, dxdy);
   LastSecondInfo                lastSecondInfo    = readFrom (vision, lastSecond);

   float headYaw  = readFrom(motion, sensors).joints.angles[Joints::HeadYaw];
   float headYawK  = readFrom(kinematics, sensorsLagged).joints.angles[Joints::HeadYaw];
   uint32_t total = readFrom(perception, total);

   times.push_back(total);
   if (times.size() > 10) times.pop_front();
   int sum = 0;
   for (unsigned int i = 0; i < times.size(); i++) sum += times[i];
   float fps = 1000000.0/total;


   perceptionAverageFPS->setText(0, createSufPref("Framerate: ", fps, " fps"));
   perceptionKinematicsTime->setText(0, createSufPref("Kinematics time: ", readFrom(perception, kinematics), ""));
   perceptionVisionTime->setText(0, createSufPref("Vision time: ", readFrom(perception, vision), ""));
   perceptionLocalisationTime->setText(0, createSufPref("Localisation time: ", readFrom(perception, localisation), ""));
   perceptionBehaviourTime->setText(0, createSufPref("Behaviour time: ", readFrom(perception, behaviour), ""));
   perceptionTotalTime->setText(0, createSufPref("Total time: ", total, ""));

   visionTimestamp->setText(0, createSufPref("Timestamp: ", readFrom(vision, timestamp), ""));
   visionFrames->setText(0, createSufPref("Surf missed frames: ", readFrom(vision, missedFrames), ""));
   visionGoalProb->setText(0, createSufPref("Away Goal Prob: ", readFrom(vision, awayGoalProb), ""));
   visionHomeMapSize->setText(0, createSufPref("Home Map Size: ", readFrom(vision, homeMapSize), ""));
   visionAwayMapSize->setText(0, createSufPref("Away Map Size: ", readFrom(vision, awayMapSize), ""));
   visionGoal->setText(0, createSufPref("Goal Area: ", PostInfo::TypeName[readFrom(vision, goalArea)], ""));

   stringstream sdxdy;
   sdxdy << "Vision DxDy: (" << dxdy.first << "," << dxdy.second << ")";
   visionDxdy->setText(0, sdxdy.str().c_str());


   stringstream sBallPos;

   sBallPos << "BallHint: " << BallHint::TypeName[ballHint.type] << "\n";

   int numBalls = balls.size ();
   if (numBalls == 0) {
   } else {
      Point ballLocation = balls[0].imageCoords;
      sBallPos << "Ball is at (" << ballLocation.x() <<
         ", " << ballLocation.y () << ")";
   }
   visionBallPos->setText(0, sBallPos.str().c_str());

   stringstream s;

   int numBoundaries = fieldBoundaries.size ();
   RANSACLine boundaries[MAX_FIELD_BOUNDARIES];

   for (int i = 0; i < numBoundaries; ++ i) {
      boundaries[i] = fieldBoundaries[i].rrBoundary;
   }


   s << "numBoundaries " << numBoundaries << endl;
   for (int i = 0; i < numBoundaries; i++) {
      s << "Edge 1: " << boundaries[i].t1 << "x + " << boundaries[i].t2 <<
         "y + " << boundaries[i].t3 << endl;
   }

   s.str ("");
   s << "Post Type 1 = " << PostInfo::TypeName[PostInfo::pNone];
   visionPostType1->setText (0, s.str ().c_str ());

   s.str ("");
   s << "Post Type 2 = " << PostInfo::TypeName[PostInfo::pNone];
   visionPostType2->setText (0, s.str ().c_str ());

   visionPost1->setText (0, "Post 1 @ ");
   visionPost2->setText (0, "Post 2 @ ");
   visionPostInfo1->setText (0, "P1 - ");
   visionPostInfo2->setText (0, "P2 - ");


   if (posts.size () > 0) {
      PostInfo p = posts[0];
      RRCoord rr = posts[0].rr;

      s.str ("");
      s << "Post Type 1 = " << PostInfo::TypeName[posts[0].type];
      visionPostType1->setText (0, s.str ().c_str ());

      s.str ("");
      s << "Post 1 @ "
        << rr.distance() << ", " << RAD2DEG(rr.heading()) << ", " << rr.heading()
        << ", (" << rr.var(0,0) << ", " << rr.var(1,1) << ")";
      visionPost1->setText (0, s.str ().c_str ());

      s.str ("");
      s << "P1 - Trust = " << p.trustDistance;
      s << ", dir = " << PostInfo::DirName[p.dir];
      visionPostInfo1->setText (0, s.str ().c_str ());
   }

   if (posts.size () > 1) {
      PostInfo p = posts[1];
      RRCoord rr = posts[1].rr;

      s.str ("");
      s << "Post Type 2 = " << PostInfo::TypeName[posts[1].type];
      visionPostType2->setText (0, s.str ().c_str ());

      s.str ("");
      s << "Post 2 @ "
        << rr.distance() << ", " << RAD2DEG(rr.heading()) << ", " << rr.heading()
        << ", (" << rr.var(1,1) << ", " << rr.var(2,2) << ")";
      visionPost2->setText (0, s.str ().c_str ());

      s.str ("");
      s << "P2 - Trust = " << p.trustDistance;
      s << ", dir = " << PostInfo::DirName[p.dir];
      visionPostInfo2->setText (0, s.str ().c_str ());

   }

   stringstream sBallPosRelative;
   if(numBalls > 0) {
      sBallPosRelative << "Ball @ " << balls[0].rr.distance() << ", " <<
         RAD2DEG(balls[0].rr.heading()) << ", " << balls[0].rr.heading();
   }
   visionBallPosRobotRelative->setText(0, sBallPosRelative.str().c_str());

   this->visionNumBalls->setText(0,createSufPref("# Balls: ", numBalls, ""));

   int numFeet = feet.size();
   this->visionNumFeet->setText(0,createSufPref("# Feet: ", numFeet, ""));

   stringstream foot1;
   foot1 << "Foot 1 Box: ";
   stringstream foot2;
   foot2 << "Foot 2 Box: ";
   if (numFeet > 0) {
      foot1 << "(" << feet[0].robotBounds.a.x() << ", " << feet[0].robotBounds.a.y() << ")" << endl;
      foot1 << "  (" << feet[0].robotBounds.b.x() << ", " << feet[0].robotBounds.b.y() << ")" << endl;
      if (numFeet > 1) {
          foot2 << "(" << feet[1].robotBounds.a.x() << ", " << feet[1].robotBounds.a.y() << ")" << endl;
          foot2 << "  (" << feet[1].robotBounds.b.x() << ", " << feet[1].robotBounds.b.y() << ")" << endl;
      }
   }
   visionFoot1->setText(0, foot1.str().c_str());
   visionFoot2->setText(0, foot2.str().c_str());

   stringstream head;
   head.str("");
   head << "HeadYaw = " << headYaw;
   motionHeadYaw->setText (0, head.str ().c_str ());

   head.str("");
   head << "HeadYawK = " << headYawK;
   kinematicsHeadYaw->setText (0, head.str ().c_str ());

   // Display info collected from frames in the last second
   visionframesProcessed->setText(0, createSufPref("Frames processed: ",
      lastSecondInfo.num_frames, ""));

   cornersObserved->setText(0, createSufPref("observed: ",
      lastSecondInfo.num_corners,
      createSufPref(" (",
         (double) lastSecondInfo.num_corners_per_frame,
         " per frame)")));

   std::string cornersDistanceString = "(" +
      boost::lexical_cast<std::string>(lastSecondInfo.min_corners_distance) + "/" +
      boost::lexical_cast<std::string>(lastSecondInfo.avg_corners_distance) + "/" +
      boost::lexical_cast<std::string>(lastSecondInfo.max_corners_distance) + ")";
   cornersDistance->setText(0, createSufPref("distance: ",
      cornersDistanceString, ""));

   tJunctionsObserved->setText(0, createSufPref("observed: ",
      lastSecondInfo.num_t_junctions,
      createSufPref(" (",
         (double) lastSecondInfo.num_t_junctions_per_frame,
         " per frame)")));

   std::string tJunctionsDistanceString = "(" +
      boost::lexical_cast<std::string>(lastSecondInfo.min_t_junctions_distance) + "/" +
      boost::lexical_cast<std::string>(lastSecondInfo.avg_t_junctions_distance) + "/" +
      boost::lexical_cast<std::string>(lastSecondInfo.max_t_junctions_distance) + ")";
   tJunctionsDistance->setText(0, createSufPref("distance: ",
      tJunctionsDistanceString, ""));

   postsObserved->setText(0, createSufPref("observed: ",
      lastSecondInfo.num_posts,
      createSufPref(" (",
         (double) lastSecondInfo.num_posts_per_frame,
         " per frame)")));

   std::string postskDistanceString = "(" +
      boost::lexical_cast<std::string>(lastSecondInfo.min_posts_kDistance) + "/" +
      boost::lexical_cast<std::string>(lastSecondInfo.avg_posts_kDistance) + "/" +
      boost::lexical_cast<std::string>(lastSecondInfo.max_posts_kDistance) + ")";
   postskDistance->setText(0, createSufPref("kDistance: ",
      postskDistanceString, ""));

   std::string postswDistanceString = "(" +
      boost::lexical_cast<std::string>(lastSecondInfo.min_posts_wDistance) + "/" +
      boost::lexical_cast<std::string>(lastSecondInfo.avg_posts_wDistance) + "/" +
      boost::lexical_cast<std::string>(lastSecondInfo.max_posts_wDistance) + ")";
   postswDistance->setText(0, createSufPref("wDistance: ",
      postswDistanceString, ""));

   ballsObserved->setText(0, createSufPref("observed: ",
      lastSecondInfo.num_balls,
      createSufPref(" (",
         (double) lastSecondInfo.num_balls_per_frame,
         " per frame)")));

   std::string ballsDistanceString = "(" +
      boost::lexical_cast<std::string>(lastSecondInfo.min_balls_distance) + "/" +
      boost::lexical_cast<std::string>(lastSecondInfo.avg_balls_distance) + "/" +
      boost::lexical_cast<std::string>(lastSecondInfo.max_balls_distance) + ")";
   ballsDistance->setText(0, createSufPref("distance: ",
      ballsDistanceString, ""));

   linesObserved->setText(0, createSufPref("observed: ",
      lastSecondInfo.num_lines,
      createSufPref(" (",
         (double) lastSecondInfo.num_lines_per_frame,
         " per frame)")));

   penaltySpotsObserved->setText(0, createSufPref("observed: ",
      lastSecondInfo.num_penalty_spots,
      createSufPref(" (",
         (double) lastSecondInfo.num_penalty_spots_per_frame,
         " per frame)")));

   centreCirclesObserved->setText(0, createSufPref("observed: ",
      lastSecondInfo.num_centre_circles,
      createSufPref(" (",
         (double) lastSecondInfo.num_centre_circles_per_frame,
         " per frame)")));

   fieldLinePointsObserved->setText(0, createSufPref("observed: ",
      lastSecondInfo.num_field_line_points,
      createSufPref(" (",
         (double) lastSecondInfo.num_field_line_points_per_frame,
         " per frame)")));

   robotsObserved->setText(0, createSufPref("observed: ",
      lastSecondInfo.num_robots,
      createSufPref(" (",
         (double) lastSecondInfo.num_robots_per_frame,
         " per frame)")));

}
