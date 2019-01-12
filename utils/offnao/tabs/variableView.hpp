#pragma once

#include <QTreeWidget>
#include <QTreeWidgetItem>
#include <deque>
#include <string>

#include "naoData.hpp"
#include "perception/vision/other/YUV.hpp"
#include "perception/vision/VisionDefinitions.hpp"
#include "types/BallInfo.hpp"

class VariableView : public QTreeWidget {
   public:
      VariableView();
      void redraw(NaoData *naoData);

   private:
      template <class T>
         const char* createSufPref(std::string, T, std::string);

      QTreeWidgetItem *behaviourHeading;
      QTreeWidgetItem *behaviourBodyRequest;
      QTreeWidgetItem *behaviourHierarchy;

      QTreeWidgetItem *motionHeading;

      QTreeWidgetItem *gameControllerHeading;
      QTreeWidgetItem *gameControllerTeam;

      QTreeWidgetItem *perceptionHeading;
      QTreeWidgetItem *perceptionAverageFPS;
      QTreeWidgetItem *perceptionKinematicsTime;
      QTreeWidgetItem *perceptionVisionTime;
      QTreeWidgetItem *perceptionLocalisationTime;
      QTreeWidgetItem *perceptionBehaviourTime;
      QTreeWidgetItem *perceptionTotalTime;


      QTreeWidgetItem *visionTimestamp;
      QTreeWidgetItem *visionDxdy;
      QTreeWidgetItem *visionFrames;
      QTreeWidgetItem *visionHeading;
      QTreeWidgetItem *visionGoal;
      QTreeWidgetItem *visionGoalProb;
      QTreeWidgetItem *visionHomeMapSize;
      QTreeWidgetItem *visionAwayMapSize;
      QTreeWidgetItem *visionBallPos;
      QTreeWidgetItem *visionBallPosRobotRelative;
      QTreeWidgetItem *visionFrameRate;
      QTreeWidgetItem *visionNumBalls;
      QTreeWidgetItem *visionPostType1;
      QTreeWidgetItem *visionPostType2;
      QTreeWidgetItem *visionPost1;
      QTreeWidgetItem *visionPost2;
      QTreeWidgetItem *visionPostInfo1;
      QTreeWidgetItem *visionPostInfo2;
      QTreeWidgetItem *visionNumFeet;
      QTreeWidgetItem *visionFoot1;
      QTreeWidgetItem *visionFoot2;

      QTreeWidgetItem *lastSecond;
      QTreeWidgetItem *visionframesProcessed;
      QTreeWidgetItem *cornersHeading;
      QTreeWidgetItem *cornersObserved;
      QTreeWidgetItem *cornersDistance;
      QTreeWidgetItem *tJunctionsHeading;
      QTreeWidgetItem *tJunctionsObserved;
      QTreeWidgetItem *tJunctionsDistance;
      QTreeWidgetItem *postsHeading;
      QTreeWidgetItem *postsObserved;
      QTreeWidgetItem *postskDistance;
      QTreeWidgetItem *postswDistance;
      QTreeWidgetItem *ballsHeading;
      QTreeWidgetItem *ballsObserved;
      QTreeWidgetItem *ballsDistance;
      QTreeWidgetItem *linesHeading;
      QTreeWidgetItem *linesObserved;
      QTreeWidgetItem *penaltySpotsHeading;
      QTreeWidgetItem *penaltySpotsObserved;
      QTreeWidgetItem *centreCirclesHeading;
      QTreeWidgetItem *centreCirclesObserved;
      QTreeWidgetItem *fieldLinePointsHeading;
      QTreeWidgetItem *fieldLinePointsObserved;
      QTreeWidgetItem *robotsHeading;
      QTreeWidgetItem *robotsObserved;

      QTreeWidgetItem *motionHeadYaw;
      QTreeWidgetItem *kinematicsHeadYaw;

      QTreeWidgetItem *localisationHeading;
      QTreeWidgetItem *localisationRobotPosx;
      QTreeWidgetItem *localisationRobotPosy;
      QTreeWidgetItem *localisationRobotHeading;
      QTreeWidgetItem *localisationRobotTopWeight;
      QTreeWidgetItem *localisationRobotHypoth;
      QTreeWidgetItem *localisationBallLost;
      QTreeWidgetItem *localisationBallSeen;
      QTreeWidgetItem *localisationBallx;
      QTreeWidgetItem *localisationBally;
      QTreeWidgetItem *localisationBallVelx;
      QTreeWidgetItem *localisationBallVely;

      std::deque<int> times;
      void updateVision(NaoData *naoData);
      void updateBehaviour(NaoData *naoData);

};
