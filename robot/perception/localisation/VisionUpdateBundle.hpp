#pragma once

#include "types/ActionCommand.hpp"
#include "types/BallInfo.hpp"
#include "types/FieldBoundaryInfo.hpp"
#include "types/FieldFeatureInfo.hpp"
#include "types/PostInfo.hpp"
#include "types/RobotInfo.hpp"
#include "types/AbsCoord.hpp"

#include <vector>

/**
 * A container for all of the input data to the localisation system during a vision update.
 */
struct VisionUpdateBundle {

   VisionUpdateBundle() :
      amIOnRedTeam(false),
      awayGoalProb(0.0f),
      headYaw(0.0f),
      isHeadingReliable(false),
      isDistanceReliable(false) {}

   VisionUpdateBundle(
         const std::vector<FieldBoundaryInfo> &fieldBoundaries,
         const std::vector<FieldFeatureInfo> &fieldFeatures,
         const std::vector<PostInfo> &posts,
         const std::vector<RobotInfo> &robots,
         const std::vector<AbsCoord> &teammatePositions,
         const bool amIOnRedTeam,
         const float awayGoalProb,
         const float headYaw,
         const std::vector<BallInfo> &visibleBalls,
         const bool isHeadingReliable,
         const bool isDistanceReliable) :
            fieldBoundaries(fieldBoundaries),
            fieldFeatures(fieldFeatures),
            posts(posts),
            robots(robots),
            teammatePositions(teammatePositions),
            amIOnRedTeam(amIOnRedTeam),
            awayGoalProb(awayGoalProb),
            headYaw(headYaw),
            visibleBalls(visibleBalls),
            isHeadingReliable(isHeadingReliable),
            isDistanceReliable(isDistanceReliable) {}

   // Visible field lines in the latest frame.
   std::vector<FieldBoundaryInfo> fieldBoundaries;

   // Visible field line features in the latest frame.
   std::vector<FieldFeatureInfo> fieldFeatures;

   // Visible goal posts in the latest frame.
   std::vector<PostInfo> posts;

   // Visible robots in the latest frame.
   std::vector<RobotInfo> robots;

   // This is kind of a remnant from a quick hack we did to use teammates as landmarks.
   // This should be removed/or done properly.
   std::vector<AbsCoord> teammatePositions;

   bool amIOnRedTeam;

   // The probability that the visible goal posts are the opponent goals. Value range [0.0, 1.0]
   float awayGoalProb;

   float headYaw;

   // Currently visible balls. We allow multiple visible balls to account for false-positives.
   std::vector<BallInfo> visibleBalls;


   bool isHeadingReliable;
   bool isDistanceReliable;
};

inline std::ostream& operator<<(std::ostream& os, const VisionUpdateBundle& bundle) {
   unsigned numFieldBoundaries = bundle.fieldBoundaries.size();
   os.write((char*) &numFieldBoundaries, sizeof(unsigned));
   for (unsigned i = 0; i < bundle.fieldBoundaries.size(); i++) {
      os << bundle.fieldBoundaries[i];
   }

   unsigned numFieldFeatures = bundle.fieldFeatures.size();
   os.write((char*) &numFieldFeatures, sizeof(unsigned));
   for (unsigned i = 0; i < bundle.fieldFeatures.size(); i++) {
      os << bundle.fieldFeatures[i];
   }

   unsigned numPosts = bundle.posts.size();
   os.write((char*) &numPosts, sizeof(unsigned));
   for (unsigned i = 0; i < bundle.posts.size(); i++) {
      os << bundle.posts[i];
   }

   os.write((char*) &(bundle.awayGoalProb), sizeof(float));
   os.write((char*) &(bundle.headYaw), sizeof(float));
   os.write((char*) &(bundle.isHeadingReliable), sizeof(bool));
   os.write((char*) &(bundle.isDistanceReliable), sizeof(bool));

   unsigned numVisibleBalls = bundle.visibleBalls.size();
   os.write((char*) &numVisibleBalls, sizeof(unsigned));
   for (unsigned i = 0; i < bundle.visibleBalls.size(); i++) {
      os << bundle.visibleBalls[i];
   }

   return os;
}

inline std::istream& operator>>(std::istream& is, VisionUpdateBundle& bundle) {
   unsigned numFieldBoundaries;
   is.read((char*) &numFieldBoundaries, sizeof(unsigned));
   bundle.fieldBoundaries.clear();
   for (unsigned i = 0; i < numFieldBoundaries; i++) {
      FieldBoundaryInfo newFieldBoundary;
      is >> newFieldBoundary;
      bundle.fieldBoundaries.push_back(newFieldBoundary);
   }

   unsigned numFieldFeatures;
   is.read((char*) &numFieldFeatures, sizeof(unsigned));
   bundle.fieldFeatures.clear();
   for (unsigned i = 0; i < numFieldFeatures; i++) {
      FieldFeatureInfo newFieldFeature;
      is >> newFieldFeature;
      bundle.fieldFeatures.push_back(newFieldFeature);
   }

   unsigned numPosts;
   is.read((char*) &numPosts, sizeof(unsigned));
   bundle.posts.clear();
   for (unsigned i = 0; i < numPosts; i++) {
      PostInfo newPost;
      is >> newPost;
      bundle.posts.push_back(newPost);
   }

   is.read((char*) &(bundle.awayGoalProb), sizeof(float));
   is.read((char*) &(bundle.headYaw), sizeof(float));
   is.read((char*) &(bundle.isHeadingReliable), sizeof(bool));
   is.read((char*) &(bundle.isDistanceReliable), sizeof(bool));

   unsigned numVisibleBalls;
   is.read((char*) &numVisibleBalls, sizeof(unsigned));
   bundle.visibleBalls.clear();
   for (unsigned i = 0; i < numVisibleBalls; i++) {
      BallInfo newBall;
      is >> newBall;
      bundle.visibleBalls.push_back(newBall);
   }

   return is;
}
