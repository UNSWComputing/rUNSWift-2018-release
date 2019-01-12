#include "LocalisationUtils.hpp"
#include "utils/Logger.hpp"

#include <iostream>

bool absDistClose(const AbsCoord &abs1, const AbsCoord &abs2, float distThreshold){
   return (abs(abs1.x() - abs2.x()) <= distThreshold) &&
          (abs(abs1.y() - abs2.y()) <= distThreshold);
}

bool absClose(const AbsCoord &abs1, const AbsCoord &abs2, float headingThreshold){
   return (abs(abs1.x() - abs2.x()) <= ABS_DIST_THRESHOLD) &&
          (abs(abs1.y() - abs2.y()) <= ABS_DIST_THRESHOLD) &&
          (fabs(NORMALISE(abs1.theta() - abs2.theta())) <= headingThreshold);
}

/* Minimum Mahalanobis distance of abs1 to abs2 and abs2 to abs1  */
/* ¿Por qué no los dos? */
float minMahalanobisDistance(const AbsCoord &abs1, const AbsCoord &abs2) {
   Eigen::Vector2f posDiff(abs1.x() - abs2.x(), abs1.y() - abs2.y());
   float headingDiff = fabs(NORMALISE(abs1.theta() - abs2.theta()));

   Eigen::Matrix<float, 1, 1> posDistSquared1 = posDiff.transpose() * abs1.var.block<2,2>(0,0).inverse() * posDiff;
   float headingDistSquared1 = headingDiff * (1/abs1.var(2,2)) * headingDiff;
   float mahaDist1 = sqrt(posDistSquared1[0]) + sqrt(headingDistSquared1);

   Eigen::Matrix<float, 1, 1> posDistSquared2 = posDiff.transpose() * abs2.var.block<2,2>(0,0).inverse() * posDiff;
   float headingDistSquared2 = headingDiff * (1/abs2.var(2,2)) * headingDiff;
   float mahaDist2 = sqrt(posDistSquared2[0]) + sqrt(headingDistSquared2);

   return std::min(mahaDist1, mahaDist2);
}

/* Mahalanobis distance from an observation AbsCoord to a state position/heading & variance */
float mahalanobisDistance(const AbsCoord &abs, const AbsCoord &state) {
   Eigen::Vector2f posDiff(abs.x() - state.x(), abs.y() - state.y());
   float headingDiff = fabs(NORMALISE(abs.theta() - state.theta()));

   Eigen::Matrix<float, 1, 1> posDistSquared = posDiff.transpose() * state.var.block<2,2>(0,0).inverse() * posDiff;
   float headingDistSquared = headingDiff * (1/state.var(2,2)) * headingDiff;

   //TODO consider using squared values to reduce need for sqrts
   float posDist = sqrt(posDistSquared[0]);
   float headingDist = sqrt(headingDistSquared);
   return posDist + headingDist;
}

/* Mahalanobis distance from an observation AbsCoord to a state position & variance */
float mahalanobisDistancePos(const AbsCoord &abs, const AbsCoord &state) {
   Eigen::Vector2f posDiff(abs.x() - state.x(), abs.y() - state.y());
   Eigen::Matrix<float, 1, 1> posDistSquared = posDiff.transpose() * state.var.block<2,2>(0,0).inverse() * posDiff;
   return sqrt(posDistSquared[0]);
}

float mahalanobisDistancePosSQR(const AbsCoord &abs, const AbsCoord &state) {
   Eigen::Vector2f posDiff(abs.x() - state.x(), abs.y() - state.y());
   return (posDiff.transpose() * state.var.block<2,2>(0,0).inverse() * posDiff)[0];
}

/* Squared Mahalanobis distance from an observation Point to a state Point & variance */
float mahalanobisDistanceSQR(const PointF &obs, const PointF &state, const Eigen::Matrix<float, 2, 2> &state_var) {
   PointF posDiff = obs - state;
   Eigen::Matrix<float, 1, 1> posDistSquared = posDiff.transpose() * state_var.inverse() * posDiff;
   return posDistSquared[0];
}

void logAbsCoord(const AbsCoord abs, std::string name) {
   std::cout << name << ": (" << abs.x() << "," << abs.y() << "," << RAD2DEG(abs.theta()) << ")" << std::endl;
}

void logAbsCoordVar(const AbsCoord abs, std::string name) {
   std::cout << name << ": (" << abs.x() << "," << abs.y() << "," << RAD2DEG(abs.theta()) << ")" << std::endl;
   std::cout << name << " var: (" << abs.var(X,X) << "," << abs.var(Y,Y) << "," << abs.var(THETA,THETA) << ")" << std::endl;
}

void logAbsCoordCov(const AbsCoord abs, std::string name) {
   std::cout << name << ": (" << abs.x() << "," << abs.y() << "," << RAD2DEG(abs.theta()) << ")" << std::endl;
   std::cout << name << " covariance:" << std::endl;
   std::cout << abs.var(0,0) << "  ,  " << abs.var(0,1) << "  ,  " << abs.var(0,2) << std::endl;
   std::cout << abs.var(1,0) << "  ,  " << abs.var(1,1) << "  ,  " << abs.var(1,2) << std::endl;
   std::cout << abs.var(2,0) << "  ,  " << abs.var(2,1) << "  ,  " << abs.var(2,2) << std::endl;
}

Eigen::MatrixXd sparseMultiplication(
      const Eigen::MatrixXd &lhs, unsigned rowStart, unsigned colStart, unsigned rowEnd, unsigned colEnd,
      const Eigen::MatrixXd &rhs) {

   if (lhs.cols() != rhs.rows()) {
      std::cout << "sparseMultiplication() ERROR: incompatible matrices" << std::endl;
      return Eigen::MatrixXd();
   }

   Eigen::MatrixXd result(lhs.rows(), rhs.cols());
   result.setZero();
   for (unsigned i = rowStart; i < rowEnd; i++) {
      for (unsigned j = colStart; j < colEnd; j++) {
         for (unsigned k = 0; k < (unsigned int) rhs.cols(); k++) {
            result(i, k) += lhs(i, j) * rhs(j, k);
         }
      }
   }

   return result;
}

Eigen::MatrixXd sparseMultiplication(
      const Eigen::MatrixXd &lhs,
      const Eigen::MatrixXd &rhs, unsigned rowStart, unsigned colStart, unsigned rowEnd, unsigned colEnd) {

   if (lhs.cols() != rhs.rows()) {
      std::cout << "sparseMultiplication() ERROR: incompatible matrices" << std::endl;
      return Eigen::MatrixXd();
   }

   Eigen::MatrixXd result(lhs.rows(), rhs.cols());
   result.setZero();

   for (unsigned i = 0; i < (unsigned int) lhs.rows(); i++) {
      for (unsigned j = rowStart; j < rowEnd; j++) {
         for (unsigned k = colStart; k < colEnd; k++) {
            result(i, k) += lhs(i, j) * rhs(j, k);
         }
      }
   }

   return result;
}

