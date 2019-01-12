#pragma once

#include "LocalisationDefs.hpp"
#include "types/AbsCoord.hpp"
#include <Eigen/Eigen>
#include <iostream>
#include <iomanip>
#include <ios>

#define ABS_DIST_THRESHOLD 500
#define MAHA_DIST_THRESHOLD 3

#define TEAM_BALL_ABS_THRESHOLD 750.0
#define TEAM_BALL_ABS_FAR_THRESHOLD 2000.0
#define TEAM_BALL_MAHA_THRESHOLD 1.0

// Assertion call disabled as we don't want to crash if we don't have to.
#define MY_ASSERT(X, Y) if (!(X)) { std::cout << "ERROR DETECTED: " << Y << std::endl; } assert(X)
#define MY_ASSERT_2(X) if (!(X)) { std::cout << "ERROR DETECTED" << std::endl; } assert(X)

#define MAX_GAUSSIANS 8

bool absDistClose(const AbsCoord &abs1, const AbsCoord &abs2, float distThreshold);

bool absClose(const AbsCoord &abs1, const AbsCoord &abs2, float headingThreshold);

float minMahalanobisDistance(const AbsCoord &abs1, const AbsCoord &abs2);

float mahalanobisDistance(const AbsCoord &abs, const AbsCoord &state);

float mahalanobisDistancePos(const AbsCoord &abs, const AbsCoord &state);

float mahalanobisDistancePosSQR(const AbsCoord &abs, const AbsCoord &state);

float mahalanobisDistanceSQR(const PointF &obs, const PointF &state, const Eigen::Matrix<float, 2, 2> &state_var);

/**
 * Gets the variance that corresponds to being 95% confident the variable falls within the given limit.
 */
inline float get95CF(float limit) {
   return limit * limit / 4.0f;
}

template<int ROWS, int COLS>
Eigen::MatrixXd copyStaticToDynamicMatrix(const Eigen::Matrix<float, ROWS, COLS> &matrix) {
   Eigen::MatrixXd result(ROWS, COLS);
   for (int i = 0; i < ROWS; i++) {
      for (int j = 0; j < COLS; j++) {
         result(i, j) = matrix(i, j);
      }
   }
   return result;
}

Eigen::MatrixXd sparseMultiplication(
      const Eigen::MatrixXd &lhs, unsigned rowStart, unsigned colStart, unsigned rowEnd, unsigned colEnd,
      const Eigen::MatrixXd &rhs);

Eigen::MatrixXd sparseMultiplication(
      const Eigen::MatrixXd &lhs,
      const Eigen::MatrixXd &rhs, unsigned rowStart, unsigned colStart, unsigned rowEnd, unsigned colEnd);

inline void prettyOutputMatrix(std::string name, const Eigen::MatrixXd &matrix) {
   if (matrix.cols() == 1) {
      std::cout << name << ":";
      for (int i = 0; i < matrix.rows(); i++) {
         std::cout << "\t" << std::fixed << std::setprecision(1) << matrix(i, 0);
      }
      std::cout << std::endl;
   } else {
      std::cout << name << std::endl;
      for (int i = 0; i < matrix.rows(); i++) {
         for (int j = 0; j < matrix.cols(); j++) {
            std::cout << std::fixed << std::setprecision(1) << matrix(i, j) << "\t";
         }
         std::cout << std::endl;
      }
   }
}

/* logging */
void logAbsCoord(const AbsCoord abs, std::string name);
void logAbsCoordVar(const AbsCoord abs, std::string name);
void logAbsCoordCov(const AbsCoord abs, std::string name);
