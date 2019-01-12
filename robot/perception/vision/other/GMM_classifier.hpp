#ifndef PERCEPTION_OTHER_GMM_
#define PERCEPTION_OTHER_GMM_

#include <cstdio>
#include <string>
#include <cmath>

#include <vector>
#include <limits>

#include <Eigen/Eigen>

#include "perception/vision/Region.hpp"

#define CLASSIFIER_TRUE 0
#define CLASSIFIER_FALSE 1

typedef Eigen::Matrix<double, 1, Eigen::Dynamic> Vectorxf;
typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> Matrixxf;

// The location directory of model files
#define PCA_MODEL_DIR_BALL "/home/nao/data/ball_classifier.pca"
#define GMM_MODEL_DIR_BALL "/home/nao/data/ball_classifier.gmm"
#define PCA_MODEL_DIR_PENALTY_SPOT "/home/nao/data/penalty_spot_classifier.pca"
#define GMM_MODEL_DIR_PENALTY_SPOT "/home/nao/data/penalty_spot_classifier.gmm"

enum ClassifierType {
  ball,
  penaltySpot,
};

/*
Helper class for GMM that produces the log probability of a Gaussian
distribution
*/
class GaussianDistribution
{
  public:
    GaussianDistribution();
    GaussianDistribution(int, const Vectorxf &, const Matrixxf &, double);
    double logprob(const Vectorxf &);

  private:
    Vectorxf mean_;
    Matrixxf precision_;
    double log_normalise_factor_;
};

/*
Class Conditional Gaussian Mixture Model
Construct GMM for each class and inference using Beysian inference.
*/
class ClassConditionalGMM
{
  public:
    ClassConditionalGMM(ClassifierType type);
    double logprob(int, const Vectorxf &);
    int getNFeatures();
    int getNComponent();
    int getNClasses();

  private:
    int n_classes_;
    int n_features_;
    int n_components_;
    std::vector<double> log_priors_;
    std::vector<std::vector<double> > log_weights_;
    std::vector<std::vector<GaussianDistribution> > gaussians_;
};

/*
PCA
The PCA reduces the dimensions of the input and
outputs the reduced vector
*/
class PCA
{
  public:
    PCA(ClassifierType type);
    Vectorxf transform(const Vectorxf &);
    int getNFeatures();
    int getNComponent();
    int getInputHeight();
    int getInputWidth();

  private:
    int n_features_;
    int input_height_;
    int input_width_;
    int n_components_;
    Vectorxf means_;
    Matrixxf components_T_;
};

/*
Estimator
The interface of the classifier. It will constrct the PCA model and
GMM model from file in the constructor.
*/
class Estimator
{
  public:
    Estimator(ClassifierType type);
    int predict(const RegionI& region);

  private:
    int n_features_;
    int input_height_;
    int input_width_;
    PCA pca_;
    ClassConditionalGMM ccgmm_;
    
    Matrixxf convert(const RegionI& region);
    Vectorxf preprocessor(const Matrixxf &);
    Matrixxf resize(const Matrixxf &, int, int);
		Vectorxf reshape(const Matrixxf &);
};

#endif