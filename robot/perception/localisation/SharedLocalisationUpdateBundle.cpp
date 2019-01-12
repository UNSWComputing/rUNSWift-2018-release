#include "perception/localisation/SharedLocalisationUpdateBundle.hpp"

bool SharedLocalisationUpdateBundle::sanityCheck()
{
    // Check for ballSeenFraction nan
    if (isnan(ballSeenFraction)){
        std::cout << "received nan for ballSeenFraction" << std::endl;
        return false;
    }

    // Check for nans in mean
    for (int i = 0; i < SHARED_DIM; i++) {
        if (isnan(sharedUpdateMean(i, 0))){
            std::cout << "received nan in sharedUpdateMean" << std::endl;
            return false;
        }
    }

    // Check for nans in covariance
    for (int row = 0; row < SHARED_DIM; row++) {
        for (int col = 0; col < SHARED_DIM; col++) {
            if (isnan(sharedUpdateCovariance(row, col))){
                std::cout << "received nan in sharedUpdateCovariance" << std::endl;
                return false;
            }
        }
    }

    // Check for sharedDx nan
    if (isnan(sharedDx)){
        std::cout << "received nan for sharedDx" << std::endl;
        return false;
    }

    // Check for sharedDy nan
    if (isnan(sharedDy)){
        std::cout << "received nan for sharedDy" << std::endl;
        return false;
    }

    // Check for sharedDh nan
    if (isnan(sharedDh)){
        std::cout << "received nan for sharedDh" << std::endl;
        return false;
    }

    // Check for sharedCovarianceDx nan
    if (isnan(sharedCovarianceDx)){
        std::cout << "received nan for sharedCovarianceDx" << std::endl;
        return false;
    }

    // Check for sharedCovarianceDy nan
    if (isnan(sharedCovarianceDy)){
        std::cout << "received nan for sharedCovarianceDy" << std::endl;
        return false;
    }

    // Check for sharedCovarianceDh nan
    if (isnan(sharedCovarianceDh)){
        std::cout << "received nan for sharedCovarianceDh" << std::endl;
        return false;
    }
    
    return true;
}