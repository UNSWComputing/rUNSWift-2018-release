#include "types/BroadcastData.hpp"
#include "utils/SPLDefs.hpp"

bool BroadcastData::sanityCheck()
{
    if (playerNum < 1)
    {
        std::cout << "received playerNum less than 1" << std::endl;
        return false;
    }

    if (playerNum > ROBOTS_PER_TEAM)
    {
        std::cout << "received playerNum greater than ROBOTS_PER_TEAM" << std::endl;
        return false;
    }

    if (abs(ballPosAbs.x()) > FULL_FIELD_LENGTH / 2 ||
        abs(ballPosAbs.y()) > FULL_FIELD_WIDTH / 2)   
    {
        std::cout << "received ballPosAbs off the field" << std::endl;
        return false;
    }

    if (pow(ballPosRR.distance(), 2) > pow(FULL_FIELD_LENGTH, 2) + pow(FULL_FIELD_WIDTH, 2)){
        std::cout << "received ballPosRR with distance longer than diagonal of the field" << std::endl;
        return false;
    }

    if (abs(ballPosRR.heading()) > 2*M_PI)
    {
        std::cout << "received ballPosRR heading greater or less than 2*pi" << std::endl;
        return false;
    }

    if (abs(ballPosRR.orientation()) > 2*M_PI)
    {
        std::cout << "received ballPosRR orientation greater or less than 2*pi" << std::endl;
        return false;
    }

    if (!sharedLocalisationBundle.sanityCheck())
    {
        std::cout << "received sharedLocalisationBundle that didn't pass sanity check" << std::endl;
        return false;
    }

    if (!behaviourSharedData.sanityCheck())
    {
        return false;
    }

    return true;
}