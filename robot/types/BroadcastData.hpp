#pragma once

#include "types/AbsCoord.hpp"
#include "types/RRCoord.hpp"
#include "types/ActionCommand.hpp"
#include "types/BehaviourSharedData.hpp"
#include "perception/localisation/SharedLocalisationUpdateBundle.hpp"
#include "gamecontroller/RoboCupGameControlData.hpp"

class BroadcastData {
  public:
    BroadcastData()
        : playerNum(0),
          ballPosAbs(),
          ballPosRR(),
          lostCount(0),
          sharedLocalisationBundle(),
          behaviourSharedData(),
          acB(ActionCommand::Body::DEAD),
          uptime(0.0),
          gameState(STATE_INITIAL)
    {
        robotPos[0] = 0.f;
        robotPos[1] = 0.f;
        robotPos[2] = 0.f;
    }

    BroadcastData(const BroadcastData& bd)
        : playerNum(bd.playerNum),
          ballPosAbs(bd.ballPosAbs),
          ballPosRR(bd.ballPosRR),
          lostCount(bd.lostCount), // TODO: this really should be "ballLostCount"
          sharedLocalisationBundle(bd.sharedLocalisationBundle),
          behaviourSharedData(bd.behaviourSharedData),
          acB(bd.acB),
          uptime(bd.uptime),
          gameState(bd.gameState)
    {
        robotPos[0] = bd.robotPos[0];
        robotPos[1] = bd.robotPos[1];
        robotPos[2] = bd.robotPos[2];
    }

    BroadcastData(const int &playerNum,
        const AbsCoord &RobotPos,
        const AbsCoord &ballPosAbs,
        const RRCoord &ballPosRR,
        const uint32_t &lostCount,
        const SharedLocalisationUpdateBundle &sharedLocalisationBundle,
        const BehaviourSharedData &behaviourSharedData,
        const ActionCommand::Body::ActionType &acB,
        const float &uptime,
        const uint8_t &gameState)
        : playerNum(playerNum),
          ballPosAbs(ballPosAbs),
          ballPosRR(ballPosRR),
          lostCount(lostCount),
          sharedLocalisationBundle(sharedLocalisationBundle),
          behaviourSharedData(behaviourSharedData),
          acB(acB),
          uptime(uptime),
          gameState(gameState)
    {
        robotPos[0] = RobotPos.x();
        robotPos[1] = RobotPos.y();
        robotPos[2] = RobotPos.theta();
    }

    bool sanityCheck();

    int playerNum;     

    float robotPos[3];
    AbsCoord ballPosAbs;
    RRCoord ballPosRR;
    uint32_t lostCount;

    SharedLocalisationUpdateBundle sharedLocalisationBundle;

    // Data set by the Python behaviours that is shared with other robots.
    BehaviourSharedData behaviourSharedData;

    ActionCommand::Body::ActionType acB;
    float uptime;
    uint8_t gameState;

    template<class Archive>
    void serialize(Archive &ar, const unsigned int file_version) {
        ar & playerNum;
        if (file_version == 0) {
            int team;
            AbsCoord robotPos;
            ar & team;
            ar & robotPos;
        } else {
            ar & robotPos;
        }
        ar & ballPosAbs;
        ar & ballPosRR;
        ar & lostCount;
        ar & sharedLocalisationBundle;
        ar & behaviourSharedData;
        ar & acB;
        ar & uptime;
        ar & gameState;
    }
};
BOOST_CLASS_VERSION(BroadcastData, 1);