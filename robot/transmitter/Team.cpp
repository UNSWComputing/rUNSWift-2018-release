#include "Team.hpp"
#include "blackboard/Blackboard.hpp"
#include "types/SPLStandardMessage.hpp"
#include "utils/incapacitated.hpp"

#include <iostream>

using namespace boost::asio;
using namespace std;

TeamTransmitter::TeamTransmitter(Blackboard *bb) :
   Adapter(bb),
   NaoTransmitter((bb->config)["network.transmitter_base_port"].as<int>()
                  + (bb->config)["player.team"].as<int>(),
                  (bb->config)["network.transmitter_address"].as<string>()),
   service(),
   socket(service, ip::udp::v4()),
   delay(0)
{}

void TeamTransmitter::tick() {
   BroadcastData bd((blackboard->config)["player.number"].as<int>(),
                    readFrom(localisation, robotPos),
                    readFrom(localisation, ballPos),
                    readFrom(localisation, ballPosRR),
                    readFrom(localisation, ballLostCount),
                    readFrom(localisation, sharedLocalisationBundle),
                    readFrom(behaviour, behaviourSharedData),
                    readFrom(motion, active).body.actionType,
                    readFrom(motion, uptime),
                    readFrom(gameController, gameState));

   // calculate incapacitated
   int playerNum = (blackboard->config)["player.number"].as<int>();
   bool incapacitated = false;
   if (readFrom(gameController, our_team).players[playerNum - 1].penalty
       != PENALTY_NONE) {
      incapacitated = true;
   }

   const ActionCommand::Body::ActionType &acB =
            readFrom(motion, active).body.actionType;
   incapacitated |= isIncapacitated(acB);

   const AbsCoord &robotPos = readFrom(localisation, robotPos);

   SPLStandardMessage m (playerNum,
                         readFrom(gameController, our_team).teamNumber, // team
                         incapacitated, // fallen
                         robotPos,
                         readFrom(localisation, ballLostCount),
                         readFrom(localisation, ballPos),
                         bd);

   writeTo(localisation, havePendingOutgoingSharedBundle, false);
   NaoTransmitter::tick(boost::asio::buffer(&m, sizeof(SPLStandardMessage)));

   // hax to send the gc packet once every two team ticks
   ++delay;
   if (delay > 5) {
      sendToGameController();
      delay = 0;
   }
 }

TeamTransmitter::~TeamTransmitter() {
   socket.close();
}

void TeamTransmitter::sendToGameController() {
   char* sendToIP = readFrom(gameController, lastGameControllerIPAddress);
   if (sendToIP == NULL) {
      // sendToIP is only initialised once GameController sends us a packet
      return;
   }

   gameControllerEndpoint.address(ip::address::from_string(sendToIP));
   gameControllerEndpoint.port(GAMECONTROLLER_RETURN_PORT);
   boost::system::error_code ec = boost::system::error_code();
   socket.connect(gameControllerEndpoint, ec);

   RoboCupGameControlReturnData d = RoboCupGameControlReturnData();
   d.team = (blackboard->config)["player.team"].as<int>();
   d.player = (blackboard->config)["player.number"].as<int>();
   d.message = GAMECONTROLLER_RETURN_MSG_ALIVE;

   // TODO (Peter): If GameController PC goes away, we should get some kind of
   // catchable error here so we can stop sending packets until another
   // GC comes online
   socket.send(boost::asio::buffer(&d, sizeof(RoboCupGameControlReturnData)), 0, ec);
}
