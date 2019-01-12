/**
 * GameController.hpp
 * Description: A thread to recieve game state information from the Game
 * Controller sever and implements the Button Interface. Adapted from 2009 code.
 */

#pragma once

#include <vector>
#include "gamecontroller/RoboCupGameControlData.hpp"
#include "types/ButtonPresses.hpp"
#include "blackboard/Adapter.hpp"

class GameController : Adapter {
    public:
        // Constructor
        GameController(Blackboard *bb);
        // Destructor
        ~GameController();
        // Called on each cycle
        void tick();
    private:
        RoboCupGameControlData data;
        TeamInfo our_team;
        bool team_red;
        bool connected;
        int sock;

        /**
         * Connect to the GameController
         */
        void initialiseConnection();

        /**
         * Update the state using the Button Interface
         */
        void buttonUpdate();

        /**
         * Update the state using the GameController Interface
         */
        void wirelessUpdate();

        /**
         * Update the robot's state to do things like leave the WiFi and
         * self-terminate on `runswift` start.
         */
        void handleFinishedPacket();

        /**
         * Return True if a whistle file was created in the last num_seconds.
         * Should be a mirror of whistle_detector.py:whistle_heard function.
         */
        bool whistleHeard(int numSeconds);

        /* Flag to turn off acting on whistle if game does not need it */
        bool actOnWhistle;

        /**
         * Parse data from the GameController
         * @param update Pointer to newly recieved GC data
         */
        void parseData(RoboCupGameControlData *update);

        // parseData helper functions

        bool isValidData(RoboCupGameControlData *gameData);

        bool checkHeader(char* header);

        bool isThisGame(RoboCupGameControlData *gameData);

        bool gameDataEqual(void* gameData, void* previous);

        void rawSwapTeams(RoboCupGameControlData *gameData);

        /**
         * Internal state for speech updates
         */
        uint8_t lastState;
        uint16_t myLastPenalty;

        /* Player & team number re-checked from config each cycle */
        int playerNumber;
        int teamNumber;

        /* Structure containing mask of buttons that have been pushed */
        ButtonPresses buttons;

        /**
         * Structure containing the timestamps of when STATE_FINISHED
         * packets were received
         */
        std::vector<time_t> finished_times;
};
