#ifndef FINDBALLEXP_RUNSWIFTSERVERRESPONSE_H_
#define FINDBALLEXP_RUNSWIFTSERVERRESPONSE_H_

#include "agent/ToAgent.h"
#include "comms/MessageParser.h"

namespace findballexp
{
    struct ToRunswiftAgent 
        : public librcsscontroller::ToAgent
    {
        int game_state;
        int penalty;

        ToRunswiftAgent()
            : game_state(0), penalty(0)
        { }

        virtual std::string ToMessage() const
        {
            std::stringstream ss;
            ss << "(";
            ss << "(game_state " <<  game_state << ")";
            ss << "(penalty " << penalty << ")";
            ss << ")";
            return ss.str();
        }

        virtual bool FromMessage(const std::string& received)
        {
            librcsscontroller::MessageParser msg(received);

            // Opening parenthesis
            if (!msg.ReadOpen())
            {
                std::cerr << "Malformed packet: missing starting parenthesis\n";
                return false;
            }

            // game_state
            if (!msg.ReadKeyVal("game_state", &game_state))
            {
                std::cerr << "Malformed packet: error reading game_state\n";
                return false;
            }            

            // penalty
            if (!msg.ReadKeyVal("penalty", &penalty))
            {
                std::cerr << "Malformed packet: error reading penalty\n";
                return false;
            }        

            // Closing parenthesis
            if (!msg.ReadClose())
            {
                std::cerr << "Malformed packet: missing starting parenthesis\n";
                return false;
            }

            return true;
        }
    };
}

#endif // FINDBALLEXP_RUNSWIFTSERVERRESPONSE_H_