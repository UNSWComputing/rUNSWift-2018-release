#ifndef FINDBALLEXP_RUNSWIFTAGENTUPDATE_H_
#define FINDBALLEXP_RUNSWIFTAGENTUPDATE_H_

#include "agent/FromAgent.h"
#include "comms/MessageParser.h"

#include <iostream>
#include <string>

namespace findballexp
{
    struct FromRunswiftAgent 
        : public librcsscontroller::FromAgent
    {
        int team_number;
        std::string team_name;
        int player_number;
        bool can_see_ball;      
        int ball_seen_count;    // number of consecutive frames the ball has been seen
        int ball_lost_count;    // number of consecutive frames the ball has not been seen
        float estimated_x_pos;    // from localisation
        float estimated_y_pos;    // from localisation
        float estimated_orientation; // from localisation
        float dist_from_ball;     // in mm

        FromRunswiftAgent()
            : team_number(-1), team_name("none"), player_number(-1), can_see_ball(false),
            ball_seen_count(-1), ball_lost_count(-1), estimated_x_pos(-1), estimated_y_pos(-1),
            estimated_orientation(-1), dist_from_ball(-1)

        { }

        std::string ToMessage() const
        {
            std::stringstream ret;
            ret << "("
                << "(team_number " << team_number << ")"
                << "(team_name " << team_name << ")"
                << "(player_number " << player_number << ")"
                << "(can_see_ball " << (can_see_ball ? "1" : "0") << ")"
                << "(ball_seen_count " << ball_seen_count << ")"
                << "(ball_lost_count " << ball_lost_count << ")"
                << "(estimated_x_pos " << estimated_x_pos << ")"
                << "(estimated_y_pos " << estimated_y_pos << ")"
                << "(estimated_orientation " << estimated_orientation << ")"
                << "(dist_from_ball " << dist_from_ball << ")"
                << ")";
            return ret.str();
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

            // team_number
            if (!msg.ReadKeyVal("team_number", &team_number))
            {
                std::cerr << "Malformed packet: error reading team_number\n";
                return false;
            }

            // team_name
            if (!msg.ReadKeyVal("team_name", &team_name))
            {
                std::cerr << "Malformed packet: error reading team_name\n";
                return false;
            }

            // player_number
            if (!msg.ReadKeyVal("player_number", &player_number))
            {
                std::cerr << "Malformed packet: error reading player_number\n";
                return false;
            }

            // can_see_ball
            if (!msg.ReadKeyVal("can_see_ball", &can_see_ball))
            {
                std::cerr << "Malformed packet: error reading can_see_ball\n";
                return false;
            }

            // ball_seen_count
            if (!msg.ReadKeyVal("ball_seen_count", &ball_seen_count))
            {
                std::cerr << "Malformed packet: error reading ball_seen_count\n";
                return false;
            }

            // ball_lost_count
            if (!msg.ReadKeyVal("ball_lost_count", &ball_lost_count))
            {
                std::cerr << "Malformed packet: error reading ball_lost_count\n";
                return false;
            }

            // estimated_x_pos
            if (!msg.ReadKeyVal("estimated_x_pos", &estimated_x_pos))
            {
                std::cerr << "Malformed packet: error reading estimated_x_pos\n";
                return false;
            }

            // estimated_y_pos
            if (!msg.ReadKeyVal("estimated_y_pos", &estimated_y_pos))
            {
                std::cerr << "Malformed packet: error reading estimated_y_pos\n";
                return false;
            }

            // estimated_orientation
            if (!msg.ReadKeyVal("estimated_orientation", &estimated_orientation))
            {
                std::cerr << "Malformed packet: error reading estimated_orientation\n";
                return false;
            }

            // dist_from_ball
            if (!msg.ReadKeyVal("dist_from_ball", &dist_from_ball))
            {
                std::cerr << "Malformed packet: error reading dist_from_ball\n";
                return false;
            }

            // Closing parenthesis
            if (!msg.ReadClose())
            {
                std::cerr << "Malformed packet: error reading closing parenthesis\n";
                std::string next;
                msg.ReadVal<std::string>(&next);
                std::cerr << "Next token: \"" << next << "\"\n";
                return false;
            }
            return true;
        }
    };
}

#endif // FINDBALLEXP_RUNSWIFTAGENTUPDATE_H_