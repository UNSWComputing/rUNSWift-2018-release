#include "ExperimentControllerConnection.hpp"
#include "ToRunswiftAgent.h"

#include <boost/lexical_cast.hpp>
#include <iostream>
#include <sstream>

using namespace findballexp;
namespace Simulation
{
    ExperimentControllerConnection::ExperimentControllerConnection(Blackboard* bb)
        : blackboard(bb), io_service_(), socket_(io_service_), connected_(false)
    { }

    ExperimentControllerConnection::~ExperimentControllerConnection()
    {
        boost::system::error_code ec;
        socket_.shutdown(boost::asio::ip::tcp::socket::shutdown_both, ec);
        socket_.close(ec);
    }

    bool ExperimentControllerConnection::init(const std::string& host, int port)
    {      
        try 
        {
            boost::asio::ip::tcp::resolver resolver(io_service_);
            boost::asio::ip::tcp::resolver::iterator endpoint = resolver.resolve(boost::asio::ip::tcp::resolver::query(host, boost::lexical_cast<std::string>(port)));
            socket_.open(boost::asio::ip::tcp::v4());
            socket_.set_option(boost::asio::ip::tcp::no_delay(true));
            boost::asio::connect(socket_, endpoint);
        }
        catch (const std::exception& e)
        {
            std::cerr << "Error connecting to ExperimentController: " << e.what() << "\n";
            return false;
        }

        std::cout << "Connected!\n";
        connected_ = true;
        sendString(buildUpdate().ToMessage());
        return true;
    }

    void ExperimentControllerConnection::tick()
    {
        if (!connected_)
        {
            return;
        }

        boost::asio::socket_base::bytes_readable command(true);
        socket_.io_control(command);
        if (command.get())
        {
            findballexp::FromRunswiftAgent update = buildUpdate();                        
            //std::cout << "Sending: " << update.ToMessage() << "\n";
            if (!sendString(update.ToMessage()))
            {
                std::cerr << "Error communicating with ExperimentController!\n";
            }

            std::string rec;
            receiveString(&rec);
            ToRunswiftAgent response;
            if (response.FromMessage(rec))
            {
                //std::cout << "Received: " << response.ToMessage() << "\n";
                RoboCupGameControlData data = readFrom(gameController, data);
                data.state = response.game_state;
                writeTo(gameController, data, data);
                writeTo(gameController, gameState, (uint8_t)response.game_state);

                TeamInfo info = readFrom(gameController, our_team);
                int player_num = readFrom(gameController, player_number);
                info.players[player_num-1].penalty = response.penalty;
                writeTo(gameController, our_team, info);
            }
        }
    }

    FromRunswiftAgent ExperimentControllerConnection::buildUpdate()
    {
        FromRunswiftAgent update;
        update.team_number = readFrom(receiver, team);

        // TODO do this better
        std::stringstream ss; 
        ss << "Team";
        ss << update.team_number;

        update.team_name = (update.team_number == 18 ? "runswift" : ss.str());
        update.player_number = readFrom(gameController, player_number);
        update.can_see_ball = readFrom(vision, balls).size();
        update.ball_seen_count = readFrom(localisation, ballSeenCount);
        update.ball_lost_count = readFrom(localisation, ballLostCount);
        AbsCoord pos = readFrom(localisation, robotPos);
        update.estimated_x_pos = pos.x();
        update.estimated_y_pos = pos.y();
        update.estimated_orientation = pos.theta();
        if (update.can_see_ball)
        {
            update.dist_from_ball = readFrom(vision, balls)[0].rr.distance();
        }
        return update;
    }


    bool ExperimentControllerConnection::sendString(const std::string& msg)
    {
        //std::cout << "\nSending: " << msg << "\n";
        unsigned int len = htonl(msg.size());
        std::string prefix((const char*)&len, sizeof(unsigned int));
        std::string to_send = prefix + msg;
        socket_.send(boost::asio::buffer(to_send));
        return true;
    }

    bool ExperimentControllerConnection::receiveString(std::string* out)
    {
        boost::system::error_code error;
        uint32_t len = 0;

        size_t recv = socket_.read_some(boost::asio::buffer((char*)&len, 4), error);
        if (error)
        {
            return false;
        }
        len = ntohl(len);

        std::vector<char> buf(len);
        recv = socket_.read_some(boost::asio::buffer(buf), error);
        if (error)
        {
            return false;
        }

        std::string msg (buf.data(), recv);
        if (len != recv)
        {
            return false;
        }

        *out = msg;
        return true;
     }


}