#include "SimulationConnection.hpp"

#include <boost/thread/thread.hpp>

#include <iomanip>
#include <sstream>
#include <vector>

// TODO debug
#include <iostream>

namespace Simulation
{

    SimulationConnection::SimulationConnection()
        : io_service(), socket(io_service)
    {

    }

    bool SimulationConnection::init(const std::string& host, unsigned int port, const std::string& team_name, int player_num, double x_pos, double y_pos, double orientation)
    {
        boost::asio::ip::tcp::resolver resolver(io_service);
        boost::asio::ip::tcp::resolver::iterator endpoint = resolver.resolve(boost::asio::ip::tcp::resolver::query(host, boost::lexical_cast<std::string>(port)));
        boost::asio::connect(socket, endpoint);

        if (!sendString("(scene rsg/agent/nao/nao.rsg)"))
        {
            return false;
        }

        // If we don't wait, the server drops our next packet
        PerceptorInfo info;
        for (int i=0; i < 5; ++i)
        {
            receivePerceptorInfo(&info);
        }

        std::stringstream ss;
        ss << "(init (unum " << player_num << ")(teamname " << team_name << "))";
        if (!sendString(ss.str()))
        {
            return false;
        }

        // If we don't wait, the server drops our next packet
        for (int i=0; i < 5; ++i)
        {
            receivePerceptorInfo(&info);
        }

        ss.str(std::string());
        ss.clear();    
        ss << "(beam " << x_pos << " " << y_pos << " " << orientation << ")";
        if (!sendString(ss.str()))
        {
            return false;
        }

        // Let's wait for the robot to spawn and joints settle before
        // we proceed
        for (int i=0; i < 2000/20; ++i) // we wait 20ms each loop so divide 2 seconds by 10ms
        {
            // We don't actually care about the perceptor info at the moment,
            // we just read it to stop the server getting all backed up yo
            // (otherwise we will be getting info 2 seconds late)
            if (!receivePerceptorInfo(&info))
            {
                return false;
            }

            // Don't need to sleep because server I/O blocks
        }

        return true;
    }

    bool SimulationConnection::sendEffectorCommand(const EffectorCommand &cmd)
    {
        std::ostringstream ss;
        ss << cmd.joints;
        return sendString(ss.str());
    }

    bool SimulationConnection::sendString(const std::string& msg)
    {
        //std::cout << "\nSending: " << msg << "\n";
        unsigned int len = htonl(msg.size());
        std::string prefix((const char*)&len, sizeof(unsigned int));
        std::string to_send = prefix + msg;
        socket.send(boost::asio::buffer(to_send));
        return true;
    }

    bool SimulationConnection::receivePerceptorInfo(PerceptorInfo* info_out)
    {
        boost::system::error_code error;
        uint32_t len = 0;

        size_t recv = socket.read_some(boost::asio::buffer((char*)&len, 4), error);
        if (error)
        {
            return false;
        }
        len = ntohl(len);

        std::vector<char> buf(len);
        recv = socket.read_some(boost::asio::buffer(buf), error);
        if (error)
        {
            return false;
        }

        std::string msg (buf.data(), recv);
        if (len != recv)
        {
            return false;
        }

        //std::cout << "\nReceived: " << msg << "\n";
        return info_out->fromString(msg);
     }

} // End namespace Simulation

