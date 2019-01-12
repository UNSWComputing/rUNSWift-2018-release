#ifndef SIMULATION_SIMULATIONCONNECTION_H_
#define SIMULATION_SIMULATIONCONNECTION_H_

#include "PerceptorInfo.hpp"
#include "EffectorCommand.hpp"

#include <boost/array.hpp>
#include <boost/asio.hpp>

#include <string>

namespace Simulation
{
    /*
     * The SimulationConnection class handles network IO with the simulator server.
     */
    class SimulationConnection
    {
    public:

        /*
         *  Constructor
         */
        SimulationConnection();

        /*
         *  Initialises connection to the server and places robot
         */
        bool init(const std::string& host, unsigned int port, const std::string& team_name, int player_num, double x_pos, double y_pos, double orientation);

        /*
         *  Sends effector command to the server
         */
        bool sendEffectorCommand(const EffectorCommand& cmd);

        /*
         *  Receives perceptor information from the serevr
         */
        bool receivePerceptorInfo(PerceptorInfo* info_out);

    private:
        bool sendString(const std::string&);

        boost::asio::io_service io_service;
        boost::asio::ip::tcp::socket socket;


    };
} // End namespace Simulation

#endif //SIMULATION_SIMULATIONCONNECTION_H_

