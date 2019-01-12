#ifndef SIMULATION_EXPERIMENTCONTROLLERCONNECTION_H_
#define SIMULATION_EXPERIMENTCONTROLLERCONNECTION_H_

#include "comms/MessageParser.h"
#include "FromRunswiftAgent.h"

#include <blackboard/Blackboard.hpp>
#include <boost/array.hpp>
#include <boost/asio.hpp>


namespace Simulation
{
    /*
     * The ExperimentControllerConnection class handles network IO with an 
     * experiment controller, if there is one running.
     *
     * An experiment controller is used to run experiments on the simulator.
     * During experiments, it controls the simulator (by moving the ball,
     * robots, etc) and agents (rUNSWift) (by penalising, changing game state, 
     * etc) and record metrics about the game (e.g. ball searching speed).
     *
     * This was primarily used for Collette's (2017) thesis, but can be used
     * for other experiments. There is no experiment controller included with
     * rUNSWift, but starter code from Collette ('librcsscontroller') and 
     * examples ('findballexp') should be available online.
     */
    class ExperimentControllerConnection
    {
    public:
        /**
         *  Constructor
         *
         *  @param bb A pointer to the current Blackboard instance in use.
         */
        ExperimentControllerConnection(Blackboard* bb);

        /**
         *  Deconstructor
         */
        ~ExperimentControllerConnection();

        /**
         *  Initialises the ExperimentControllerConnection
         *
         *  @param host The host of the experiment controller to connect to
         *  @param port The port of the experiment controller to connect to
         *  @return bool True indicates success. False indicates failure.
         */
        bool init(const std::string& host, int port);

        /**
         *  Attempts to send agent information to the experiment controller and
         *  receive an update in response.
         */
        void tick();

    private:        
        /**< Builds an agent update to send to the experiment controller */
        findballexp::FromRunswiftAgent buildUpdate();

        /**< Sends a string to the experiment controller */
        bool sendString(const std::string& msg);
        
        /**< Receives a string from the experiment controller */
        bool receiveString(std::string* out);

        Blackboard* blackboard; /**< Current blackboard instance */
        boost::asio::io_service io_service_;    /**< Boost TCP stuff */
        boost::asio::ip::tcp::socket socket_;   /**< Boost TCP socker */
        bool connected_;                        /**< Indicates connectivity */
    };

}

#endif // SIMULATION_EXPERIMENTCONTROLLERCONNECTION_H_