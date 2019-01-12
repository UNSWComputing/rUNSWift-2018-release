#ifndef LIBRCSSCONTROLLER_TOAGENT_H_
#define LIBRCSSCONTROLLER_TOAGENT_H_

#include "comms/MessageParser.h"

namespace librcsscontroller 
{
    /**
     *  The ToAgent struct is an abstract base struct that must be inherited
     *  by types that are to be sent from AgentServer instances to external
     *  agents.
     */
    struct ToAgent        
    {
        /**
         *  Converts the inherited type to a string to be transmitted via TCP
         *  to an agent. Essentially the reverse of the FromMessage() 
         *  function.
         *
         *  @return std::string The inherited type represented as a consumable
         *  string.
         */
        virtual std::string ToMessage() const = 0;

        /**
         *  Populates the inherited type using a string representation, most
         *  likely transmitted over TCP from an agent. The reverse of the 
         *  ToMessage() function.
         *
         *  @param msg The received message to populate the struct with.
         *  @return bool True if the struct was successfully populated,
         *  false otherwise.
         */
        virtual bool FromMessage(const std::string& received) = 0;
    };

}

#endif // LIBRCSSCONTROLLER_TOAGENT_H_