#ifndef LIBRCSSCONTROLLER_MESSAGEPARSER_H_
#define LIBRCSSCONTROLLER_MESSAGEPARSER_H_

#include <algorithm>
#include <iostream>
#include <iterator>
#include <string>
#include <sstream>

namespace librcsscontroller
{
    /**
     *  MessageParser assists the parsing of strings sent over the network.
     *  It is particularly handy for decoding symbolic expressions.
     */
    class MessageParser
    {
    public:
        /** 
         *  Constructor
         *  
         *  @param msg The string message to parse.
         */
        MessageParser(const std::string msg);

        /** 
         *  Copy constructor
         *  
         *  @param copy The MessageParser to copy.
         */
        MessageParser(const MessageParser& copy);

        /**
         *  Returns the underlying string being parsed.
         *
         *  @return std::string The underlying string being parsed.
         */
        std::string GetString() const;

        /**
         *  Attempts to read a specific type from the string.
         *
         *  @tparam T The type to read.
         *  @param out[out] The location to save the type to.
         *  @return bool Returns true if the type was successfully read.
         *  Otherwise returns false.
         */
        template<typename T> bool ReadType(T* out);

        /**
         *  Attempts to read a delimited string from the string.
         * 
         *  @param out[out] The string to write the delimited string to.
         *  @return bool Returns true if the string was successfully read.
         *  False otherwise.
         */
        bool ReadType(std::string* out);

        /**
         *  Attempts to read a delimited string from the string.
         * 
         *  @param out[out] The string to write the delimited string to.
         *  @return bool Returns true if the string was successfully read.
         *  False otherwise.
         */
        bool ReadDelimitedWord(std::string* out);

        /**
         *  Attempts to read a symbolic expression openning '('.
         *
         *  @return bool True if an openning '(' was read. False otherwise.
         */
        bool ReadOpen();

        /**
         *  Attempts to read a symbolic expression closing ')'.
         *
         *  @return bool True if an openning ')' was read. False otherwise.
         */        
        bool ReadClose();

        /**
         *  Attempts to read an expected symbolic expression key.
         *
         *  @param expected The expected key to be read.
         *  @return bool True if the expected key was read. False otherwise.
         */        
        bool ReadKey(const std::string& expected);

        /**
         *  Attempts to read a symbolic expression value.
         *
         *  @tparam T The expected type of the value.
         *  @param out[out] A pointer where the read value is stored.
         *  @return bool True if the value was read. False otherwise.
         */        
        template<typename T> bool ReadVal(T* out);

        /**
         *  Attempts to read a symbolic expression key and value pair.
         *
         *  @tparam T The expected type of the value.
         *  @param expected_key The expected key to be read.
         *  @param out[out] A pointer where the read value is stored.
         *  @return bool True if the expected key and value was read. False
         *  otherwise.
         */        
        template<typename T>
        bool ReadKeyVal(const std::string& expected_key, T* out);

        /**
         *  Indicates whether or not the MessageParser can continue reading.
         *
         *  @return True if the MessageParser can continue reading. False
         *  otherwise.
         */ 
        bool IsGood();

    private:
        /**< Underlying stringstream instance to process the string */
        std::stringstream ss_;  

    };
}

#include "MessageParser.tcc"

#endif // LIBRCSSCONTROLLER_MESSAGEPARSER_H_        