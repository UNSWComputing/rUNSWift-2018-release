#include "comms/MessageParser.h"

namespace librcsscontroller
{

    MessageParser::MessageParser(const std::string msg)
        : ss_(msg)
    { }

    MessageParser::MessageParser(const MessageParser& copy)
        : ss_(copy.ss_.str())
    { }

    std::string MessageParser::GetString() const
    {
        return ss_.str();
    }

    bool MessageParser::ReadType(std::string* out)
    {
        return ReadDelimitedWord(out);
    }

    bool MessageParser::ReadDelimitedWord(std::string* out)
    {
        std::string word;
        if (!ReadType<std::string>(&word))
        {
            return false;
        }

        // Can't really delimit a word of size 1
        if (word.size() <= 1)
        {
            return true;
        }

        // Find the first closing parenthesis (or the end)
        std::string::iterator end = std::find(word.begin()+1, word.end(), ')');
        
        // Find the last opening parenthesis before the first closing
        // parenthesis (or the start)
        std::reverse_iterator<std::string::iterator> start = 
            std::find(std::reverse_iterator<std::string::iterator>(end), 
                      word.rend(), '(');          

        // Get the delimited word
        std::string delimited_word(start.base(), end);

        // Put the rest of the word back in the stringstream
        if (word.end()-end)
        {
            std::string remaining;
            getline(ss_, remaining);
            remaining = std::string(end, word.end()) + remaining;
            ss_.str(remaining);
            ss_.clear();
        }
        
        // Output delimited word
        *out = delimited_word;
        return true;
    }

    bool MessageParser::ReadOpen()
    {
        char p;
        if (!ReadType(&p) || p != '(')
        {
            return false;
        }
        return true;
    }

    bool MessageParser::ReadClose()
    {
        char p;
        if (!ReadType(&p) || p != ')')
        {
            return false;
        }
        return true;
    }

    bool MessageParser::ReadKey(const std::string& expected)
    {
        if (!ReadOpen())
        {
            return false;
        }

        std::string read;
        if (!ReadType(&read))
        {
            return false;
        }
        return (read == expected);
    }

    bool MessageParser::IsGood()
    {
        return static_cast<bool>(ss_);
    }

}