
namespace librcsscontroller
{
    
    template<typename T>
    bool MessageParser::ReadType(T* out)
    {
        ss_ >> *out;
        return ss_.good();
    }

    template<typename T>
    bool MessageParser::ReadVal(T* out)
    {
        if (!ReadType(out))
        {
            return false;
        }

        if (!ReadClose())
        {
            return false;
        }
        return true;
    }

    template<typename T>
    bool MessageParser::ReadKeyVal(const std::string& expected_key, T* out)
    {
        if (!ReadKey(expected_key))
        {
            return false;
        }
        return ReadVal(out);
    }

}