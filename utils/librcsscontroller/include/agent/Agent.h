#ifndef LIBRCSSCONTROLLER_AGENT_H_
#define LIBRCSSCONTROLLER_AGENT_H_

namespace librcsscontroller
{
    struct Agent
    {
    public:
        Agent(int id)
            : id{id}
        { }

        int id;
    };

}

#endif // LIBRCSSCONTROLLER_AGENT_H_