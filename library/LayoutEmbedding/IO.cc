#include "IO.hh"

#include <cstring>

namespace LayoutEmbedding
{

bool flag(const char* _flag, int _argc, char** _argv)
{
    for (int i = 1; i < _argc; ++i)
    {
        if (strcmp(_argv[i], _flag) == 0)
            return true;
    }

    return false;
}

}
