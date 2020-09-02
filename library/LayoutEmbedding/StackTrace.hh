#pragma once

#include <csignal>
#include <iostream>

namespace LayoutEmbedding {

void handle_segfault(int);

struct RegisterSegfaultHandler
{
    RegisterSegfaultHandler()
    {
        std::signal(SIGSEGV, handle_segfault);
    }
};
static RegisterSegfaultHandler register_segfault_handler;

}
