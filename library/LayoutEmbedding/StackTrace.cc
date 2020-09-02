#include "StackTrace.hh"

#include <csignal>
#include <iostream>

#if defined(__GLIBCXX__) || defined(__GLIBCPP__)
// GCC: Implement demangling using cxxabi
#include <cxxabi.h>
std::string demangle(const std::string& _symbol)
{
    int status;
    char* demangled = abi::__cxa_demangle(_symbol.c_str(), nullptr, nullptr, &status);
    if (demangled) {
        std::string result{demangled};
        free(demangled);
        if (status == 0) {
            return result;
        }
        else {
            return _symbol;
        }
    }
    else {
        return _symbol;
    }
}
#else
// Other compiler environment: no demangling
std::string demangle(const std::string& _symbol)
{
    return _symbol;
}
#endif

#ifdef __unix__
#include <execinfo.h>
#include <regex>
void stack_trace()
{
    void *addresses[20];
    char **strings;

    int size = backtrace(addresses, 20);
    strings = backtrace_symbols(addresses, size);
    std::cerr << "Stack frames: " << size << std::endl;
    // line format:
    // <path>(<mangled_name>+<offset>) [<address>]
    std::regex line_format{R"(^\s*(.+)\((([^()]+)?\+(0x[0-9a-f]+))?\)\s+\[(0x[0-9a-f]+)\]\s*$)"};
    for(int i = 0; i < size; i++) {
        std::string line{strings[i]};
        std::smatch match;
        std::regex_match(line, match, line_format);
        if (!match.empty()) {
            auto file_name = match[1].str();
            auto symbol = demangle(match[3].str());
            auto offset = match[4].str();
            auto address = match[5].str();
            std::cerr << i << ":";
            if (!file_name.empty()) std::cerr << " " << file_name << " ::";
            if (!symbol.empty()) std::cerr << " " << symbol;
            if (!offset.empty()) std::cerr << " (+" << offset << ")";
            if (!address.empty()) std::cerr << " [" << address << "]";
            std::cerr << std::endl;
        }
    }
    free(strings);
}
#else
void stack_trace()
{
}
#endif

namespace LayoutEmbedding {

void handle_segfault(int)
{
    // Prevent infinite recursion
    std::signal(SIGSEGV, SIG_DFL);
    stack_trace();
    std::abort();
}

}
