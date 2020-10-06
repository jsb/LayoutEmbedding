#pragma once

#include <iostream>

#define LE_DEBUG_OUT(msg) \
    {std::cout << "[DEBUG] " << msg << std::endl;}

#define LE_DEBUG_VAR(var) \
    LE_DEBUG_OUT(#var " = " << (var))
