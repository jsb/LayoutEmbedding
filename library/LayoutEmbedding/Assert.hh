#pragma once

#include <iostream>

#define LE_ERROR(str) \
    std::cout \
    << "[ERROR] " \
    << str \
    << " (in function " << __FUNCTION__ << ", " << __FILE__ << ":" << __LINE__ << ")" \
    << std::endl

#define LE_ERROR_THROW(str) \
    {LE_ERROR(str); \
    throw std::runtime_error("ERROR");}


#define LE_ASSERT(exp) \
    {if (!(exp)) LE_ERROR_THROW("Assertion failed: " << (#exp));}
