#pragma once

#include <LayoutEmbedding/Util/StackTrace.hh>

#include <cmath>
#include <iostream>

#define LE_ERROR(str) \
    std::cout \
    << "[ERROR] " \
    << str \
    << " (in function " << __FUNCTION__ << ", " << __FILE__ << ":" << __LINE__ << ")" \
    << std::endl

#define LE_ERROR_THROW(str) \
    {LE_ERROR(str); \
    print_stack_trace(); \
    throw std::runtime_error("ERROR");}

#define LE_ASSERT(exp) \
    {if (!(exp)) LE_ERROR_THROW("Assertion failed: " #exp);}

#define LE_ASSERT_MSG(exp, msg) \
    {if (!(exp)) LE_ERROR_THROW("Assertion failed: " #exp ". " << msg);}

#define LE_ASSERT_EQ(a, b) \
    {if (!((a) == (b))) LE_ERROR_THROW("Assertion failed: " #a " == " #b "\n" \
                                       "    " #a " = " << (a) << "\n" \
                                       "    " #b " = " << (b) << "\n");}

#define LE_ASSERT_NEQ(a, b) \
    {if (!((a) != (b))) LE_ERROR_THROW("Assertion failed: " #a " != " #b "\n" \
                                       "    " #a " = " << (a) << "\n" \
                                       "    " #b " = " << (b) << "\n");}

#define LE_ASSERT_G(a, b) \
    {if (!((a) > (b))) LE_ERROR_THROW("Assertion failed: " #a " > " #b "\n" \
                                      "    " #a " = " << (a) << "\n" \
                                      "    " #b " = " << (b) << "\n");}

#define LE_ASSERT_GEQ(a, b) \
    {if (!((a) >= (b))) LE_ERROR_THROW("Assertion failed: " #a " >= " #b "\n" \
                                       "    " #a " = " << (a) << "\n" \
                                       "    " #b " = " << (b) << "\n");}

#define LE_ASSERT_L(a, b) \
    {if (!((a) < (b))) LE_ERROR_THROW("Assertion failed: " #a " < " #b "\n" \
                                      "    " #a " = " << (a) << "\n" \
                                      "    " #b " = " << (b) << "\n");}

#define LE_ASSERT_LEQ(a, b) \
    {if (!((a) <= (b))) LE_ERROR_THROW("Assertion failed: " #a " <= " #b "\n" \
                                       "    " #a " = " << (a) << "\n" \
                                       "    " #b " = " << (b) << "\n");}

#define LE_ASSERT_EPS(a, b, eps) \
    {if (std::abs((a) - (b)) >= (eps)) LE_ERROR_THROW("Assertion failed: |" #a " - " #b "| < " #eps "\n" \
                                                      "    " #a " = " << (a) << "\n" \
                                                      "    " #b " = " << (b) << "\n" \
                                                      "    |" #a " - " #b "| = " << std::abs((a) - (b)) << "\n");}
