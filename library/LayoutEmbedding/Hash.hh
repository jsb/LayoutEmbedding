#pragma once

#include <functional>

#include <typed-geometry/functions/std/hash.hh>

namespace LayoutEmbedding {

using HashValue = std::size_t;

HashValue hash_combine(HashValue _a, HashValue _b);

template<typename T>
HashValue hash(const T& _x)
{
    return std::hash<T>()(_x);
}

}
