#include "Hash.hh"

namespace LayoutEmbedding {

HashValue hash_combine(HashValue _a, HashValue _b)
{
    // Taken from https://stackoverflow.com/a/2595226/3077540
    return _a ^ (_b + 0x9e3779b9 + (_a << 6) + (_a >> 2));
}

}
