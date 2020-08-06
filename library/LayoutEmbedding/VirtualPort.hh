#pragma once

#include <LayoutEmbedding/VirtualVertex.hh>

namespace LayoutEmbedding {

/// Represents an outgoing direction from a real vertex to a virtual vertex.
/// We can then use this to rotate the represented port by "virtual" steps around its origin
/// (alternatingly pointing to real / virtual vertices).
struct VirtualPort
{
    pm::vertex_handle from;
    VirtualVertex to;

    bool operator==(const VirtualPort& _rhs) const;
    bool operator!=(const VirtualPort& _rhs) const;

    VirtualPort rotated_cw() const;
    VirtualPort rotated_ccw() const;
};

}
