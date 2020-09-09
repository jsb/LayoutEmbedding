#pragma once

#include <LayoutEmbedding/VirtualVertex.hh>

namespace LayoutEmbedding {

/// Represents an outgoing direction from a real vertex to a virtual vertex.
/// We can then use this to rotate the represented port by "virtual" steps around its origin
/// (alternatingly pointing to real / virtual vertices).
struct VirtualPort
{
    VirtualPort() = default;
    VirtualPort(const pm::vertex_handle& _from, const VirtualVertex& _to);
    explicit VirtualPort(const pm::halfedge_handle& _real_he);

    bool operator==(const VirtualPort& _rhs) const;
    bool operator!=(const VirtualPort& _rhs) const;
    bool operator<(const VirtualPort& _rhs) const; // Provided so this can be used as a map key

    VirtualPort rotated_cw() const;
    VirtualPort rotated_ccw() const;

    // Warning: Fails if 'to' is not a real vertex
    pm::halfedge_handle real_halfedge() const;

    bool is_valid() const;

    pm::vertex_handle from;
    VirtualVertex to;
};

}
