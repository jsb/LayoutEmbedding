#pragma once

#include <LayoutEmbedding/Hash.hh>
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

    VirtualPort rotated_cw() const;
    VirtualPort rotated_ccw() const;

    // Warning: Fails if 'to' is not a real vertex
    pm::halfedge_handle real_halfedge() const;

    bool is_valid() const;

    pm::vertex_handle from;
    VirtualVertex to;
};

}

namespace std
{
    template<> struct hash<LayoutEmbedding::VirtualPort>
    {
        std::size_t operator()(const LayoutEmbedding::VirtualPort& _x) const noexcept
        {
            using namespace LayoutEmbedding;
            std::size_t result = 0;
            result = hash_combine(result, _x.from.idx.value);
            result = hash_combine(result, _x.to.index());
            if (is_real_vertex(_x.to)) {
                result = hash_combine(result, real_vertex(_x.to).value);
            }
            else {
                result = hash_combine(result, real_edge(_x.to).value);
            }
            return result;
        }
    };
}

