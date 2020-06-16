#pragma once

#include "CutGraph.hh"

#include <polymesh/pm.hh>
#include <polymesh/std/hash.hh>

// It is always either
// start.vertex_from() == end.vertex_from()
// or
// end.is_invalid().
//
// Describes a contiguous sector of faces around a vertex, delineated by a cut graph.
// The sector starts at start (inclusive) and goes ccw to end (exclusive).
// If start == end, then start.edge() is excluded from the sector.
// If end.is_invalid(), then this describes the entire disk around start.vertex_from().
// The empty sector is not representable.
struct Sector
{
    pm::halfedge_handle start;
    pm::halfedge_handle end;

    bool operator==(const Sector& _rhs) const noexcept
    {
        return (start == _rhs.start) && (end == _rhs.end);
    }
};

namespace std
{
    template<>
    struct hash<Sector>
    {
        std::size_t operator()(const Sector& _sec) const noexcept
        {
            std::size_t h1 = std::hash<pm::halfedge_handle>()(_sec.start);
            std::size_t h2 = std::hash<pm::halfedge_handle>()(_sec.end);
            return h1 ^ (h2 << 1);
        }
    };
}

bool is_valid(const Sector& _sec);

Sector find_sector(const pm::halfedge_handle& _he, const CutGraph& _cg);
