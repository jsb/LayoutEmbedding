#include "Sector.hh"

bool is_valid(const Sector& _sec)
{
    if (_sec.start.is_invalid()) {
        return false;
    }
    else if (_sec.end.is_invalid()) {
        return true;
    }
    else {
        return (_sec.start.vertex_from() == _sec.end.vertex_from());
    }
}

Sector find_sector(const pm::halfedge_handle& _he, const CutGraph& _cg)
{
    Sector result;

    // Find start by rotating cw
    result.start = _he;
    while (!on_cut_graph(result.start, _cg)) {
        result.start = result.start.opposite().next(); // Rotate cw around vertex_from()
        if (result.start == _he) {
            // If we reach the initial halfedge again without encountering a cut graph, then this is a disk sector.
            result.end = pm::halfedge_handle::invalid;
            return result;
        }
    }

    // Find end by rotating ccw
    result.end = result.start;
    do {
        result.end = result.end.prev().opposite(); // Rotate ccw around vertex_from()
    }
    while (!on_cut_graph(result.end, _cg));

    return result;
}
