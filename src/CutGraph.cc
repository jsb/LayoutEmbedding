#include "CutGraph.hh"

#include <cassert>

bool on_cut_graph(const pm::edge_handle& _e, const CutGraph& _cg)
{
    return _cg[_e];
}

bool on_cut_graph(const pm::halfedge_handle& _he, const CutGraph& _cg)
{
    return on_cut_graph(_he.edge(), _cg);
}

bool on_cut_graph(const pm::vertex_handle& _v, const CutGraph& _cg)
{
    for (const auto e : _v.edges()) {
        if (on_cut_graph(e, _cg)) {
            return true;
        }
    }
    return false;
}

CutGraph embed_cut_graph(const std::vector<polymesh::vertex_handle>& _nodes)
{
    assert(!_nodes.empty());
    const auto& m = *_nodes.front().mesh;
    CutGraph result;
    return result;
}
