#pragma once

#include <polymesh/pm.hh>

using CutGraph = pm::edge_attribute<bool>;

bool on_cut_graph(const pm::edge_handle& _e, const CutGraph& _cg);
bool on_cut_graph(const pm::halfedge_handle& _he, const CutGraph& _cg);
bool on_cut_graph(const pm::vertex_handle& _v, const CutGraph& _cg);

CutGraph embed_cut_graph(const std::vector<pm::vertex_handle>& _nodes);
