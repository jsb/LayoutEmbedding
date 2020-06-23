#pragma once

#include "VertexEdgePath.hh"
#include "RefinableMesh.hh"

struct Embedding
{
    pm::Mesh* l_m;      // Layout mesh
    RefinableMesh* t_m; // Target mesh
    pm::vertex_attribute<pm::vertex_handle> l_matching_vertex;
    pm::vertex_attribute<pm::vertex_handle> t_matching_vertex;
    pm::halfedge_attribute<pm::halfedge_handle> t_matching_halfedge;
};

Embedding make_embedding(pm::Mesh& _l_m, RefinableMesh& _rm);

void set_matching_vertices(Embedding& _e, std::vector<std::pair<pm::vertex_handle, pm::vertex_handle>> _mvs);

/// If the layout halfedge _l_h has an embedding, returns the target halfedge at the start of the corresponding embedded path.
/// Otherwise, returns an invalid halfedge.
pm::halfedge_handle get_embedded_target_halfedge(Embedding& _e, const pm::halfedge_handle& _l_he);

/// Returns the target halfedge representing the sector in which the layout halfedge _l_h can be currently embedded.
/// Returns an invalid halfedge if the layout halfedge is already embedded.
pm::halfedge_handle get_embeddable_sector(Embedding& _e, const pm::halfedge_handle& _l_he);

bool is_blocked(Embedding& _e, const pm::edge_handle& _t_e);
bool is_blocked(Embedding& _e, const pm::vertex_handle& _t_v);
bool is_blocked(Embedding& _e, const VertexEdgeElement& _t_el);

tg::pos3 element_pos(Embedding& _e, const pm::edge_handle& _t_e);
tg::pos3 element_pos(Embedding& _e, const pm::vertex_handle& _t_v);
tg::pos3 element_pos(Embedding& _e, const VertexEdgeElement& _t_el);

VertexEdgePath find_shortest_path(
    Embedding& _e,
    const pm::halfedge_handle& _t_h_sector_start, // Target halfedge, at the beginning of a sector
    const pm::halfedge_handle& _t_h_sector_end    // Target halfedge, at the beginning of a sector
);
VertexEdgePath find_shortest_path(
    Embedding& _e,
    const pm::halfedge_handle& _l_h // Layout halfedge
);

void insert_path(Embedding& _e, const pm::halfedge_handle& _l_h, const VertexEdgePath& _path);
