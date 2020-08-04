#pragma once

#include <LayoutEmbedding/LayoutGeneration.hh>
#include <LayoutEmbedding/VirtualVertex.hh>
#include <LayoutEmbedding/VirtualPath.hh>

namespace LayoutEmbedding {

struct Embedding
{
    const pm::Mesh* l_m; // Layout mesh
    pm::Mesh* t_m; // Target mesh
    pm::vertex_attribute<tg::pos3>* t_pos;

    pm::vertex_attribute<pm::vertex_handle> l_matching_vertex;
    pm::vertex_attribute<pm::vertex_handle> t_matching_vertex;
    pm::halfedge_attribute<pm::halfedge_handle> t_matching_halfedge;
};

Embedding make_embedding(const pm::Mesh& _l_m, pm::Mesh& _t_m, pm::vertex_attribute<tg::pos3>& _t_pos);

void set_matching_vertices(Embedding& _e, const MatchingVertices& _mvs);

/// If the layout halfedge _l_h has an embedding, returns the target halfedge at the start of the corresponding embedded path.
/// Otherwise, returns an invalid halfedge.
pm::halfedge_handle get_embedded_target_halfedge(const Embedding& _e, const pm::halfedge_handle& _l_he);

bool is_embedded(const Embedding& _e, const pm::halfedge_handle& _l_he);
bool is_embedded(const Embedding& _e, const pm::edge_handle& _l_e);

/// Returns the target halfedge representing the sector in which the layout halfedge _l_h can be currently embedded.
/// Returns an invalid halfedge if the layout halfedge is already embedded.
pm::halfedge_handle get_embeddable_sector(const Embedding& _e, const pm::halfedge_handle& _l_he);

bool is_blocked(const Embedding& _e, const pm::edge_handle& _t_e);
bool is_blocked(const Embedding& _e, const pm::vertex_handle& _t_v);
bool is_blocked(const Embedding& _e, const VirtualVertex& _t_vv);

tg::pos3 element_pos(const Embedding& _e, const pm::edge_handle& _t_e);
tg::pos3 element_pos(const Embedding& _e, const pm::vertex_handle& _t_v);
tg::pos3 element_pos(const Embedding& _e, const VirtualVertex& _t_vv);

VirtualPath find_shortest_path(
    const Embedding& _e,
    const pm::halfedge_handle& _t_h_sector_start, // Target halfedge, at the beginning of a sector
    const pm::halfedge_handle& _t_h_sector_end    // Target halfedge, at the beginning of a sector
);
VirtualPath find_shortest_path(
    const Embedding& _e,
    const pm::halfedge_handle& _l_he // Layout halfedge
);
VirtualPath find_shortest_path(
    const Embedding& _e,
    const pm::edge_handle& _l_e // Layout edge
);

void embed_path(Embedding& _e, const pm::halfedge_handle& _l_h, const VirtualPath& _path);
void unembed_path(Embedding& _e, const pm::halfedge_handle& _l_h);
void unembed_path(Embedding& _e, const pm::edge_handle& _l_e);

std::vector<pm::vertex_handle> get_embedded_path(const Embedding& _e, const pm::halfedge_handle& _l_he);

double path_length(const Embedding& _e, const VirtualPath& _path);

double embedded_path_length(const Embedding& _e, const pm::halfedge_handle& _l_he);
double embedded_path_length(const Embedding& _e, const pm::edge_handle& _l_e);
double total_embedded_path_length(const Embedding& _e);

}
