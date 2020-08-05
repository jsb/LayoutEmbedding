#pragma once

#include <LayoutEmbedding/LayoutGeneration.hh>
#include <LayoutEmbedding/VirtualVertex.hh>
#include <LayoutEmbedding/VirtualPath.hh>

namespace LayoutEmbedding {

class Embedding
{
public:
    explicit Embedding(const pm::Mesh& _l_m, const pm::Mesh& _t_m, const pm::vertex_attribute<tg::pos3>& _t_pos);
    explicit Embedding(const Embedding& _em);

    void set_matching_vertices(const MatchingVertices& _mvs);

    /// If the layout halfedge _l_h has an embedding, returns the target halfedge at the start of the corresponding embedded path.
    /// Otherwise, returns an invalid halfedge.
    pm::halfedge_handle get_embedded_target_halfedge(const pm::halfedge_handle& _l_he) const;

    bool is_embedded(const pm::halfedge_handle& _l_he) const;
    bool is_embedded(const pm::edge_handle& _l_e) const;

    /// Returns the target halfedge representing the sector in which the layout halfedge _l_h can be currently embedded.
    /// Returns an invalid halfedge if the layout halfedge is already embedded.
    pm::halfedge_handle get_embeddable_sector(const pm::halfedge_handle& _l_he) const;

    bool is_blocked(const pm::edge_handle& _t_e) const;
    bool is_blocked(const pm::vertex_handle& _t_v) const;
    bool is_blocked(const VirtualVertex& _t_vv) const;

    tg::pos3 element_pos(const pm::edge_handle& _t_e) const;
    tg::pos3 element_pos(const pm::vertex_handle& _t_v) const;
    tg::pos3 element_pos(const VirtualVertex& _t_vv) const;

    VirtualPath find_shortest_path(
        const pm::halfedge_handle& _t_h_sector_start, // Target halfedge, at the beginning of a sector
        const pm::halfedge_handle& _t_h_sector_end    // Target halfedge, at the beginning of a sector
    ) const;
    VirtualPath find_shortest_path(
        const pm::halfedge_handle& _l_he // Layout halfedge
    ) const;
    VirtualPath find_shortest_path(
        const pm::edge_handle& _l_e // Layout edge
    ) const;

    double path_length(const VirtualPath& _path) const;

    void embed_path(const pm::halfedge_handle& _l_he, const VirtualPath& _path);
    void unembed_path(const pm::halfedge_handle& _l_he);
    void unembed_path(const pm::edge_handle& _l_e);

    std::vector<pm::vertex_handle> get_embedded_path(const pm::halfedge_handle& _l_he) const;
    double embedded_path_length(const pm::halfedge_handle& _l_he) const;
    double embedded_path_length(const pm::edge_handle& _l_e) const;
    double total_embedded_path_length() const;

    const pm::Mesh& layout_mesh() const
    {
        return *l_m;
    }

    const pm::Mesh& target_mesh() const
    {
        return t_m;
    }

    const pm::vertex_attribute<tg::pos3>& target_pos() const
    {
        return t_pos;
    }

    const pm::vertex_handle matching_target_vertex(const pm::vertex_handle& _l_v) const;
    const pm::vertex_handle matching_layout_vertex(const pm::vertex_handle& _t_v) const;

private:
    const pm::Mesh* l_m; // Layout mesh. Pointer.
    pm::Mesh t_m; // Target mesh. Copy.
    pm::vertex_attribute<tg::pos3> t_pos; // Target mesh positions. Copy.

    pm::vertex_attribute<pm::vertex_handle> l_matching_vertex;
    pm::vertex_attribute<pm::vertex_handle> t_matching_vertex;
    pm::halfedge_attribute<pm::halfedge_handle> t_matching_halfedge;
};

}
