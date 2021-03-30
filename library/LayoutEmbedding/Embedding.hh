#pragma once

#include <LayoutEmbedding/EmbeddingInput.hh>
#include <LayoutEmbedding/LayoutGeneration.hh>
#include <LayoutEmbedding/VirtualVertex.hh>
#include <LayoutEmbedding/VirtualPath.hh>
#include <polymesh/formats/obj.hh>

#include <Eigen/Dense>

#include <optional>

namespace LayoutEmbedding {

struct Snake;

class Embedding
{
public:
    explicit Embedding(EmbeddingInput& _input);

    Embedding(const Embedding& _em);
    Embedding& operator=(const Embedding& _em);

    /// If the layout halfedge _l_h has an embedding, returns the target halfedge at the start of the corresponding embedded path.
    /// Otherwise, returns an invalid halfedge.
    pm::halfedge_handle get_embedded_target_halfedge(const pm::halfedge_handle& _l_he) const;

    bool is_embedded(const pm::halfedge_handle& _l_he) const;
    bool is_embedded(const pm::edge_handle& _l_e) const;
    bool is_embedded(const pm::edge_index& _l_e) const;

    /// Returns the target halfedge representing the sector in which the layout halfedge _l_h can be currently embedded.
    /// Returns an invalid halfedge if the layout halfedge is already embedded.
    pm::halfedge_handle get_embeddable_sector(const pm::halfedge_handle& _l_he) const;

    bool is_blocked(const pm::edge_handle& _t_e) const;
    bool is_blocked(const pm::vertex_handle& _t_v) const;
    bool is_blocked(const VirtualVertex& _t_vv) const;

    tg::pos3 element_pos(const pm::edge_handle& _t_e) const;
    tg::pos3 element_pos(const pm::vertex_handle& _t_v) const;
    tg::pos3 element_pos(const VirtualVertex& _t_vv) const;

    enum class ShortestPathMetric
    {
        Geodesic,
        VertexRepulsive,
    };

    VirtualPath find_shortest_path(
        const pm::halfedge_handle& _t_h_sector_start, // Target halfedge, at the beginning of a sector
        const pm::halfedge_handle& _t_h_sector_end,   // Target halfedge, at the beginning of a sector
        ShortestPathMetric _metric = ShortestPathMetric::Geodesic
    ) const;
    VirtualPath find_shortest_path(
        const pm::halfedge_handle& _l_he, // Layout halfedge
        ShortestPathMetric _metric = ShortestPathMetric::Geodesic
    ) const;
    VirtualPath find_shortest_path(
        const pm::edge_handle& _l_e, // Layout edge
        ShortestPathMetric _metric = ShortestPathMetric::Geodesic
    ) const;

    double path_length(const VirtualPath& _path) const;

    void embed_path(const pm::halfedge_handle& _l_he, const VirtualPath& _path);
    void embed_path(const pm::halfedge_handle& _l_he, const Snake& _snake);
    void unembed_path(const pm::halfedge_handle& _l_he);
    void unembed_path(const pm::edge_handle& _l_e);

    std::vector<pm::vertex_handle> get_embedded_path(const pm::halfedge_handle& _l_he) const;
    std::vector<pm::face_handle> get_patch(const pm::face_handle& _l_f) const;
    double embedded_path_length(const pm::halfedge_handle& _l_he) const;
    double embedded_path_length(const pm::edge_handle& _l_e) const;
    double total_embedded_path_length() const;
    bool is_complete() const;

    bool save(std::string filename, bool write_target_mesh=true,
              bool write_layout_mesh=true, bool write_target_input_mesh=true) const;

    bool load(std::string filename);

    // Getters.
    const pm::Mesh& layout_mesh() const; // This will always refer to the original l_m in the input
    pm::Mesh& layout_mesh(); // This will always refer to the original l_m in the input
    const pm::vertex_attribute<tg::pos3>& layout_pos() const;
    pm::vertex_attribute<tg::pos3>& layout_pos();
    const pm::Mesh& target_mesh() const; // This refers to the local copy contained in this Embedding (can be different from the original target mesh due to local refinements).
    pm::Mesh& target_mesh();
    const pm::vertex_attribute<tg::pos3>& target_pos() const;
    pm::vertex_attribute<tg::pos3>& target_pos();
    const pm::vertex_handle& matching_target_vertex(const pm::vertex_handle& _l_v) const;
    pm::vertex_handle& matching_target_vertex(const pm::vertex_handle& _l_v);
    const pm::vertex_handle& matching_layout_vertex(const pm::vertex_handle& _t_v) const;
    pm::vertex_handle& matching_layout_vertex(const pm::vertex_handle& _t_v);
    const pm::halfedge_handle& matching_layout_halfedge(const pm::halfedge_handle& _t_h) const;
    pm::halfedge_handle& matching_layout_halfedge(const pm::halfedge_handle& _t_h);

    double get_vertex_repulsive_energy(const pm::vertex_handle& _t_v, const pm::vertex_handle& _l_v) const;
    double get_vertex_repulsive_energy(const VirtualVertex& _t_vv, const pm::vertex_handle& _l_v) const;

private:
    EmbeddingInput* input;
    pm::Mesh t_m; // Target mesh. Copy.
    pm::vertex_attribute<tg::pos3> t_pos; // Target mesh positions. Copy.

    pm::vertex_attribute<pm::vertex_handle> l_matching_vertex;
    pm::vertex_attribute<pm::vertex_handle> t_matching_vertex;
    pm::halfedge_attribute<pm::halfedge_handle> t_matching_halfedge;

    // Cache for the energy used for vertex repulsive path tracing [Praun2001].
    // Computed lazily when required. Access via get_vertex_repulsive_energy.
    mutable std::optional<pm::vertex_attribute<Eigen::VectorXd>> vertex_repulsive_energy;
};

}
