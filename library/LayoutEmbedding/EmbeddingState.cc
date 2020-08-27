#include "EmbeddingState.hh"

#include <LayoutEmbedding/Assert.hh>
#include <LayoutEmbedding/UnionFind.hh>
#include <LayoutEmbedding/VirtualPathConflictSentinel.hh>

namespace LayoutEmbedding {

EmbeddingState::EmbeddingState(const Embedding& _em) :
    em(_em),
    candidate_paths(_em.layout_mesh())
{
}

void EmbeddingState::extend(const polymesh::edge_handle& _l_e)
{
    LE_ASSERT(_l_e.mesh == &em.layout_mesh());
    LE_ASSERT(embedded_l_edges.count(_l_e) == 0);

    auto l_he = _l_e.halfedgeA();
    auto path = em.find_shortest_path(l_he);
    if (path.empty()) {
        embedded_cost = std::numeric_limits<double>::infinity();
        valid = false;
        return;
    }
    else {
        embedded_cost += em.path_length(path);
        em.embed_path(l_he, path);
        embedded_l_edges.insert(_l_e);
    }
}

void EmbeddingState::extend(const polymesh::edge_index& _l_ei)
{
    const auto& l_e = em.layout_mesh().edges()[_l_ei];
    extend(l_e);
}

void EmbeddingState::extend(const InsertionSequence& _seq)
{
    for (const auto& l_e : _seq) {
        extend(l_e);
        if (!valid) {
            break;
        }
    }
}

void EmbeddingState::compute_candidate_paths()
{
    const Embedding& c_em = em; // We don't want to modify the embedding in this method.

    LE_ASSERT(&candidate_paths.mesh() == &em.layout_mesh());
    candidate_paths.clear();
    conflicting_l_edges.clear();
    non_conflicting_l_edges.clear();
    unembedded_cost = 0.0;

    VirtualPathConflictSentinel vpcs(c_em);

    for (const auto& l_e : c_em.layout_mesh().edges()) {
        if (!embedded_l_edges.count(l_e)) {
            auto l_he = l_e.halfedgeA();
            auto path = c_em.find_shortest_path(l_he);
            if (path.empty()) {
                unembedded_cost = std::numeric_limits<double>::infinity();
                valid = false;
                break;
            }
            candidate_paths[l_e].path = path;
            candidate_paths[l_e].cost = c_em.path_length(path);
            vpcs.insert_path(path, l_e);
            unembedded_cost += candidate_paths[l_e].cost;
        }
    }
    if (!valid) {
        return;
    }

    vpcs.check_path_ordering();

    conflicting_l_edges = vpcs.global_conflicts;
    for (const auto& l_e : c_em.layout_mesh().edges()) {
        if (!embedded_l_edges.count(l_e) && !conflicting_l_edges.count(l_e)) {
            non_conflicting_l_edges.insert(l_e);
        }
    }
    LE_ASSERT(embedded_l_edges.size() + conflicting_l_edges.size() + non_conflicting_l_edges.size() == em.layout_mesh().edges().size());
}

double EmbeddingState::cost_lower_bound() const
{
    return embedded_cost + unembedded_cost;
}

HashValue EmbeddingState::hash() const
{
    HashValue h = 0;
    for (const auto l_e : em.layout_mesh().edges()) {
        if (em.is_embedded(l_e)) {
            const auto& path = em.get_embedded_path(l_e.halfedgeA());
            for (const auto& t_v : path) {
                const auto& pos = em.target_pos()[t_v];
                h = hash_combine(h, LayoutEmbedding::hash(pos));
            }
        }
    }
    return h;
}

int EmbeddingState::count_connected_components() const
{
    int num_components = em.layout_mesh().faces().size();
    UnionFind face_components(num_components);
    for (const auto l_e : em.layout_mesh().edges()) {
        if (!em.is_embedded(l_e)) {
            const int id_A = l_e.faceA().idx.value;
            const int id_B = l_e.faceB().idx.value;
            if (!face_components.equivalent(id_A, id_B)) {
                --num_components;
                face_components.merge(id_A, id_B);
            }
        }
    }
    return num_components;
}

}
