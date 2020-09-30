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

void EmbeddingState::extend(const pm::edge_index& _l_ei, const VirtualPath& _path)
{
    const auto& l_e = em.layout_mesh().edges()[_l_ei];
    LE_ASSERT(!em.is_embedded(l_e));

    LE_ASSERT(_path.size() >= 2);

    auto l_he = l_e.halfedgeA();
    LE_ASSERT(is_real_vertex(_path.front()));
    LE_ASSERT(is_real_vertex(_path.back()));
    LE_ASSERT(real_vertex(_path.front()) == em.matching_target_vertex(l_he.vertex_from()));
    LE_ASSERT(real_vertex(_path.back())  == em.matching_target_vertex(l_he.vertex_to()));

    em.embed_path(l_he, _path);
}

void EmbeddingState::compute_candidate_path(const pm::edge_index& _l_ei)
{
    const Embedding& c_em = em; // We don't want to modify the embedding in this method.
    const auto& l_e = c_em.layout_mesh().edges()[_l_ei];

    LE_ASSERT(&candidate_paths.mesh() == &c_em.layout_mesh());
    LE_ASSERT(!em.is_embedded(l_e));

    auto l_he = l_e.halfedgeA();
    auto path = c_em.find_shortest_path(l_he);

    candidate_paths[l_e] = path;
}

void EmbeddingState::compute_all_candidate_paths()
{
    const Embedding& c_em = em; // We don't want to modify the embedding in this method.

    candidate_paths.clear();
    for (const auto l_e : c_em.layout_mesh().edges()) {
        if (!c_em.is_embedded(l_e)) {
            compute_candidate_path(l_e);
        }
    }
}

void EmbeddingState::detect_candidate_path_conflicts()
{
    const Embedding& c_em = em; // We don't want to modify the embedding in this method.
    conflicts.clear();

    if (valid()) {
        VirtualPathConflictSentinel vpcs(c_em);
        for (const auto l_e : c_em.layout_mesh().edges()) {
            if (!c_em.is_embedded(l_e)) {
                const auto& path = candidate_paths[l_e];
                LE_ASSERT(!path.empty());
                vpcs.insert_path(path, l_e);
            }
        }
        vpcs.check_path_ordering();
        conflicts = vpcs.conflict_relation;
    }

    LE_ASSERT(c_em.layout_mesh().edges().size() == embedded_edges().size() + conflicting_edges().size() + non_conflicting_edges().size());
}

std::vector<pm::edge_index> EmbeddingState::get_conflicting_candidates(const pm::edge_index& _l_ei)
{
    std::vector<pm::edge_index> result;
    for (const auto& [l_ei_A, l_ei_B] : conflicts) {
        LE_ASSERT(l_ei_A != l_ei_B);
        if (_l_ei == l_ei_A) {
            result.push_back(l_ei_B);
        }
        else if (_l_ei == l_ei_B) {
            result.push_back(l_ei_A);
        }
    }
    return result;
}

bool EmbeddingState::valid() const
{
    for (const auto l_e : em.layout_mesh().edges()) {
        if (!em.is_embedded(l_e)) {
            if (candidate_paths[l_e].empty()) {
                return false;
            }
        }
    }
    return true;
}

double EmbeddingState::cost_lower_bound() const
{
    if (use_candidate_paths_for_lower_bounds) {
        return embedded_cost() + unembedded_cost();
    }
    else {
        if (std::isinf(unembedded_cost())) {
            return std::numeric_limits<double>::infinity();
        }
        else {
            return embedded_cost();
        }
    }
}

double EmbeddingState::embedded_cost() const
{
    return em.total_embedded_path_length();
}

double EmbeddingState::unembedded_cost() const
{
    double result = 0.0;
    for (const auto l_e : em.layout_mesh().edges()) {
        if (!em.is_embedded(l_e)) {
            const auto& path = candidate_paths[l_e];
            if (path.empty()) {
                return std::numeric_limits<double>::infinity();
            }
            else {
                result += em.path_length(path);
            }
        }
    }
    return result;
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

std::set<pm::edge_index> EmbeddingState::embedded_edges() const
{
    std::set<pm::edge_index> result;
    for (const auto l_e : em.layout_mesh().edges()) {
        if (em.is_embedded(l_e)) {
            result.insert(l_e);
        }
    }
    return result;
}

std::set<polymesh::edge_index> EmbeddingState::unembedded_edges() const
{
    std::set<pm::edge_index> result;
    for (const auto l_e : em.layout_mesh().edges()) {
        if (!em.is_embedded(l_e)) {
            result.insert(l_e);
        }
    }
    return result;
}

std::set<pm::edge_index> EmbeddingState::conflicting_edges() const
{
    std::set<pm::edge_index> result;
    for (const auto& [l_ei_A, l_ei_B] : conflicts) {
        LE_ASSERT(l_ei_A != l_ei_B);
        LE_ASSERT(!em.is_embedded(l_ei_A));
        LE_ASSERT(!em.is_embedded(l_ei_B));

        result.insert(l_ei_A);
        result.insert(l_ei_B);
    }
    return result;
}

std::set<pm::edge_index> EmbeddingState::non_conflicting_edges() const
{
    std::set<pm::edge_index> result;
    const auto& conf = conflicting_edges();
    for (const auto l_e : em.layout_mesh().edges()) {
        if (!em.is_embedded(l_e)) {
            if (!conf.count(l_e)) {
                result.insert(l_e);
            }
        }
    }
    return result;
}

}
