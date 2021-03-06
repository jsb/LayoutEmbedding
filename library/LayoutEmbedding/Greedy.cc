#include "Greedy.hh"

#include <LayoutEmbedding/IGLMesh.hh>
#include <LayoutEmbedding/UnionFind.hh>
#include <LayoutEmbedding/VirtualPort.hh>
#include <LayoutEmbedding/Util/Assert.hh>

#include <algorithm>
#include <set>
#include <queue>

namespace LayoutEmbedding {

namespace {

/// Heuristic detection of paths that might introduce swirls after insertion.
/// For each vertex around the face that is incident to _l_he on the left,
/// a shortest path towards the given path is traced.
/// If the path is hit from the right side (instead of the left), this is considered a potential swirl.
/// Returns true if a potential swirl is detected, false otherwise.
bool swirl_detection(Embedding& _em, const pm::halfedge_handle& _l_he, const VirtualPath& _path)
{
    const pm::Mesh& l_m = _em.layout_mesh();
    const pm::Mesh& t_m = _em.target_mesh();
    const pm::vertex_attribute<tg::pos3>& t_pos = _em.target_pos();

    // Walk along the VertexEdgePath and mark the vertices directly left and right of it with a special attribute:
    // The indicator attribute assigns each vertex a value in {-1, 0, 1},
    // -1 meaning it is directly on the left of the arc,
    // 1 meaning it is directly on the right of the arc,
    // 0 otherwise.
    pm::vertex_attribute<int> t_indicator(t_m);

    LE_ASSERT(is_real_vertex(_path.front()));
    LE_ASSERT(is_real_vertex(_path.back()));

    for (int i = 0; i < _path.size(); ++i) {
        const auto& vv = _path[i];

        if (is_real_vertex(vv)) {
            if (i > 0 && i < _path.size() - 1) {
                const auto& el_prev = _path[i - 1];
                const auto& el_next = _path[i + 1];
                const auto& v = real_vertex(vv, t_m);

                VirtualPort vh_start{v, el_prev};
                VirtualPort vh_end{v, el_next};

                auto vh_current = vh_start.rotated_cw();
                while (vh_current != vh_end) {
                    if (is_real_vertex(vh_current.to)) {
                        const auto& v_to = real_vertex(vh_current.to);
                        t_indicator[v_to] = -1; // "Left"
                    }
                    vh_current = vh_current.rotated_cw();
                }

                while (vh_current != vh_start) {
                    if (is_real_vertex(vh_current.to)) {
                        const auto& v_to = real_vertex(vh_current.to);
                        t_indicator[v_to] = 1; // "Right"
                    }
                    vh_current = vh_current.rotated_cw();
                }
            }
        }
        else if (is_real_edge(vv)) {
            LE_ASSERT_G(i, 0);
            LE_ASSERT_L(i, _path.size() - 1);

            const auto& e = real_edge(vv, t_m);
            auto he = pm::halfedge_handle::invalid;

            const auto& vv_next = _path[i + 1];
            if (is_real_vertex(vv_next)) {
                const auto& v_next = real_vertex(vv_next);
                if (e.halfedgeA().next().vertex_to() == v_next) {
                    he = e.halfedgeA();
                }
                else if (e.halfedgeB().next().vertex_to() == v_next) {
                    he = e.halfedgeB();
                }
            }
            else if (is_real_edge(vv_next)) {
                const auto& e_next = real_edge(vv_next, t_m);

                if ((e.halfedgeA().face() == e_next.halfedgeA().face()) || (e.halfedgeA().face() == e_next.halfedgeB().face())) {
                    he = e.halfedgeA();
                }
                else if ((e.halfedgeB().face() == e_next.halfedgeA().face()) || (e.halfedgeB().face() == e_next.halfedgeB().face())) {
                    he = e.halfedgeB();
                }
            }

            LE_ASSERT(he.is_valid());
            t_indicator[he.vertex_from()] = -1; // "Left"
            t_indicator[he.vertex_to()] = 1; // "Right"
        }
    }

    // Start a shortest-path search from the seed vertices and see whether it first meets a vertex marked "Left" (good) or "Right" (bad)

    struct Candidate
    {
        double distance;
        pm::vertex_handle v;

        bool operator<(const Candidate& _rhs) const
        {
            return distance > _rhs.distance;
        }
    };

    pm::vertex_attribute<double> distance(t_m, std::numeric_limits<double>::infinity());
    std::priority_queue<Candidate> q;
    std::vector<pm::vertex_handle> t_seed_vertices;
    const auto& l_f = _l_he.face();
    for (const auto l_v : l_f.vertices()) {
        if ((l_v == _l_he.vertex_from()) || (l_v == _l_he.vertex_to())) {
            continue;
        }
        const auto t_v = _em.matching_target_vertex(l_v);
        t_seed_vertices.push_back(t_v);
        distance[t_v] = 0.0;
        q.push({0.0, t_v});
    }

    while (!q.empty()) {
        const auto c = q.top();
        q.pop();

        if (t_indicator[c.v] == -1) {
            // We arrived on the correct (left) side of the path. Probably no spiral.
            return false;
        }
        else if (t_indicator[c.v] == 1) {
            // We arrived on the wrong (right) side of the path. Spiral detected.
            return true;
        }

        for (const auto he : c.v.outgoing_halfedges()) {
            const auto& v_to = he.vertex_to();
            const double new_distance = distance[c.v] + tg::distance(t_pos[c.v], t_pos[v_to]);
            if (new_distance < distance[v_to]) {
                distance[v_to] = new_distance;
                q.push({new_distance, v_to});
            }
        }
    }
    // This will likely be never reached
    return false;
}

bool swirl_detection_bidirectional(Embedding& _em, const pm::halfedge_handle& _l_he, const VirtualPath& _path)
{
    if (swirl_detection(_em, _l_he, _path)) {
        return true;
    }
    else {
        const auto l_he_opp = _l_he.opposite();
        auto path_opp = _path;
        std::reverse(path_opp.begin(), path_opp.end());
        return swirl_detection(_em, l_he_opp, path_opp);
    }
}

/// [Kraevoy2003] / [Kraevoy2004] blocking condition.
/// _l_h_seed is already (temporarily) embedded.
bool is_blocking(const Embedding& _em, const pm::halfedge_handle& _l_h_seed)
{
    LE_ASSERT(_em.is_embedded(_l_h_seed));

    // Find sets of layout vertices in patch defined by _l_h.
    // Both in layout and target mesh.
    std::set<pm::vertex_index> l_vertices;
    std::set<pm::vertex_index> t_vertices;

    { // Collect vertices in layout
        std::queue<pm::halfedge_handle> queue;
        queue.push(_l_h_seed);

        auto visited = _em.layout_mesh().faces().make_attribute<bool>(false);
        while (!queue.empty()) {
            const auto h = queue.front();
            const auto f = h.face();
            queue.pop();

            if (visited[f])
                continue;
            visited[f] = true;

            // Collect layout vertices
            for (auto l_v : f.vertices())
                l_vertices.insert(l_v.idx);

            // Enqueue neighbors
            for (auto l_h_f : f.halfedges()) {
                const auto l_h_opp = l_h_f.opposite();
                if (!_em.is_embedded(l_h_opp) && !visited[l_h_opp.face()]) {
                    queue.push(l_h_opp);
                }
            }
        }
    }

    { // Collect vertices in target mesh
        std::queue<pm::halfedge_handle> queue;
        queue.push(_em.get_embedded_target_halfedge(_l_h_seed));

        auto visited = _em.target_mesh().faces().make_attribute<bool>(false);
        while (!queue.empty()) {
            const auto h = queue.front();
            const auto f = h.face();
            queue.pop();

            if (visited[f])
                continue;
            visited[f] = true;

            // Collect layout vertices
            for (auto t_v : f.vertices()) {
                const auto l_v = _em.matching_layout_vertex(t_v);
                if (l_v.is_valid())
                    t_vertices.insert(l_v.idx);
            }

            // Enqueue neighbors
            for (auto t_h_f : f.halfedges()) {
                const auto t_h_opp = t_h_f.opposite();
                if (!_em.is_blocked(t_h_opp.edge()) && !visited[t_h_opp.face()]) {
                    queue.push(t_h_opp);
                }
            }
        }
    }

    return l_vertices != t_vertices;
}

/// [Kraevoy2003] / [Kraevoy2004] blocking condition.
/// Check if sets of layout vertices left and right of path match between layout and target mesh.
/// _l_e is not yet embedded.
bool is_blocking(const Embedding& _em, const pm::edge_handle& _l_e, const VirtualPath& _path)
{
    LE_ASSERT(!_em.is_embedded(_l_e));

    // Copy embedding to temporarily embed the path
    Embedding em_copy = _em;
    em_copy.embed_path(_l_e.halfedgeA(), _path);

    return is_blocking(em_copy, _l_e.halfedgeA()) || is_blocking(em_copy, _l_e.halfedgeB());
}

}

GreedyResult embed_greedy(Embedding& _em, const GreedySettings& _settings, const std::string& _name)
{
    GreedyResult result(_name, _settings);

    // If vertex-repulsive tracing is enabled, copy input embedding.
    // Used to re-trace paths as shortest paths.
    std::optional<Embedding> em_copy;
    if (_settings.use_vertex_repulsive_tracing)
        em_copy = _em;

    const pm::Mesh& l_m = _em.layout_mesh();

    auto l_extremal_vertex = l_m.vertices().make_attribute<bool>(false);
    if (_settings.prefer_extremal_vertices) {
        // Compute for each vertex the average geodesic distance to its neighbors
        auto l_avg_neighbor_distance = l_m.vertices().make_attribute<double>();
        for (const auto l_v : l_m.vertices()) {
            double total_distance = 0.0;
            int valence = 0;
            for (const auto l_he : l_v.outgoing_halfedges()) {
                const auto path = _em.find_shortest_path(l_he);
                total_distance += _em.path_length(path);
                ++valence;
            }
            l_avg_neighbor_distance[l_v] = total_distance / valence;
        }

        std::vector<pm::vertex_handle> extremal_vertices = l_m.vertices().to_vector();
        std::sort(extremal_vertices.begin(), extremal_vertices.end(), [&](const auto& a, const auto& b){
            return l_avg_neighbor_distance[a] > l_avg_neighbor_distance[b];
        });
        double cutoff = l_avg_neighbor_distance[extremal_vertices[extremal_vertices.size() * _settings.extremal_vertex_ratio]];
        int num_extremal_vertices = 0;
        for (const auto& l_v : extremal_vertices) {
            if (l_avg_neighbor_distance[l_v] > cutoff) {
                l_extremal_vertex[l_v] = true;
                ++num_extremal_vertices;
            }
        }
    }

    auto incident_to_extremal_vertex = [&] (const pm::edge_handle& _l_e) {
        if (_l_e.is_valid()) {
            return l_extremal_vertex[_l_e.vertexA()] && l_extremal_vertex[_l_e.vertexB()];
        }
        else {
            return false;
        }
    };

    // If edges fail the "Swirl Test", their score will receive a penalty so they are processed later.
    pm::edge_attribute<bool> l_penalty(l_m);

    pm::edge_attribute<bool> l_is_embedded(l_m);
    const int l_num_vertices = l_m.vertices().size();
    const int l_num_edges = l_m.edges().size();
    int l_num_embedded_edges = 0;

    UnionFind l_v_components(l_m.vertices().size());

    while (l_num_embedded_edges < l_num_edges) {
        VirtualPath best_path;
        double best_path_cost = std::numeric_limits<double>::infinity();
        pm::edge_handle best_l_e = pm::edge_handle::invalid;

        const bool is_spanning_tree = (l_num_embedded_edges >= l_num_vertices - 1);

        for (const auto l_e : l_m.edges()) {
            if (l_is_embedded[l_e]) {
                continue;
            }

            int l_vi_a = l_e.vertexA().idx.value;
            int l_vi_b = l_e.vertexB().idx.value;

            if (!_settings.use_blocking_condition) {
                if (!is_spanning_tree) {
                    if (l_v_components.equivalent(l_vi_a, l_vi_b)) {
                        continue;
                    }
                }
            }

            auto metric = Embedding::ShortestPathMetric::Geodesic;
            if (_settings.use_vertex_repulsive_tracing) {
                metric = Embedding::ShortestPathMetric::VertexRepulsive;
            }

            VirtualPath path = _em.find_shortest_path(l_e.halfedgeA(), metric);
            double path_cost = _em.path_length(path);

            // If we use the blocking condition, we have to discard the path if
            // the vertices enclosed in new patches differ between the layout and the embedding.
            if (_settings.use_blocking_condition) {
                if (l_v_components.equivalent(l_vi_a, l_vi_b)) {
                    if (is_blocking(_em, l_e, path)) {
                        continue;
                    }
                }
            }

            // If we use an arbitrary insertion order, we can early-out after the first path is found
            if (_settings.insertion_order == GreedySettings::InsertionOrder::Arbitrary) {
                best_path_cost = path_cost;
                best_path = std::move(path);
                best_l_e = l_e;
                break;
            }

            if (_settings.use_swirl_detection) {
                // Only do the swirl test if the current path is already a contender.
                if (path_cost < best_path_cost) {
                    if (swirl_detection_bidirectional(_em, l_e.halfedgeA(), path)) {
                        path_cost *= _settings.swirl_penalty_factor;
                    }
                }
            }

            const int extremal_priority = 1 - incident_to_extremal_vertex(l_e);
            const int best_extremal_priority = 1 - incident_to_extremal_vertex(best_l_e);
            if (std::tie(extremal_priority, path_cost) < std::tie(best_extremal_priority, best_path_cost)) {
                best_path_cost = path_cost;
                best_path = std::move(path);
                best_l_e = l_e;
            }
        }

        result.insertion_sequence.push_back(best_l_e);
        _em.embed_path(best_l_e.halfedgeA(), best_path);
        l_v_components.merge(best_l_e.vertexA().idx.value, best_l_e.vertexB().idx.value);
        l_is_embedded[best_l_e] = true;
        ++l_num_embedded_edges;
    }

    // If vertex-repulsive tracing was used,
    // re-trace the insertion sequence as shortest paths
    if (_settings.use_vertex_repulsive_tracing) {
        LE_ASSERT(em_copy);
        for (auto l_e_idx : result.insertion_sequence) {
            const auto l_h = em_copy.value().layout_mesh().edges()[l_e_idx].halfedgeA();
            const VirtualPath path = em_copy.value().find_shortest_path(l_h, Embedding::ShortestPathMetric::Geodesic);
            em_copy.value().embed_path(l_h, path);
        }
        _em = em_copy.value();
    }

    LE_ASSERT(_em.is_complete());
    result.cost = _em.total_embedded_path_length();

    return result;
}

GreedyResult embed_praun(Embedding& _em, const GreedySettings& _settings)
{
    GreedySettings settings = _settings;
    settings.use_swirl_detection = true;
    settings.use_vertex_repulsive_tracing = true;
    settings.use_blocking_condition = false;
    settings.prefer_extremal_vertices = false;

    return embed_greedy(_em, settings, "praun");
}

GreedyResult embed_kraevoy(Embedding& _em, const GreedySettings& _settings)
{
    GreedySettings settings = _settings;
    settings.use_swirl_detection = false;
    settings.use_vertex_repulsive_tracing = false;
    settings.use_blocking_condition = true;
    settings.prefer_extremal_vertices = false;

    return embed_greedy(_em, settings, "kraevoy");
}

GreedyResult embed_schreiner(Embedding& _em, const GreedySettings& _settings)
{
    GreedySettings settings = _settings;
    settings.use_swirl_detection = true;
    settings.use_vertex_repulsive_tracing = false;
    settings.use_blocking_condition = false;
    settings.prefer_extremal_vertices = true;

    return embed_greedy(_em, settings, "schreiner");
}

std::vector<GreedyResult> embed_greedy(Embedding& _em, const std::vector<GreedySettings>& _all_settings)
{
    const int n = _all_settings.size();
    std::vector<Embedding> all_embeddings(n, _em); // n copies
    std::vector<GreedyResult> all_results(n);

    //#pragma omp parallel for
    for (std::size_t i = 0; i < n; ++i) {
        const auto& settings = _all_settings[i];

        auto& em = all_embeddings[i];
        auto& result = all_results[i];

        em = _em; // copy
        result = embed_greedy(em, settings);

        if (result.settings.use_swirl_detection)
            result.algorithm += "_swirl";
        if (result.settings.use_vertex_repulsive_tracing)
            result.algorithm += "_repulsive";
        if (result.settings.prefer_extremal_vertices)
            result.algorithm += "_extremal";

        std::cout << "Embedding cost: " << result.cost << std::endl;
    }

    int best_idx;
    const auto& best_result = best(all_results, best_idx);

    std::cout << "Best settings:" << std::endl;
    std::cout << std::boolalpha;
    std::cout << "    use_swirl_detection: " << best_result.settings.use_swirl_detection << std::endl;
    std::cout << "    use_vertex_repulsive_tracing: " << best_result.settings.use_vertex_repulsive_tracing << std::endl;
    std::cout << "    prefer_extremal_vertices: " << best_result.settings.prefer_extremal_vertices << std::endl;
    std::cout << "Best cost: " << best_result.cost << std::endl;

    _em = all_embeddings[best_idx]; // copy

    return all_results;
}

std::vector<GreedyResult> embed_competitors(Embedding& _em, const GreedySettings& _settings)
{
    std::vector<GreedySettings> all_settings;
    { // Plain
        GreedySettings settings = _settings;
        settings.use_swirl_detection = false;
        settings.use_vertex_repulsive_tracing = false;
        settings.use_blocking_condition = false;
        settings.prefer_extremal_vertices = false;
        all_settings.push_back(settings);
    }
    { // [Praun2001]
        GreedySettings settings = _settings;
        settings.use_swirl_detection = true;
        settings.use_vertex_repulsive_tracing = true;
        settings.use_blocking_condition = false;
        settings.prefer_extremal_vertices = false;
        all_settings.push_back(settings);
    }
    { // [Kraevoy2003] / [Kraevoy2004]
        GreedySettings settings = _settings;
        settings.use_swirl_detection = false;
        settings.use_vertex_repulsive_tracing = false;
        settings.use_blocking_condition = true;
        settings.prefer_extremal_vertices = false;
        all_settings.push_back(settings);
    }
    { // [Schreiner2004]
        GreedySettings settings = _settings;
        settings.use_swirl_detection = true;
        settings.use_vertex_repulsive_tracing = false;
        settings.use_blocking_condition = false;
        settings.prefer_extremal_vertices = true;
        all_settings.push_back(settings);
    }

    return embed_greedy(_em, all_settings);
}

const GreedyResult& best(const std::vector<GreedyResult>& _results)
{
    int best_idx;
    return best(_results, best_idx);
}

const GreedyResult& best(const std::vector<GreedyResult>& _results, int& best_idx)
{
    double best_cost = std::numeric_limits<double>::infinity();
    best_idx = -1;
    for (std::size_t i = 0; i < _results.size(); ++i) {
        if (_results[i].cost < best_cost) {
            best_cost = _results[i].cost;
            best_idx = i;
        }
    }
    LE_ASSERT_GEQ(best_idx, 0);

    return _results[best_idx];
}

}
