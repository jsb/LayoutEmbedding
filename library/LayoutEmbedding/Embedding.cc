#include "Embedding.hh"

#include <LayoutEmbedding/Assert.hh>
#include <LayoutEmbedding/Connectivity.hh>
#include <LayoutEmbedding/VirtualVertexAttribute.hh>

#include <queue>

namespace LayoutEmbedding {

Embedding::Embedding(const pm::Mesh& _l_m, const pm::Mesh& _t_m, const pm::vertex_attribute<tg::pos3>& _t_pos)  :
    l_m(&_l_m),
    t_m(),
    t_pos(t_m),
    l_matching_vertex(*l_m),
    t_matching_vertex(t_m),
    t_matching_halfedge(t_m)
{
    t_m.copy_from(_t_m);
    t_pos.copy_from(_t_pos);
}

Embedding::Embedding(const Embedding& _em) :
    l_m(_em.l_m),
    t_m(),
    t_pos(t_m),
    l_matching_vertex(*l_m),
    t_matching_vertex(t_m),
    t_matching_halfedge(t_m)
{
    t_m.copy_from(_em.t_m);
    t_pos.copy_from(_em.t_pos);
    for (const auto& l_v : l_m->vertices()) {
        l_matching_vertex[l_v] = t_m[_em.l_matching_vertex[l_v.idx].idx];
    }
    for (const auto& t_v : t_m.vertices()) {
        t_matching_vertex[t_v] = (*l_m)[_em.t_matching_vertex[t_v.idx].idx];
    }
    for (const auto& t_he : t_m.halfedges()) {
        t_matching_halfedge[t_he] = (*l_m)[_em.t_matching_halfedge[t_he.idx].idx];
    }
}

void Embedding::set_matching_vertices(const MatchingVertices& _mvs)
{
    l_matching_vertex.clear();
    t_matching_vertex.clear();
    for (const auto& [l_v, t_v] : _mvs) {
        l_matching_vertex[l_v] = t_v;
        t_matching_vertex[t_v] = l_v;
    }
}

pm::halfedge_handle Embedding::get_embedded_target_halfedge(const pm::halfedge_handle& _l_he) const
{
    LE_ASSERT(_l_he.mesh == l_m);
    const auto& l_v = _l_he.vertex_from();
    const auto& t_v = l_matching_vertex[l_v];
    LE_ASSERT(t_v.is_valid());
    for (const auto t_h : t_v.outgoing_halfedges()) {
        if (t_matching_halfedge[t_h] == _l_he) {
            return t_h;
        }
    }
    return pm::halfedge_handle::invalid;
}

bool Embedding::is_embedded(const pm::halfedge_handle& _l_he) const
{
    return get_embedded_target_halfedge(_l_he).is_valid();
}

bool Embedding::is_embedded(const pm::edge_handle& _l_e) const
{
    return is_embedded(_l_e.halfedgeA());
}

pm::halfedge_handle Embedding::get_embeddable_sector(const pm::halfedge_handle& _l_he) const
{
    LE_ASSERT(_l_he.mesh == l_m);
    LE_ASSERT(get_embedded_target_halfedge(_l_he).is_invalid());

    // Find the first layout halfedge that
    // - comes after _l_h in a clockwise sense,
    // - is already embedded.
    auto t_embedded_he = pm::halfedge_handle::invalid;
    const auto l_he_start = _l_he;
    auto l_he = l_he_start.opposite().next(); // Rotate cw
    while (l_he != l_he_start) {
        if (auto t_he = get_embedded_target_halfedge(l_he); t_he.is_valid()) {
            t_embedded_he = t_he;
            break;
        }
        l_he = l_he.opposite().next(); // Rotate cw
    }

    if (t_embedded_he.is_valid()) {
        return t_embedded_he;
    }
    else {
        // No layout halfedge is embedded at this vertex yet.
        const auto& l_v = _l_he.vertex_from();
        const auto& t_v = l_matching_vertex[l_v];
        LE_ASSERT(t_v.is_valid());
        LE_ASSERT(t_v.mesh == &t_m);
        return t_v.any_outgoing_halfedge();
    }
}

bool Embedding::is_blocked(const pm::edge_handle& _t_e) const
{
    LE_ASSERT(_t_e.mesh == &t_m);
    return t_matching_halfedge[_t_e.halfedgeA()].is_valid() || t_matching_halfedge[_t_e.halfedgeB()].is_valid();
}

bool Embedding::is_blocked(const pm::vertex_handle& _t_v) const
{
    LE_ASSERT(_t_v.mesh == &t_m);
    // Pinned vertices
    if (t_matching_vertex[_t_v].is_valid()) {
        return true;
    }
    // Embedded edges
    for (const auto& t_e : _t_v.edges()) {
        if (is_blocked(t_e)) {
            return true;
        }
    }
    return false;
}

bool Embedding::is_blocked(const VirtualVertex& _t_vv) const
{
    if (const auto* v = std::get_if<pm::vertex_handle>(&_t_vv)) {
        return is_blocked(*v);
    }
    else if (const auto* e = std::get_if<pm::edge_handle>(&_t_vv)) {
        return is_blocked(*e);
    }
    else {
        LE_ASSERT(false);
    }
}

tg::pos3 Embedding::element_pos(const pm::edge_handle& _t_e) const
{
    LE_ASSERT(_t_e.mesh == &t_m);
    return tg::centroid(t_pos[_t_e.vertexA()], t_pos[_t_e.vertexB()]);
}

tg::pos3 Embedding::element_pos(const pm::vertex_handle& _t_v) const
{
    LE_ASSERT(_t_v.mesh == &t_m);
    return t_pos[_t_v];
}

tg::pos3 Embedding::element_pos(const VirtualVertex& _t_vv) const
{
    if (const auto* v = std::get_if<pm::vertex_handle>(&_t_vv)) {
        return element_pos(*v);
    }
    else if (const auto* e = std::get_if<pm::edge_handle>(&_t_vv)) {
        return element_pos(*e);
    }
    else {
        LE_ASSERT(false);
    }
}

VirtualPath Embedding::find_shortest_path(const pm::halfedge_handle& _t_h_sector_start, const pm::halfedge_handle& _t_h_sector_end) const
{
    struct Distance
    {
        int edges_crossed = std::numeric_limits<int>::max();
        double geodesic = std::numeric_limits<double>::infinity();

        bool operator<(const Distance& rhs) const
        {
            //return std::tie(edges_crossed, geodesic) < std::tie(rhs.edges_crossed, rhs.geodesic);
            return geodesic < rhs.geodesic;
        }
    };

    struct Candidate
    {
        VirtualVertex vv;
        tg::pos3 p;
        Distance dist;

        bool operator>(const Candidate& rhs) const
        {
            return rhs.dist < dist; // reversed
        }
    };

    LE_ASSERT(_t_h_sector_start.mesh == &t_m);
    LE_ASSERT(_t_h_sector_end.mesh == &t_m);

    VirtualVertexAttribute<VirtualVertex> prev(t_m);
    VirtualVertexAttribute<Distance> distance(t_m);

    const pm::vertex_handle t_v_start = _t_h_sector_start.vertex_from();
    const pm::vertex_handle t_v_end   = _t_h_sector_end.vertex_from();

    const VirtualVertex vv_start(t_v_start);
    const VirtualVertex vv_end(t_v_end);

    auto get_virtual_vertices_in_sector = [&](const pm::halfedge_handle& t_he_sector) {
        auto t_he_sector_start = t_he_sector;
        auto t_he_sector_end = t_he_sector;
        t_he_sector_end = t_he_sector_end.prev().opposite(); // Rotate ccw
        while (true) {
            if (t_he_sector_start == t_he_sector_end) {
                break;
            }
            if (!is_blocked(t_he_sector_start.edge())) {
                t_he_sector_start = t_he_sector_start.opposite().next(); // Rotate cw
            }
            else if (!is_blocked(t_he_sector_end.edge())) {
                t_he_sector_end = t_he_sector_end.prev().opposite(); // Rotate ccw
            }
            else {
                break;
            }
        }
        std::vector<VirtualVertex> vvs;
        auto t_he = t_he_sector_start;
        do {
            // Incident edge midpoints
            vvs.push_back(t_he.next().edge());

            // Incident vertices
            if (!is_blocked(t_he.edge())) {
                vvs.push_back(t_he.vertex_to());
            }

            t_he = t_he.prev().opposite(); // Rotate ccw
        }
        while (t_he != t_he_sector_end);
        return vvs;
    };

    std::vector<VirtualVertex> legal_first_vvs = get_virtual_vertices_in_sector(_t_h_sector_start);
    std::vector<VirtualVertex> legal_last_vvs = get_virtual_vertices_in_sector(_t_h_sector_end);

    distance[t_v_start].edges_crossed = 0;
    distance[t_v_start].geodesic = 0.0;

    std::priority_queue<Candidate, std::vector<Candidate>, std::greater<Candidate>> q;

    {
        Candidate c;
        c.vv = VirtualVertex(t_v_start);
        c.p = t_pos[t_v_start];
        c.dist.edges_crossed = 0;
        c.dist.geodesic = 0.0;
        q.push(c);
    }

    auto legal_step = [&](const VirtualVertex& from, const VirtualVertex& to) {
        if (from == vv_start) {
            if (std::find(legal_first_vvs.cbegin(), legal_first_vvs.cend(), to) == legal_first_vvs.cend()) {
                return false;
            }
        }

        if (to == vv_end) {
            if (std::find(legal_last_vvs.cbegin(), legal_last_vvs.cend(), from) == legal_last_vvs.cend()) {
                return false;
            }
        }
        else {
            if (is_blocked(to)) {
                return false;
            }
        }

        return true;
    };

    auto visit_vv = [&](const Candidate& c, const VirtualVertex& vv) {
        if (legal_step(c.vv, vv)) {
            const Distance& current_dist = distance[vv];
            const auto& p = element_pos(vv);
            Distance new_dist = c.dist;
            new_dist.geodesic += tg::distance(p, c.p);
            if (is_real_edge(vv)) {
                new_dist.edges_crossed += 1;
            }

            if (new_dist < current_dist) {
                Candidate new_c;
                new_c.vv = vv;
                new_c.p = p;
                new_c.dist = new_dist;

                distance[vv] = new_c.dist;
                prev[vv] = c.vv;

                q.push(new_c);
            }
        }
    };

    while (!q.empty()) {
        const auto u = q.top();
        q.pop();

        const auto& vv = u.vv;

        // Expand vertex neighborhood
        if (is_real_vertex(vv)) {
            const auto& t_v = real_vertex(u.vv);

            if (t_v == t_v_end) {
                break;
            }

            // Add incident vertices (if they're not blocked)
            for (const auto t_v_adj : t_v.adjacent_vertices()) {
                visit_vv(u, VirtualVertex(t_v_adj));
            }

            // Add incident edge midpoints (if they're not blocked)
            for (const auto t_he_out : t_v.outgoing_halfedges()) {
                if (t_he_out.is_boundary()) {
                    continue;
                }
                const auto eh_opp_edge = t_he_out.next().edge();
                visit_vv(u, VirtualVertex(eh_opp_edge));
            }
        }
        // Expand edge midpoint neighborhood
        else if (is_real_edge(vv)) {
            const auto& t_e = real_edge(vv);

            const auto& t_he = t_e.halfedgeA();
            const auto& t_he_opp = t_e.halfedgeB();

            // Visit opposite vertices
            const auto& t_v_u = opposite_vertex(t_he);
            const auto& t_v_u_opp = opposite_vertex(t_he_opp);
            visit_vv(u, VirtualVertex(t_v_u));
            visit_vv(u, VirtualVertex(t_v_u_opp));

            // Visit incident edges
            if (!t_he.is_boundary()) {
                const auto& eh_u_r = t_he.next().edge();
                const auto& eh_u_l = t_he.prev().edge();
                visit_vv(u, eh_u_r);
                visit_vv(u, eh_u_l);
            }
            if (!t_he_opp.is_boundary()) {
                const auto& eh_u_opp_r = t_he_opp.prev().edge();
                const auto& eh_u_opp_l = t_he_opp.next().edge();
                visit_vv(u, eh_u_opp_r);
                visit_vv(u, eh_u_opp_l);
            }
        }
    }

    if (std::isinf(distance[t_v_end].geodesic)) {
        return {};
    }
    else {
        VirtualPath path;
        VirtualVertex vv_current(t_v_end);
        VirtualVertex vv_start(t_v_start);
        while (vv_current != vv_start) {
            path.push_back(vv_current);
            vv_current = prev[vv_current];
        }
        path.push_back(vv_start);
        std::reverse(path.begin(), path.end());
        return path;
    }
}

VirtualPath Embedding::find_shortest_path(const pm::halfedge_handle& _l_he) const
{
    LE_ASSERT(_l_he.mesh == l_m);
    LE_ASSERT(!is_embedded(_l_he));
    const auto l_he_end = _l_he.opposite();
    const auto t_he_sector_start = get_embeddable_sector(_l_he);
    const auto t_he_sector_end = get_embeddable_sector(l_he_end);
    return find_shortest_path(t_he_sector_start, t_he_sector_end);
}

VirtualPath Embedding::find_shortest_path(const pm::edge_handle& _l_e) const
{
    LE_ASSERT(_l_e.mesh == l_m);
    const auto l_he = _l_e.halfedgeA();
    return find_shortest_path(l_he);
}

double Embedding::path_length(const VirtualPath& _path) const
{
    LE_ASSERT(_path.size() >= 2);
    double length = 0.0;
    for (int i = 0; i < _path.size() - 1; ++i) {
        const auto& vv_i = _path[i];
        const auto& vv_j = _path[i+1];
        const auto p_i = element_pos(vv_i);
        const auto p_j = element_pos(vv_j);
        length += tg::distance(p_i, p_j);
    }
    return length;
}

void Embedding::embed_path(const pm::halfedge_handle& _l_he, const VirtualPath& _path)
{
    LE_ASSERT(!get_embedded_target_halfedge(_l_he).is_valid());
    LE_ASSERT(_path.size() >= 2);

    // Turn the VertexEdgePath into a pure vertex path by splitting edges
    std::vector<pm::vertex_handle> vertex_path;
    for (const auto& vv : _path) {
        if (is_real_edge(vv)) {
            const auto& t_e = real_edge(vv);

            const auto& p0 = t_pos[t_e.vertexA()];
            const auto& p1 = t_pos[t_e.vertexB()];
            const auto p = tg::mix(p0, p1, 0.5);

            const auto t_v_new = t_m.edges().split_and_triangulate(t_e);
            t_pos[t_v_new] = p;
            vertex_path.push_back(t_v_new);
        }
        else {
            vertex_path.push_back(std::get<pm::vertex_handle>(vv));
        }
    }

    // Mark the halfedges along the newly subdivided vertex path
    for (int i = 0; i < vertex_path.size() - 1; ++i) {
        int j = i + 1;
        const auto t_he = pm::halfedge_from_to(vertex_path[i], vertex_path[j]);
        LE_ASSERT(t_he.is_valid());
        t_matching_halfedge[t_he] = _l_he;
        t_matching_halfedge[t_he.opposite()] = _l_he.opposite();
    }
}

void Embedding::unembed_path(const pm::halfedge_handle& _l_he)
{
    auto path = get_embedded_path(_l_he);
    for (int i = 0; i < path.size() - 1; ++i) {
        const auto& t_v_i = path[i];
        const auto& t_v_j = path[i+1];
        const auto& t_he = pm::halfedge_from_to(t_v_i, t_v_j);
        LE_ASSERT(t_matching_halfedge[t_he] == _l_he);
        LE_ASSERT(t_matching_halfedge[t_he.opposite()] == _l_he.opposite());
        t_matching_halfedge[t_he] = pm::halfedge_handle::invalid;
        t_matching_halfedge[t_he.opposite()] = pm::halfedge_handle::invalid;
    }
}

void Embedding::unembed_path(const pm::edge_handle& _l_e)
{
    unembed_path(_l_e.halfedgeA());
}

std::vector<pm::vertex_handle> Embedding::get_embedded_path(const pm::halfedge_handle& _l_he) const
{
    LE_ASSERT(is_embedded(_l_he));
    std::vector<pm::vertex_handle> result;
    const auto t_v_start = get_embedded_target_halfedge(_l_he).vertex_from();
    const auto t_v_end = l_matching_vertex[_l_he.vertex_to()];
    auto t_v = t_v_start;
    while (t_v != t_v_end) {
        result.push_back(t_v);
        for (const auto t_he : t_v.outgoing_halfedges()) {
            if (t_matching_halfedge[t_he] == _l_he) {
                t_v = t_he.vertex_to();
                break;
            }
        }
    }
    result.push_back(t_v_end);
    return result;
}

double Embedding::embedded_path_length(const pm::halfedge_handle& _l_he) const
{
    LE_ASSERT(is_embedded(_l_he));
    const auto& path = get_embedded_path(_l_he);
    LE_ASSERT(path.size() >= 2);
    double length = 0.0;
    for (int i = 0; i < path.size() - 1; ++i) {
        const auto& v_i = path[i];
        const auto& v_j = path[i + 1];
        const auto& p_i = t_pos[v_i];
        const auto& p_j = t_pos[v_j];
        length += tg::distance(p_i, p_j);
    }
    return length;
}

double Embedding::embedded_path_length(const polymesh::edge_handle& _l_e) const
{
    return embedded_path_length(_l_e.halfedgeA());
}

double Embedding::total_embedded_path_length() const
{
    double total_length = 0.0;
    for (const auto& l_e : l_m->edges()) {
        if (is_embedded(l_e)) {
            total_length += embedded_path_length(l_e);
        }
    }
    return total_length;
}

bool Embedding::is_complete() const
{
    for (const auto& l_e : l_m->edges()) {
        if (!is_embedded(l_e)) {
            return false;
        }
    }
    return true;
}

const polymesh::Mesh& Embedding::layout_mesh() const
{
    return *l_m;
}

const polymesh::Mesh& Embedding::target_mesh() const
{
    return t_m;
}

const pm::vertex_attribute<tg::pos3>& Embedding::target_pos() const
{
    return t_pos;
}

const pm::vertex_handle Embedding::matching_target_vertex(const pm::vertex_handle& _l_v) const
{
    LE_ASSERT(_l_v.mesh == l_m);
    return l_matching_vertex[_l_v];
}

const pm::vertex_handle Embedding::matching_layout_vertex(const pm::vertex_handle& _t_v) const
{
    LE_ASSERT(_t_v.mesh == &t_m);
    return t_matching_vertex[_t_v];
}

}
