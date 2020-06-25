#include "Embedding.hh"

#include "connectivity.hh"

#include <cassert>
#include <queue>

Embedding make_embedding(pm::Mesh& _l_m, RefinableMesh& _rm)
{
    return {&_l_m, &_rm, _l_m, *_rm.m, *_rm.m};
}

void set_matching_vertices(Embedding& _e, std::vector<std::pair<pm::vertex_handle, pm::vertex_handle>> _mvs)
{
    _e.l_matching_vertex.clear();
    _e.t_matching_vertex.clear();
    for (const auto& [l_v, t_v] : _mvs) {
        _e.l_matching_vertex[l_v] = t_v;
        _e.t_matching_vertex[t_v] = l_v;
    }
}

pm::halfedge_handle get_embedded_target_halfedge(const Embedding& _e, const pm::halfedge_handle& _l_he)
{
    assert(_l_he.mesh == _e.l_m);
    const auto& l_v = _l_he.vertex_from();
    const auto& t_v = _e.l_matching_vertex[l_v];
    if (t_v.is_valid()) {
        for (const auto t_h : t_v.outgoing_halfedges()) {
            if (_e.t_matching_halfedge[t_h] == _l_he) {
                return t_h;
            }
        }
    }
    return pm::halfedge_handle::invalid;
}

polymesh::halfedge_handle get_embeddable_sector(const Embedding& _e, const pm::halfedge_handle& _l_he)
{
    assert(_l_he.mesh == _e.l_m);

    // TODO: maybe we can skip this check!
    if (get_embedded_target_halfedge(_e, _l_he).is_valid()) {
        return pm::halfedge_handle::invalid;
    }

    // Find the first layout halfedge that
    // - comes after _l_h in a clockwise sense,
    // - is already embedded.
    auto t_embedded_he = pm::halfedge_handle::invalid;
    const auto l_he_start = _l_he;
    auto l_he = l_he_start.opposite().next(); // Rotate cw
    while (l_he != l_he_start) {
        if (auto t_he = get_embedded_target_halfedge(_e, l_he); t_he.is_valid()) {
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
        const auto& t_v = _e.l_matching_vertex[l_v];
        assert(t_v.is_valid());
        assert(t_v.mesh == _e.t_m->m);
        return t_v.any_outgoing_halfedge();
    }
}

VertexEdgePath find_shortest_path(
    const Embedding& _e,
    const pm::halfedge_handle& _t_h_sector_start,
    const pm::halfedge_handle& _t_h_sector_end
)
{
    struct Distance
    {
        int edges_crossed = std::numeric_limits<int>::max();
        double geodesic = std::numeric_limits<double>::infinity();

        bool operator<(const Distance& rhs) const
        {
            return std::tie(edges_crossed, geodesic) < std::tie(rhs.edges_crossed, rhs.geodesic);
            //return geodesic < rhs.geodesic;
        }
    };

    struct Candidate
    {
        VertexEdgeElement el;
        tg::pos3 p;
        Distance dist;

        bool operator>(const Candidate& rhs) const
        {
            return rhs.dist < dist; // reversed
        }
    };

    const pm::Mesh& l_m = *_e.l_m;
    const pm::Mesh& t_m = *_e.t_m->m;
    const pm::vertex_attribute<tg::pos3>& t_pos = *_e.t_m->pos;

    assert(_t_h_sector_start.mesh == &t_m);
    assert(_t_h_sector_end.mesh == &t_m);

    VertexEdgeAttribute<VertexEdgeElement> prev(t_m);
    VertexEdgeAttribute<Distance> distance(t_m);

    const pm::vertex_handle t_v_start = _t_h_sector_start.vertex_from();
    const pm::vertex_handle t_v_end   = _t_h_sector_end.vertex_from();

    const VertexEdgeElement el_start(t_v_start);
    const VertexEdgeElement el_end(t_v_end);

    auto get_elements_in_sector = [&](const pm::halfedge_handle& t_he_sector) {
        auto t_he_sector_start = t_he_sector;
        auto t_he_sector_end = t_he_sector;
        t_he_sector_end = t_he_sector_end.prev().opposite(); // Rotate ccw
        while (true) {
            if (t_he_sector_start == t_he_sector_end) {
                break;
            }
            if (!is_blocked(_e, t_he_sector_start.edge())) {
                t_he_sector_start = t_he_sector_start.opposite().next(); // Rotate cw
            }
            else if (!is_blocked(_e, t_he_sector_end.edge())) {
                t_he_sector_end = t_he_sector_end.prev().opposite(); // Rotate ccw
            }
            else {
                break;
            }
        }
        std::vector<VertexEdgeElement> elements;
        auto t_he = t_he_sector_start;
        do {
            // Incident edge midpoints
            VertexEdgeElement el_edge(t_he.next().edge());
            elements.push_back(el_edge);

            // Incident vertices
            if (!is_blocked(_e, t_he.edge())) {
                VertexEdgeElement el_vertex(t_he.vertex_to());
                elements.push_back(el_vertex);
            }

            t_he = t_he.prev().opposite(); // Rotate ccw
        }
        while (t_he != t_he_sector_end);
        return elements;
    };

    std::vector<VertexEdgeElement> legal_first_elements = get_elements_in_sector(_t_h_sector_start);
    std::vector<VertexEdgeElement> legal_last_elements = get_elements_in_sector(_t_h_sector_end);

    distance[t_v_start].edges_crossed = 0;
    distance[t_v_start].geodesic = 0.0;

    std::priority_queue<Candidate, std::vector<Candidate>, std::greater<Candidate>> q;

    {
        Candidate c;
        c.el = VertexEdgeElement(t_v_start);
        c.p = t_pos[t_v_start];
        c.dist.edges_crossed = 0;
        c.dist.geodesic = 0.0;
        q.push(c);
    }

    auto legal_step = [&](const VertexEdgeElement& from, const VertexEdgeElement& to) {
        if (from == el_start) {
            if (std::find(legal_first_elements.cbegin(), legal_first_elements.cend(), to) == legal_first_elements.cend()) {
                return false;
            }
        }

        if (to == el_end) {
            if (std::find(legal_last_elements.cbegin(), legal_last_elements.cend(), from) == legal_last_elements.cend()) {
                return false;
            }
        }
        else {
            if (is_blocked(_e, to)) {
                return false;
            }
        }

        return true;
    };

    auto visit_elem = [&](const Candidate& c, const VertexEdgeElement& el) {
        if (legal_step(c.el, el)) {
            const Distance& current_dist = distance[el];
            const auto& p = element_pos(_e, el);
            Distance new_dist = c.dist;
            new_dist.geodesic += tg::distance(p, c.p);
            if (std::holds_alternative<pm::edge_handle>(el)) {
                new_dist.edges_crossed += 1;
            }

            if (new_dist < current_dist) {
                Candidate new_c;
                new_c.el = VertexEdgeElement(el);
                new_c.p = p;
                new_c.dist = new_dist;

                distance[el] = new_c.dist;
                prev[el] = c.el;

                q.push(new_c);
            }
        }
    };

    while (!q.empty()) {
        const auto u = q.top();
        q.pop();

        const auto& el = u.el;

        // Expand vertex neighborhood
        if (std::holds_alternative<pm::vertex_handle>(el)) {
            const auto& t_v = std::get<pm::vertex_handle>(u.el);

            if (t_v == t_v_end) {
                break;
            }

            // Add incident vertices (if they're not blocked)
            for (const auto t_v_adj : t_v.adjacent_vertices()) {
                visit_elem(u, VertexEdgeElement(t_v_adj));
            }

            // Add incident edge midpoints (if they're not blocked)
            for (const auto t_he_out : t_v.outgoing_halfedges()) {
                if (t_he_out.is_boundary()) {
                    continue;
                }
                const auto eh_opp_edge = t_he_out.next().edge();
                visit_elem(u, VertexEdgeElement(eh_opp_edge));
            }
        }
        // Expand edge midpoint neighborhood
        else if (std::holds_alternative<pm::edge_handle>(el)) {
            const auto& t_e = std::get<pm::edge_handle>(el);

            const auto& t_he = t_e.halfedgeA();
            const auto& t_he_opp = t_e.halfedgeB();

            // Visit opposite vertices
            const auto& t_v_u = opposite_vertex(t_he);
            const auto& t_v_u_opp = opposite_vertex(t_he_opp);
            visit_elem(u, VertexEdgeElement(t_v_u));
            visit_elem(u, VertexEdgeElement(t_v_u_opp));

            // Visit incident edges
            if (!t_he.is_boundary()) {
                const auto& eh_u_r = t_he.next().edge();
                const auto& eh_u_l = t_he.prev().edge();
                visit_elem(u, eh_u_r);
                visit_elem(u, eh_u_l);
            }
            if (!t_he_opp.is_boundary()) {
                const auto& eh_u_opp_r = t_he_opp.prev().edge();
                const auto& eh_u_opp_l = t_he_opp.next().edge();
                visit_elem(u, eh_u_opp_r);
                visit_elem(u, eh_u_opp_l);
            }
        }
    }

    if (std::isinf(distance[t_v_end].geodesic)) {
        return {};
    }
    else {
        VertexEdgePath path;
        VertexEdgeElement el_current(t_v_end);
        VertexEdgeElement el_start(t_v_start);
        while (el_current != el_start) {
            path.push_back(el_current);
            el_current = prev[el_current];
        }
        path.push_back(el_start);
        std::reverse(path.begin(), path.end());
        return path;
    }
}

VertexEdgePath find_shortest_path(const Embedding& _e, const pm::halfedge_handle& _l_he)
{
    assert(_l_he.mesh == _e.l_m);
    const auto l_he_end = _l_he.opposite();
    const auto t_he_sector_start = get_embeddable_sector(_e, _l_he);
    const auto t_he_sector_end = get_embeddable_sector(_e, l_he_end);
    return find_shortest_path(_e, t_he_sector_start, t_he_sector_end);
}

VertexEdgePath find_shortest_path(const Embedding& _e, const pm::edge_handle& _l_e)
{
    assert(_l_e.mesh == _e.l_m);
    const auto l_he = _l_e.halfedgeA();
    return find_shortest_path(_e, l_he);
}

bool is_blocked(const Embedding& _e, const pm::edge_handle& _t_e)
{
    assert(_t_e.mesh == _e.t_m->m);
    return _e.t_matching_halfedge[_t_e.halfedgeA()].is_valid() || _e.t_matching_halfedge[_t_e.halfedgeB()].is_valid();
}

bool is_blocked(const Embedding& _e, const pm::vertex_handle& _t_v)
{
    assert(_t_v.mesh == _e.t_m->m);
    for (const auto& t_e : _t_v.edges()) {
        if (is_blocked(_e, t_e)) {
            return true;
        }
    }
    return false;
}

bool is_blocked(const Embedding& _e, const VertexEdgeElement& _t_el)
{
    if (const auto* v = std::get_if<pm::vertex_handle>(&_t_el)) {
        return is_blocked(_e, *v);
    }
    else if (const auto* e = std::get_if<pm::edge_handle>(&_t_el)) {
        return is_blocked(_e, *e);
    }
    else {
        assert(false);
    }
}

tg::pos3 element_pos(const Embedding& _e, const pm::edge_handle& _t_e)
{
    const auto& t_pos = *_e.t_m->pos;
    return tg::centroid(t_pos[_t_e.vertexA()], t_pos[_t_e.vertexB()]);
}

tg::pos3 element_pos(const Embedding& _e, const pm::vertex_handle& _t_v)
{
    const auto& t_pos = *_e.t_m->pos;
    return t_pos[_t_v];
}

tg::pos3 element_pos(const Embedding& _e, const VertexEdgeElement& _t_el)
{
    if (const auto* v = std::get_if<pm::vertex_handle>(&_t_el)) {
        return element_pos(_e, *v);
    }
    else if (const auto* e = std::get_if<pm::edge_handle>(&_t_el)) {
        return element_pos(_e, *e);
    }
    else {
        assert(false);
    }
}

void embed_path(Embedding& _e, const pm::halfedge_handle& _l_h, const VertexEdgePath& _path)
{
    assert(!get_embedded_target_halfedge(_e, _l_h).is_valid());

    auto& t_m = *_e.t_m->m;

    // Turn the VertexEdgePath into a pure vertex path by splitting edges
    std::vector<pm::vertex_handle> vertex_path;
    for (const auto& el : _path) {
        if (std::holds_alternative<pm::edge_handle>(el)) {
            const auto& t_e = std::get<pm::edge_handle>(el);
            const auto t_v_new = split_edge(*_e.t_m, t_e);
            vertex_path.push_back(t_v_new);
        }
        else {
            vertex_path.push_back(std::get<pm::vertex_handle>(el));
        }
    }

    // Mark the halfedges along the newly subdivided vertex path
    for (int i = 0; i < vertex_path.size() - 1; ++i) {
        int j = i + 1;
        const auto t_he = pm::halfedge_from_to(vertex_path[i], vertex_path[j]);
        assert(t_he.is_valid());
        _e.t_matching_halfedge[t_he] = _l_h;
        _e.t_matching_halfedge[t_he.opposite()] = _l_h.opposite();
    }
}

double path_length(const Embedding& _e, const VertexEdgePath& _path)
{
    double length = 0.0;
    for (int i = 0; i < _path.size() - 1; ++i) {
        const auto& el_i = _path[i];
        const auto& el_j = _path[i+1];
        const auto p_i = element_pos(_e, el_i);
        const auto p_j = element_pos(_e, el_j);
        length += tg::distance(p_i, p_j);
    }
    return length;
}

bool is_embedded(const Embedding& _e, const pm::halfedge_handle& _l_he)
{
    return get_embedded_target_halfedge(_e, _l_he).is_valid();
}

bool is_embedded(const Embedding& _e, const pm::edge_handle& _l_e)
{
    return is_embedded(_e, _l_e.halfedgeA());
}

std::vector<pm::vertex_handle> get_embedded_path(const Embedding& _e, const pm::halfedge_handle& _l_he)
{
    assert(is_embedded(_e, _l_he));
    std::vector<pm::vertex_handle> result;
    const auto t_v_start = get_embedded_target_halfedge(_e, _l_he).vertex_from();
    const auto t_v_end = _e.l_matching_vertex[_l_he.vertex_to()];
    auto t_v = t_v_start;
    while (t_v != t_v_end) {
        result.push_back(t_v);
        for (const auto t_he : t_v.outgoing_halfedges()) {
            if (_e.t_matching_halfedge[t_he] == _l_he) {
                t_v = t_he.vertex_to();
                break;
            }
        }
    }
    result.push_back(t_v_end);
    return result;
}
