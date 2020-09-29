#pragma once

#include <LayoutEmbedding/Embedding.hh>
#include <LayoutEmbedding/VirtualPath.hh>
#include <LayoutEmbedding/VirtualPort.hh>
#include <LayoutEmbedding/VirtualVertex.hh>
#include <LayoutEmbedding/VirtualVertexAttribute.hh>

namespace LayoutEmbedding {

struct VirtualPathConflictSentinel
{
    const Embedding& em;

    using Segment = std::pair<VirtualVertex, VirtualVertex>;
    using Label = pm::edge_index;
    using LabelSet = std::set<Label>;

    using Conflict = std::pair<Label, Label>;
    using ConflictSet = std::set<Conflict>;

    pm::vertex_attribute<LabelSet> v_label;
    pm::edge_attribute<LabelSet> e_label;
    pm::face_attribute<LabelSet> f_label;

    ConflictSet conflict_relation; // The pairs of labels which are conflicting

    pm::halfedge_attribute<VirtualPort> l_port;

    explicit VirtualPathConflictSentinel(const Embedding& _em);

    void insert(const pm::vertex_handle& _v, const Label& _l);
    void insert(const pm::edge_handle& _e, const Label& _l);
    void insert(const pm::face_handle& _f, const Label& _l);
    void insert_virtual_vertex(const VirtualVertex& _vv, const Label& _l);
    void insert_segment(const VirtualVertex& _vv0, const VirtualVertex& _vv1, const Label& _l);
    void insert_path(const VirtualPath& _path, const Label& _l);

    void mark_conflicting(const Label& _a, const Label& _b);

    void check_path_ordering();
};

}
