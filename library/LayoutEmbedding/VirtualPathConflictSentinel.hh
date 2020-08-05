#pragma once

#include <LayoutEmbedding/Embedding.hh>
#include <LayoutEmbedding/VirtualVertex.hh>
#include <LayoutEmbedding/VirtualPath.hh>

namespace LayoutEmbedding {

struct VirtualPathConflictSentinel
{
    const Embedding& em;

    using Segment = std::pair<VirtualVertex, VirtualVertex>;
    using Label = pm::edge_index;
    using LabelSet = std::set<Label>;

    pm::vertex_attribute<Label> v_label;
    pm::edge_attribute<Label> e_label;
    pm::face_attribute<Label> f_label;
    LabelSet global_conflicts;

    explicit VirtualPathConflictSentinel(const Embedding& _em);

    void insert(const pm::vertex_handle& _v, const Label& _l);
    void insert(const pm::edge_handle& _e, const Label& _l);
    void insert(const pm::face_handle& _f, const Label& _l);
    void insert_virtual_vertex(const VirtualVertex& _vv, const Label& _l);
    void insert_segment(const VirtualVertex& _vv0, const VirtualVertex& _vv1, const Label& _l);
    void insert_path(const VirtualPath& _path, const Label& _l);
};

}
