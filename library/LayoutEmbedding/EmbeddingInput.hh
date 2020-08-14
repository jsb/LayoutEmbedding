#pragma once

#include <polymesh/pm.hh>
#include <typed-geometry/tg.hh>

namespace LayoutEmbedding {

struct EmbeddingInput
{
    pm::Mesh l_m;
    pm::Mesh t_m;

    pm::vertex_attribute<tg::pos3> l_pos;
    pm::vertex_attribute<pm::vertex_handle> l_matching_vertex;

    pm::vertex_attribute<tg::pos3> t_pos;

    EmbeddingInput();
};

}
