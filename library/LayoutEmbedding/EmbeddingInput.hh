#pragma once

#include <polymesh/pm.hh>
#include <typed-geometry/tg.hh>
#include <LayoutEmbedding/Assert.hh>

#include <fstream>
#include <sstream>
#include <filesystem>

namespace LayoutEmbedding {

struct EmbeddingInput
{
    pm::Mesh l_m;
    pm::Mesh t_m;

    pm::vertex_attribute<tg::pos3> l_pos;
    pm::vertex_attribute<pm::vertex_handle> l_matching_vertex;

    pm::vertex_attribute<tg::pos3> t_pos;

    EmbeddingInput();

    // Methods
    /** \brief Saves embedding input in .inp file and optionally .obj files
     *         of layout and target mesh.
     *
     * \arg   - filename: Name of input file (including path but excluding suffix)
     *        - write_layout_mesh: If true, also writes obj file of layout_mesh under <filename>_layout.obj
     *        - write_target_input_mesh: If true, also writes obj file of target_mesh under <filename>_target_input.obj
     *
     * \details: Structure of .inp (EmbeddingInput) file
     *           # model name (as a comment)
     *
     *           # relative link to layout mesh
     *           lf  <relative/path/to/layout>
     *           # relative path to target input mesh
     *           tif <relative/path/to/target/input>
     *
     *           # matching vertices
     *           mv <layout_vertex_id_1> <target_vertex_id_1>
     *           ...
     */
    bool save(std::string filename,
              bool write_layout_mesh=true,
              bool write_target_input_mesh=true) const;

    bool load(std::string filename);
};

// TODO: Find elegant way to cast vertex-attribute position from pos3 to std::array<float,3>
bool write_obj_file(std::string file_name, const pm::Mesh & mesh,
                    const pm::vertex_attribute<tg::pos3>& t_pos);

}
