#include "EmbeddingInput.hh"

#include <LayoutEmbedding/LayoutGeneration.hh>

namespace LayoutEmbedding
{

EmbeddingInput::EmbeddingInput() :
    l_pos(l_m),
    l_matching_vertex(l_m),
    t_pos(t_m)
{
}

EmbeddingInput::EmbeddingInput(const EmbeddingInput& _ei)
{
    (*this) = _ei;
}

EmbeddingInput& EmbeddingInput::operator=(const EmbeddingInput& _ei)
{
    l_m.copy_from(_ei.l_m);
    t_m.copy_from(_ei.t_m);

    l_pos = l_m.vertices().make_attribute<tg::pos3>();
    l_pos.copy_from(_ei.l_pos);

    t_pos = t_m.vertices().make_attribute<tg::pos3>();
    t_pos.copy_from(_ei.t_pos);

    l_matching_vertex = l_m.vertices().make_attribute<pm::vertex_handle>();
    for (const auto& l_v : l_m.vertices()) {
        l_matching_vertex[l_v] = t_m[_ei.l_matching_vertex[l_v.idx].idx];
    }

    return *this;
}

bool EmbeddingInput::save(std::string filename,
                                           bool write_layout_mesh, bool write_target_input_mesh) const
{
    // Names (including paths) of files to be stored
    //   For target mesh
    std::string t_m_write_file_name = filename + "_target_input.obj";
    //   For layout mesh
    std::string l_m_write_file_name = filename + "_layout.obj";
    //   For embedding
    std::string inp_write_file_name  = filename + ".inp";
    // Filename without path (for symbolic links in inp file
    auto last_slash_pos = filename.find_last_of("/");
    std::string filename_without_path;
    // Check, whether directory specified in relative path already exists
    std::string directory_of_inp_file;
    // If relative path hints to subdirectory, check whether subdirectory exists
    if(last_slash_pos != std::string::npos)
    {
        filename_without_path = filename.substr(last_slash_pos+1);

        // Check, whether directory exists
         directory_of_inp_file = filename.substr(0, last_slash_pos+1);
        LE_ASSERT(std::filesystem::exists(directory_of_inp_file));
    }
    else
    {
        // No dirs in filename
        filename_without_path = filename;
    }


    if(write_target_input_mesh) // default argument: true
    {
        pm::save(t_m_write_file_name, t_pos);
    }

    if(write_layout_mesh) // default argument: true
    {
        pm::save(l_m_write_file_name, l_pos);
    }

    // Prepare writing embedded mesh. See file "lem" file format for more information

    // Collect matching vertex pairs
    std::vector<std::pair<pm::vertex_handle, pm::vertex_handle>> matching_vertices_vector;
    for(auto layout_vertex: l_m.vertices())
    {
        if(l_matching_vertex[layout_vertex].is_valid())
        {
            matching_vertices_vector.emplace_back(std::make_pair(layout_vertex, l_matching_vertex[layout_vertex]));
        }
    }

    // Now write this data to the corresponding lem file
    // TODO: Check whether this file exists using std::filesystem::exists(em_write_file_name)
    std::ofstream inp_file_stream(inp_write_file_name);
    if(inp_file_stream.is_open())
    {
        // Write links to layout mesh and target mesh
        inp_file_stream << "lf " + filename_without_path + "_layout.obj" + "\n";
        inp_file_stream << "tif " + filename_without_path + "_target_input.obj" + "\n\n";

        // Write matching vertices
        for(auto pair: matching_vertices_vector)
        {
            inp_file_stream << "mv " + std::to_string(int(pair.first.idx)) + " " + std::to_string(int(pair.second.idx)) + "\n";
        }
        inp_file_stream << "\n";

        inp_file_stream.close();
    }
    else
    {
        std::cerr << "Could not create inp file." << std::endl;
        return false;
    }
    return true;
}

bool EmbeddingInput::load(const std::string& _path_prefix)
{
    std::string inp_file_name = _path_prefix;
    std::cout << "Input file name: " << inp_file_name << std::endl;
    std::string lm_file_name;
    std::string tim_file_name;
    // Extract (relative) path of embedding file (relative to current working directory)
    auto last_slash_position = _path_prefix.find_last_of("/");
    std::string directory_of_inp_file = "";
    // If relative path hints to subdirectory, check whether subdirectory exists
    if(last_slash_position != std::string::npos)
    {
        // Only add relative path, if it is not . (same directory)
        directory_of_inp_file = _path_prefix.substr(0, last_slash_position+1);
        // Check, whether directory exists
        LE_ASSERT(std::filesystem::exists(directory_of_inp_file));
    }
    // Open input file stream for embedding file
    std::ifstream inp_file_stream(inp_file_name);
    LE_ASSERT(inp_file_stream.is_open()==true);
    // Start parsing embedding file (Adopted from polymesh::obj_reader.parse)
    std::string line_s;
    auto line_nr = 0;
    // Used to store tokens from files
    std::vector<std::pair<int, int>> mv_token_vector;
    std::string tempString;

    // Parse lem file
    while(std::getline(inp_file_stream, line_s))
    {
        ++line_nr;
        while (line_s.size() > 0 && (line_s.back() == '\r' || line_s.back() == ' ' || line_s.back() == '\t'))
            line_s.pop_back();
        std::istringstream line(line_s);
        std::string type;


        line >> type;

        // empty lines
        if (type.empty())
        {
            continue;
        }
        // comments
        else if (type[0] == '#')
        {
            continue;
        }
        // layout mesh file
        else if(type == "lf")
        {
            line >> lm_file_name;
        }
        // target mesh file
        else if(type == "tif")
        {
            line >> tim_file_name;
        }
        // Matching vertices
        else if(type == "mv")
        {
            // Read ID of layout vertex into buffer
            tempString.clear();
            line >> tempString;
            int layout_vertex_id = std::stoi(tempString);
            tempString.clear();
            // Read ID of corresponding target vertex into buffer
            line >> tempString;
            int target_vertex_id = std::stoi(tempString);
            // Make vertex_handles
            mv_token_vector.emplace_back(std::make_pair(layout_vertex_id, target_vertex_id));
        }
    }

    // Add relative path prefix
    lm_file_name = directory_of_inp_file + lm_file_name;
    std::cout << "Name of layout file " + lm_file_name << std::endl;
    tim_file_name = directory_of_inp_file + tim_file_name;


    // Load layout mesh into input
    if(!pm::load(lm_file_name, l_m, l_pos))
    {
        std::cerr << "Could not load layout mesh object file that was specified in the inp file. Please check again." << std::endl;
        return false;
    }
    // Load target mesh into input
    if(!pm::load(tim_file_name, t_m, t_pos))
    {
        std::cerr << "Could not load target mesh object file that was specified in the inp file. Please check again." << std::endl;
        return false;
    }
    // Save matching vertices
    for(auto matching_pair: mv_token_vector)
    {
        l_matching_vertex[l_m[pm::vertex_index(matching_pair.first)]] = t_m[pm::vertex_index(matching_pair.second)];
    }
    return true;
}

bool EmbeddingInput::load(
        const fs::path& _layout_path,
        const fs::path& _target_path)
{
    // Load layout
    LE_ASSERT(pm::load(_layout_path, l_m, l_pos));
    std::cout << "Layout Mesh: ";
    std::cout << l_m.vertices().size() << " vertices, ";
    std::cout << l_m.edges().size() << " edges, ";
    std::cout << l_m.faces().size() << " faces. ";
    std::cout << "χ = " << pm::euler_characteristic(l_m) << std::endl;

    // Load target mesh
    LE_ASSERT(pm::load(_target_path, t_m, t_pos));
    std::cout << "Target Mesh: ";
    std::cout << t_m.vertices().size() << " vertices, ";
    std::cout << t_m.edges().size() << " edges, ";
    std::cout << t_m.faces().size() << " faces. ";
    std::cout << "χ = " << pm::euler_characteristic(t_m) << std::endl;

//    if (pm::euler_characteristic(l_m) != pm::euler_characteristic(t_m)) {
//        std::cout << "Euler characteristic does not match. Skipping." << std::endl;
//        return false;
//    }

    find_matching_vertices_by_proximity(*this);

    return true;
}

bool EmbeddingInput::load(
        const fs::path& _layout_path,
        const fs::path& _target_path,
        const fs::path& _landmarks_path,
        const LandmarkFormat& _format)
{
    if (!load(_layout_path, _target_path)) {
        return false;
    }

    // Load landmarks from file
    const auto landmark_ids = load_landmarks(_landmarks_path, _format);
    std::cout << landmark_ids.size() << " landmarks." << std::endl;
    if (landmark_ids.size() != l_m.vertices().size()) {
        std::cout << "Wrong number of landmarks. Skipping." << std::endl;
        return false;
    }
    for (size_t i = 0; i < landmark_ids.size(); ++i) {
        const auto l_v = l_m.vertices()[i];
        const auto t_v = t_m.vertices()[landmark_ids[i]];
        l_matching_vertex[l_v] = t_v;
    }

    return true;
}

void EmbeddingInput::normalize_surface_area()
{
    auto area = [] (const pm::vertex_attribute<tg::pos3>& pos)
    {
        return pos.mesh().faces().sum([&] (auto f) { return pm::face_area(f, pos); });
    };

    const double new_area = 1.0;
    const double scale = std::sqrt(new_area) / std::sqrt(area(t_pos));
    t_pos.apply([&] (auto& p) { p *= scale; });
    std::cout << "Normalized surface area of target mesh." << std::endl;
}

void EmbeddingInput::center_translation()
{
    // Move center of gravity to origin
    const auto cog = t_pos.sum() / t_pos.mesh().vertices().size();
    t_pos.apply([&] (auto& p) { p -= cog - tg::pos3::zero; });
}

void EmbeddingInput::invert_layout()
{
    // Remember face connectivity
    std::vector<std::vector<pm::vertex_handle>> fv_indices;
    for (const auto f : l_m.faces()) {
        std::vector<pm::vertex_handle> local_fv_indices;
        for (const auto v : f.vertices()) {
            local_fv_indices.push_back(v);
        }
        fv_indices.push_back(std::move(local_fv_indices));
    }

    // Remove all edges and faces
    for (const auto e : l_m.edges()) {
        l_m.edges().remove(e);
    }

    // Rebuild the mesh using inverted faces
    for (auto& local_fv_indices : fv_indices) {
        std::reverse(local_fv_indices.begin(), local_fv_indices.end());
        l_m.faces().add(local_fv_indices);
    }

    l_m.compactify();
}

std::vector<int> load_landmarks(const fs::path& _file_path, const LandmarkFormat& _format)
{
    std::ifstream f { _file_path };
    std::vector<int> result;

    switch (_format) {
        case LandmarkFormat::id_x_y_z:
        {
            while (f.good()) {
                int id;
                float x, y, z; // Unused
                f >> id >> x >> y >> z;
                if (f.good()) {
                    result.push_back(id);
                }
            }
            break;
        }
        case LandmarkFormat::id:
        {
            while (f.good()) {
                int id;
                f >> id;
            }
            break;
        }
        default:
            LE_ERROR_THROW("");
    }

    return result;
}

}
