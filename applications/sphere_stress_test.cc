#include <glow-extras/timing/CpuTimer.hh>

#include <polymesh/formats.hh>

#include <typed-geometry/tg.hh>

#include <LayoutEmbedding/Assert.hh>
#include <LayoutEmbedding/BranchAndBound.hh>
#include <LayoutEmbedding/Embedding.hh>
#include <LayoutEmbedding/LayoutGeneration.hh>
#include <LayoutEmbedding/Praun2001.hh>

#include <algorithm>
#include <fstream>
#include <numeric>

using namespace LayoutEmbedding;

int main()
{
    const std::string data_path = LE_DATA_PATH;

    pm::Mesh input_t_m;
    auto input_t_pos = input_t_m.vertices().make_attribute<tg::pos3>();
    load(data_path + "/models/target-meshes/sphere.obj", input_t_m, input_t_pos);

    pm::Mesh input_l_m;
    auto input_l_pos = input_l_m.vertices().make_attribute<tg::pos3>();
    load(data_path + "/models/layouts/cube_layout.obj", input_l_m, input_l_pos);

    const std::string stats_filename = "stats_sphere_stress_test.csv";
    {
        std::ofstream f(stats_filename);
        f << "seed,algorithm,runtime,score" << std::endl;
    }

    int seed = 0;
    while (true) {
        for (const auto& algorithm : {"greedy", "bnb"}) {
            Embedding em(input_l_m, input_t_m, input_t_pos);
            const auto& l_m = em.layout_mesh();
            const auto& t_m = em.target_mesh();
            const auto& t_pos = em.target_pos();

            // Generate random matching vertices
            std::srand(seed);
            std::vector<pm::vertex_handle> t_vertices = t_m.vertices().to_vector();
            std::random_shuffle(t_vertices.begin(), t_vertices.end());
            MatchingVertices matching_vertices;
            for (int i = 0; i < l_m.vertices().size(); ++i) {
                matching_vertices.push_back({l_m.vertices()[i], t_vertices[i]});
            }

            em.set_matching_vertices(matching_vertices);

            glow::timing::CpuTimer timer;
            if (algorithm == "greedy") {
                praun2001(em);
            }
            else if (algorithm == "bnb") {
                BranchAndBoundSettings settings;
                settings.time_limit = 30 * 60;
                branch_and_bound(em, settings);
            }
            const double runtime = timer.elapsedSeconds();
            const double embedding_cost = em.is_complete() ? em.total_embedded_path_length() : std::numeric_limits<double>::infinity();

            {
                std::ofstream f{stats_filename, std::ofstream::app};
                f << seed << ",";
                f << algorithm << ",";
                f << runtime << ",";
                f << embedding_cost << std::endl;
            }

            std::cout << "Seed:      " << seed << std::endl;
            std::cout << "Algorithm: " << algorithm << std::endl;
            std::cout << "Runtime:   " << runtime << std::endl;
            std::cout << "Cost:      " << embedding_cost << std::endl;
        }
        ++seed;
    }
}
