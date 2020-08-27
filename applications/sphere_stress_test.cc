#include <glow-extras/timing/CpuTimer.hh>

#include <polymesh/formats.hh>

#include <typed-geometry/tg.hh>

#include <LayoutEmbedding/Assert.hh>
#include <LayoutEmbedding/BranchAndBound.hh>
#include <LayoutEmbedding/Embedding.hh>
#include <LayoutEmbedding/EmbeddingInput.hh>
#include <LayoutEmbedding/LayoutGeneration.hh>
#include <LayoutEmbedding/Praun2001.hh>

#include <algorithm>
#include <fstream>

using namespace LayoutEmbedding;

int main()
{
    const std::string data_path = LE_DATA_PATH;

    EmbeddingInput input;
    load(data_path + "/models/target-meshes/sphere.obj", input.t_m, input.t_pos);
    load(data_path + "/models/layouts/cube_layout.obj", input.l_m, input.l_pos);

    const std::string stats_filename = "stats_sphere_stress_test.csv";
    {
        std::ofstream f(stats_filename);
        f << "seed,algorithm,runtime,score" << std::endl;
    }

    int seed = 0;
    while (true) {
        for (const auto& algorithm : {"greedy", "greedy_with_swirl_detection", "bnb", "bnb_with_hashing"}) {
            // Generate random matching vertices
            std::srand(seed); // TODO: make the seed a parameter of randomize_matching_vertices?
            randomize_matching_vertices(input);
            Embedding em(input);

            glow::timing::CpuTimer timer;
            if (algorithm == "greedy") {
                Praun2001Settings settings;
                settings.use_swirl_detection = false;
                praun2001(em, settings);
            }
            else if (algorithm == "greedy_with_swirl_detection") {
                Praun2001Settings settings;
                settings.use_swirl_detection = true;
                praun2001(em, settings);
            }
            else if (algorithm == "bnb") {
                BranchAndBoundSettings settings;
                settings.time_limit = 30 * 60;
                settings.use_hashing = false;
                branch_and_bound(em, settings);
            }
            else if (algorithm == "bnb_with_hashing") {
                BranchAndBoundSettings settings;
                settings.time_limit = 30 * 60;
                settings.use_hashing = true;
                branch_and_bound(em, settings);
            }
            else {
                LE_ASSERT(false);
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
