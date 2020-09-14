#include <glow-extras/timing/CpuTimer.hh>

#include <polymesh/formats.hh>

#include <typed-geometry/tg.hh>

#include <LayoutEmbedding/ApproximateGeodesicDistance.hh>
#include <LayoutEmbedding/Assert.hh>
#include <LayoutEmbedding/BranchAndBound.hh>
#include <LayoutEmbedding/Embedding.hh>
#include <LayoutEmbedding/EmbeddingInput.hh>
#include <LayoutEmbedding/Greedy.hh>
#include <LayoutEmbedding/LayoutGeneration.hh>
#include <LayoutEmbedding/StackTrace.hh>

#include <algorithm>
#include <filesystem>
#include <fstream>

using namespace LayoutEmbedding;

int main()
{
    namespace fs = std::filesystem;

    register_segfault_handler();

    const fs::path data_path = LE_DATA_PATH;
    const fs::path output_dir = LE_OUTPUT_PATH;
    const fs::path jitter_evaluation_output_dir = output_dir / "jitter_evaluation";

    EmbeddingInput input;
    load(data_path / "models/layouts/horse_layout.obj", input.l_m, input.l_pos);
    load(data_path / "models/target-meshes/horse_8078.obj", input.t_m, input.t_pos);
    find_matching_vertices_by_proximity(input);

    std::vector<pm::vertex_handle> matching_target_vertices;
    for (const auto l_v : input.l_m.vertices()) {
        const auto& t_v = input.l_matching_vertex[l_v];
        matching_target_vertices.push_back(t_v);
    }
    auto geodesic_distance = approximate_geodesic_distance(input.t_pos, matching_target_vertices);

    const fs::path stats_path = jitter_evaluation_output_dir / "stats.csv";
    fs::create_directories(jitter_evaluation_output_dir);
    {
        std::ofstream f{stats_path};
        f << "jitter_iters,seed,avg_d,max_d,algo,cost,t" << '\n';
    }

    for (int jitter_iters = 0; jitter_iters < 40; ++jitter_iters) {
        int num_seeds = 1;
        if (jitter_iters == 0) {
            num_seeds = 1;
        }
        for (int seed = 0; seed < num_seeds; ++seed) {
            EmbeddingInput jittered_input = input; // Copy
            jitter_matching_vertices(jittered_input, jitter_iters, seed);

            //std::vector<double> sampled_geodesic_distances;
            double max_d = 0.0;
            double sum_d = 0.0;
            for (const auto l_v : jittered_input.l_m.vertices()) {
                const auto& t_v = jittered_input.l_matching_vertex[l_v];
                const auto d = geodesic_distance[t_v.idx];
                max_d = std::max(max_d, d);
                sum_d += d;
            }
            const double avg_d = sum_d / jittered_input.l_m.vertices().size();

            std::cout<< "Jitter Distance: " << jitter_iters << ", Seed: " << seed << " Avg Geodesic Distance: " << avg_d << ", Max Geodesic Distance: " << max_d << std::endl;

            const std::vector<std::string> algos = {
                "bnb",
                "praun",
                "kraevoy",
                "schreiner",
            };

            for (const auto& algo : algos) {
                Embedding em(jittered_input);

                double cost = std::numeric_limits<double>::infinity();
                glow::timing::CpuTimer timer;
                if (algo == "bnb") {
                    BranchAndBoundSettings settings;
                    settings.time_limit = 60.0;
                    auto result = branch_and_bound(em, settings);
                    cost = result.cost;
                }
                else if (algo == "praun") {
                    auto result = embed_praun(em);
                    cost = result.cost;
                }
                else if (algo == "kraevoy") {
                    auto result = embed_kraevoy(em);
                    cost = result.cost;
                }
                else if (algo == "schreiner") {
                    auto result = embed_schreiner(em);
                    cost = result.cost;
                }
                else {
                    LE_ERROR_THROW("unknown algo");
                }
                const double runtime = timer.elapsedSecondsD();

                {
                    std::ofstream f{stats_path, std::ofstream::app};
                    f << jitter_iters << ",";
                    f << seed << ",";
                    f << avg_d << ",";
                    f << max_d << ",";
                    f << algo << ",";
                    f << cost << ",";
                    f << runtime << "\n";
                }
            }

            /*
            const fs::path lb_path = jitter_evaluation_output_dir / (std::to_string(seed) + "_lower_bound.csv");
            const fs::path ub_path = jitter_evaluation_output_dir / (std::to_string(seed) + "_upper_bound.csv");
            Embedding em(jittered_input);

            BranchAndBoundSettings settings;
            settings.record_lower_bound_events = true;
            settings.record_upper_bound_events = true;
            settings.time_limit = 10 * 60;
            auto result = branch_and_bound(em, settings);

            fs::create_directories(jitter_evaluation_output_dir);
            {
                std::ofstream f{lb_path};
                f << "t,lower_bound" << '\n';
                for (const auto& record : result.lower_bound_events) {
                    f << record.t << "," << record.lower_bound << '\n';
                }
            }
            {
                std::ofstream f{ub_path};
                f << "t,upper_bound" << '\n';
                for (const auto& record : result.upper_bound_events) {
                    f << record.t << "," << record.upper_bound << '\n';
                }
            }
            */
        }
    }
}
