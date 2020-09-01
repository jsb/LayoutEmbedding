#include <glow-extras/glfw/GlfwContext.hh>
#include <glow-extras/viewer/view.hh>
#include <glow-extras/timing/CpuTimer.hh>

#include <imgui/imgui.h>

#include <polymesh/formats.hh>

#include <typed-geometry/tg.hh>

#include <LayoutEmbedding/Assert.hh>
#include <LayoutEmbedding/BranchAndBound.hh>
#include <LayoutEmbedding/Embedding.hh>
#include <LayoutEmbedding/EmbeddingInput.hh>
#include <LayoutEmbedding/LayoutGeneration.hh>
#include <LayoutEmbedding/Praun2001.hh>
#include <LayoutEmbedding/Visualization/Visualization.hh>
#include <LayoutEmbedding/Visualization/RWTHColors.hh>

#include <algorithm>
#include <fstream>
#include <numeric>

using namespace LayoutEmbedding;

static bool screenshots_only = true;
static auto screenshot_size = tg::ivec2(1920, 1080);
static int screenshot_samples = 64;

namespace  {

struct TestCase
{
    std::string name;
    std::string target_mesh_filename;
    std::string layout_mesh_filename;
    std::optional<gv::camera_transform> view;
    int jitter_vertices = 0;
};

void run_test_case(const TestCase& tc)
{
    const std::vector<std::string> algorithms = {
        "greedy",
        "greedy_with_swirl_detection",
        "bnb",
        "bnb_with_hashing",
    };

    const std::string stats_filename = "stats_" + tc.name + ".csv";
    {
        // CSV header
        std::ofstream f{stats_filename};
        f << "layout_edges,algorithm,runtime,score" << std::endl;
    }

    for (const auto& algorithm : algorithms) {
        // Load meshes from file
        EmbeddingInput input;
        load(tc.target_mesh_filename, input.t_m, input.t_pos);
        load(tc.layout_mesh_filename, input.l_m, input.l_pos);

        // Find embedding positions for the Layout vertices on the Target Mesh (nearest neighbors)
        find_matching_vertices_by_proximity(input);
        // Optional: Perturb the target positions on the surface of the Target Mesh a bit
        jitter_matching_vertices(input, tc.jitter_vertices);

        Embedding em(input);
        const pm::Mesh& l_m = em.layout_mesh();
        const pm::Mesh& t_m = em.target_mesh();

        std::cout << "Layout Mesh: ";
        std::cout << l_m.vertices().size() << " vertices, ";
        std::cout << l_m.edges().size() << " edges, ";
        std::cout << l_m.faces().size() << " faces. ";
        std::cout << "χ = " << pm::euler_characteristic(l_m) << std::endl;

        std::cout << "Target Mesh: ";
        std::cout << t_m.vertices().size() << " vertices, ";
        std::cout << t_m.edges().size() << " edges, ";
        std::cout << t_m.faces().size() << " faces. ";
        std::cout << "χ = " << pm::euler_characteristic(t_m) << std::endl;

        glow::timing::CpuTimer timer;

        if (algorithm == "bnb") {
            BranchAndBoundSettings settings;
            settings.use_hashing = false;
            branch_and_bound(em, settings);
        }
        else if (algorithm == "bnb_with_hashing") {
            BranchAndBoundSettings settings;
            settings.use_hashing = true;
            branch_and_bound(em, settings);
        }
        else if (algorithm == "greedy") {
            GreedySettings settings;
            settings.insertion_order = GreedySettings::InsertionOrder::BestFirst;
            settings.use_swirl_detection = false;
            embed_greedy(em, settings);
        }
        else if (algorithm == "greedy_with_swirl_detection") {
            GreedySettings settings;
            settings.insertion_order = GreedySettings::InsertionOrder::BestFirst;
            settings.use_swirl_detection = true;
            embed_greedy(em, settings);
        }
        else {
            LE_ASSERT(false);
        }

        // Stats
        const double optimization_time = timer.elapsedSeconds();
        const double score = em.total_embedded_path_length();
        std::cout << "Optimization took " << optimization_time << " s" << std::endl;
        std::cout << "Total embedding length: " << score << std::endl;
        {
            std::ofstream f{stats_filename, std::ofstream::app};
            f << l_m.edges().size() << ",";
            f << algorithm << ",";
            f << optimization_time << ",";
            f << score << std::endl;
        }

        // Visualization
        auto style = default_style();

        // Optional config: Camera position
        auto cfg_view = tc.view ? gv::config(tc.view.value()) : gv::config();

        if (screenshots_only) {
            {
                const std::string filename = "screenshot_" + tc.name + "_" + algorithm + "_layout.png";
                auto cfg_screenshot = gv::config(gv::headless_screenshot(screenshot_size, screenshot_samples, filename));
                view_layout(em);
            }
            {
                const std::string filename = "screenshot_" + tc.name + "_" + algorithm + "_target.png";
                auto cfg_screenshot = gv::config(gv::headless_screenshot(screenshot_size, screenshot_samples, filename));
                view_target(em);
            }
        }
        else {
            auto g = gv::grid();
            {
                view_layout(em);
            }
            {
                view_target(em);
            }
        }
    }
}

}

int main()
{
    glow::glfw::GlfwContext ctx;
    const std::string data_path = LE_DATA_PATH;

    std::vector<TestCase> test_cases;

    {
        TestCase tc;
        tc.name = "pigcrazy3";
        tc.target_mesh_filename = data_path + "/models/target-meshes/pigtarget.obj";
        tc.layout_mesh_filename = data_path + "/models/layouts/pigcrazy3.obj";
        tc.view = glow::viewer::camera_transform(tg::pos3(5.687729f, 2.793668f, -3.253803f), tg::pos3(3.420963f, 1.650422f, -2.051982f));
        test_cases.push_back(tc);
    }
    {
        TestCase tc;
        tc.name = "centaur1crazy5";
        tc.target_mesh_filename = data_path + "/models/target-meshes/centaur1simple.obj";
        tc.layout_mesh_filename = data_path + "/models/layouts/centaur1crazy5.obj";
        tc.view = glow::viewer::camera_transform(tg::pos3(-3.003015f, -1.117670f, -3.858746f), tg::pos3(-1.600717f, -0.867071f, -2.207390f));
        test_cases.push_back(tc);
    }
    {
        TestCase tc;
        tc.name = "seahorse1crazy5";
        tc.target_mesh_filename = data_path + "/models/target-meshes/seahorse1target.obj";
        tc.layout_mesh_filename = data_path + "/models/layouts/seahorse1crazy5.obj";
        tc.view = glow::viewer::camera_transform(tg::pos3(3.134081f, 2.811929f, 3.790234f), tg::pos3(1.946006f, 1.656200f, 2.341523f));
        test_cases.push_back(tc);
    }
    {
        TestCase tc;
        tc.name = "cat2crazy3";
        tc.target_mesh_filename = data_path + "/models/target-meshes/cat2target.obj";
        tc.layout_mesh_filename = data_path + "/models/layouts/cat2crazy3.obj";
        tc.view = glow::viewer::camera_transform(tg::pos3(3.968053f, 0.351618f, 4.078722f), tg::pos3(2.327140f, 0.445463f, 2.521302f));
        test_cases.push_back(tc);
    }
    {
        TestCase tc;
        tc.name = "horse_challenge";
        tc.target_mesh_filename = data_path + "/models/target-meshes/horse_8078.obj";
        tc.layout_mesh_filename = data_path + "/models/layouts/horse_layout_praun_challenge.obj";
        tc.jitter_vertices = 5;
        tc.view = glow::viewer::camera_transform(tg::pos3(1.659233f, 0.582366f, 0.573250f), tg::pos3(1.007779f, 0.359270f, 0.402972f));
        test_cases.push_back(tc);
    }
    {
        TestCase tc;
        tc.name = "horse_easy";
        tc.target_mesh_filename = data_path + "/models/target-meshes/horse_8078.obj";
        tc.layout_mesh_filename = data_path + "/models/layouts/horse_layout.obj";
        tc.view = glow::viewer::camera_transform(tg::pos3(1.659233f, 0.582366f, 0.573250f), tg::pos3(1.007779f, 0.359270f, 0.402972f));
        test_cases.push_back(tc);
    }
    {
        TestCase tc;
        tc.name = "cuboid_twisted_inset";
        tc.target_mesh_filename = data_path + "/models/target-meshes/cuboid_16998.obj";
        tc.layout_mesh_filename = data_path + "/models/layouts/cuboid_twisted_inset.obj";
        tc.view = glow::viewer::camera_transform(tg::pos3(1.370662f, 0.989902f, -3.568955f), tg::pos3(0.976111f, 0.692004f, -2.699711f));
        test_cases.push_back(tc);
    }
    {
        TestCase tc;
        tc.name = "hand_plain_easy";
        tc.target_mesh_filename = data_path + "/models/target-meshes/hand_plain.obj";
        tc.layout_mesh_filename = data_path + "/models/layouts/hand_plain.obj";
        tc.view = glow::viewer::camera_transform(tg::pos3(-0.703929f, 0.441189f, 0.791465f), tg::pos3(-0.449164f, 0.256721f, 0.506854f));
        test_cases.push_back(tc);
    }
    {
        TestCase tc;
        tc.name = "hand_plain_challenge";
        tc.target_mesh_filename = data_path + "/models/target-meshes/hand_plain.obj";
        tc.layout_mesh_filename = data_path + "/models/layouts/hand_plain_crooked_finger.obj";
        tc.view = glow::viewer::camera_transform(tg::pos3(-0.703929f, 0.441189f, 0.791465f), tg::pos3(-0.449164f, 0.256721f, 0.506854f));
        test_cases.push_back(tc);
    }
    {
        TestCase tc;
        tc.name = "spot";
        tc.target_mesh_filename = data_path + "/models/target-meshes/spot_triangulated.obj";
        tc.layout_mesh_filename = data_path + "/models/layouts/spot_layout.obj";
        tc.view = glow::viewer::camera_transform(tg::pos3(-1.870854f, 0.707627f, -1.624399f), tg::pos3(-1.273037f, 0.460767f, -1.053504f));
        test_cases.push_back(tc);
    }

    for (const auto& tc : test_cases) {
        run_test_case(tc);
    }
}
