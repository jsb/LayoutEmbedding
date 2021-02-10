/**
  * Loads different embeddings on a cube from file.
  *
  * If "--viewer" is enabled, multiple windows will open successively.
  * Press ESC to close the current window.
  *
  * Output files can be found in <build-folder>/output/homotopy_cube_figure.
  */

#include <LayoutEmbedding/IO.hh>
#include <LayoutEmbedding/PathSmoothing.hh>
#include <LayoutEmbedding/Util/StackTrace.hh>
#include <LayoutEmbedding/Visualization/Visualization.hh>

#include <cxxopts.hpp>

using namespace LayoutEmbedding;
namespace fs = std::filesystem;

const auto screenshot_size = tg::ivec2(1920, 1080) * 3;
const int screenshot_samples = 64;
const auto cam_pos = glow::viewer::camera_transform(tg::pos3(5.743672f, 3.705137f, 5.629703f), tg::pos3(0.922786f, 0.451209f, 0.929011f));

int main(int argc, char** argv)
{
    register_segfault_handler();

    bool open_viewer = false;
    cxxopts::Options opts("homotopy_cube_figure", "Generates Fig. 2");
    opts.add_options()("v,viewer", "Open viewer widget", cxxopts::value<bool>()->default_value("false"));
    opts.add_options()("h,help", "Help");
    try {
        auto args = opts.parse(argc, argv);
        if (args.count("help")) {
            std::cout << opts.help() << std::endl;
            return 0;
        }
        open_viewer = args["viewer"].as<bool>();
    } catch (const cxxopts::OptionException& e) {
        std::cout << e.what() << "\n\n";
        std::cout << opts.help() << std::endl;
        return 1;
    }

    glow::glfw::GlfwContext ctx;

    // Paths
    const auto embedding_base_dir = fs::path(LE_DATA_PATH) / "embeddings/homotopy_cube";
    const auto output_dir = fs::path(LE_OUTPUT_PATH) / "homotopy_cube_figure";
    fs::create_directories(output_dir);

    std::vector<std::string> names
    {
        "embedding_original",
        "embedding_twist1",
        "embedding_twist2",
    };

    for (const auto name : names)
    {
        const auto embedding_path = embedding_base_dir / name / "homotopy_cube";

        EmbeddingInput input;
        Embedding em(input);
        em.load(embedding_path);

        if (name == "embedding_twist1")
        {
            // Smooth paths in y direction
            std::vector<pm::edge_handle> l_e_smooth;
            for (auto e : em.layout_mesh().edges())
            {
                if (fabs(tg::dot(tg::normalize(pm::edge_dir(e.halfedgeA(), em.layout_pos())), tg::vec3(0, 1, 0))) > 0.5)
                    l_e_smooth.push_back(e);
            }
            em = smooth_paths(em, l_e_smooth, 4, false);
        }
        else if (name == "embedding_twist2")
        {
            std::vector<pm::edge_handle> l_e_smooth;
            for (auto e : em.layout_mesh().edges())
            {
                if (e != pm::halfedge_from_to(em.layout_mesh().vertices()[3], em.layout_mesh().vertices()[7]).edge() &&
                    e != pm::halfedge_from_to(em.layout_mesh().vertices()[2], em.layout_mesh().vertices()[0]).edge() &&
                    e != pm::halfedge_from_to(em.layout_mesh().vertices()[0], em.layout_mesh().vertices()[1]).edge() &&
                    e != pm::halfedge_from_to(em.layout_mesh().vertices()[1], em.layout_mesh().vertices()[5]).edge())
                {
                    l_e_smooth.push_back(e);
                }
            }

            em = subdivide(em);
            em = smooth_paths(em, l_e_smooth, 1, false);
            em = subdivide(em);
            em = smooth_paths(em, l_e_smooth, 1, false);
        }

        // Layout screenshot
        {
            auto cfg_style = default_style();
            auto cfg_view = gv::config(cam_pos);
            const auto screenshot_path = output_dir / "layout.png";
            auto cfg_screenshot = gv::config(gv::headless_screenshot(screenshot_size, screenshot_samples, screenshot_path.string(), GL_RGBA8));
            view_layout(em, true, 30, 15);
        }

        // Embedding screenshot
        {
            auto cfg_style = default_style();
            auto cfg_view = gv::config(cam_pos);
            const auto screenshot_path = output_dir / (name + ".png");
            auto cfg_screenshot = gv::config(gv::headless_screenshot(screenshot_size, screenshot_samples, screenshot_path.string(), GL_RGBA8));
            view_target(em, true, 30, 15);
        }

        if (open_viewer)
        {
            auto cfg_style = default_style();
            view_embedding(em);
        }
    }
}
