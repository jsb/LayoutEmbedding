#include "shrec07.hh"
#include <LayoutEmbedding/Visualization/Visualization.hh>
#include <GLFW/glfw3.h>

using namespace LayoutEmbedding;

namespace
{

bool starts_with(const std::string& str, const std::string& prefix)
{
    return str.size() >= prefix.size() && 0 == str.compare(0, prefix.size(), prefix);
}

bool ends_with(const std::string& str, const std::string& suffix)
{
    return str.size() >= suffix.size() && 0 == str.compare(str.size() - suffix.size(), suffix.size(), suffix);
}

std::string prefix(const std::string& s, const std::string& delimiter)
{
    return s.substr(0, s.find(delimiter));
}

}

const int screenshot_samples = 64;

const auto input_dir = fs::path(LE_OUTPUT_PATH) / "shrec07_results/saved_embeddings";
const auto output_dir = fs::path(LE_OUTPUT_PATH) / "shrec07_figure";
const auto suffix = "smoothed.lem";
std::string open_prefix = "88";

struct ViewSettings
{
    std::string input_prefix;
    std::string output_suffix;
    glow::viewer::camera_transform cam_pos;
    tg::ivec2 screenshot_size;
    tg::vec3 rotation_euler;
    bool show_paths;
};

std::vector<ViewSettings> view_settings
{
    { // Standing human
        "10_kraevoy",
        "",
        glow::viewer::camera_transform(tg::pos3(-1.159352f, 0.489396f, 0.970308f), tg::pos3(-0.025531f, -0.012148f, 0.003359f)),
        tg::ivec2(1920, 1080),
        tg::vec3(0, 0, 0),
        true,
    },
    { // Standing human
        "10_kraevoy",
        "_zoom",
        glow::viewer::camera_transform(tg::pos3(-1.159352f, 0.489396f, 0.970308f), tg::pos3(-0.025531f, -0.012148f, 0.003359f)),
        tg::ivec2(1920, 1080) * 3,
        tg::vec3(0, 0, 0),
        true,
    },
    { // Standing human
        "10_bnb",
        "",
        glow::viewer::camera_transform(tg::pos3(-1.159352f, 0.489396f, 0.970308f), tg::pos3(-0.025531f, -0.012148f, 0.003359f)),
        tg::ivec2(1920, 1080),
        tg::vec3(0, 0, 0),
        true,
    },
    { // Standing human
        "10_bnb",
        "_zoom",
        glow::viewer::camera_transform(tg::pos3(-1.159352f, 0.489396f, 0.970308f), tg::pos3(-0.025531f, -0.012148f, 0.003359f)),
        tg::ivec2(1920, 1080) * 3,
        tg::vec3(0, 0, 0),
        true,
    },
    { // Sitting human
        "18_kraevoy",
        "",
        glow::viewer::camera_transform(tg::pos3(-1.177402f, 0.224309f, -0.281856f), tg::pos3(-0.787182f, 0.122423f, -0.185255f)),
        tg::ivec2(1920, 1080),
        tg::vec3(0, 0, 0),
        true,
    },
    { // Sitting human
        "18_bnb",
        "",
        glow::viewer::camera_transform(tg::pos3(-1.177402f, 0.224309f, -0.281856f), tg::pos3(-0.787182f, 0.122423f, -0.185255f)),
        tg::ivec2(1920, 1080),
        tg::vec3(0, 0, 0),
        true,
    },
    { // Sitting human
        "18_bnb",
        "_zoom_target",
        glow::viewer::camera_transform(tg::pos3(-1.177402f, 0.224309f, -0.281856f), tg::pos3(-0.787182f, 0.122423f, -0.185255f)),
        tg::ivec2(1920, 1080) * 4,
        tg::vec3(0, 0, 0),
        false,
    },
    { // Sitting human
        "18_bnb",
        "_zoom",
        glow::viewer::camera_transform(tg::pos3(-1.177402f, 0.224309f, -0.281856f), tg::pos3(-0.787182f, 0.122423f, -0.185255f)),
        tg::ivec2(1920, 1080) * 4,
        tg::vec3(0, 0, 0),
        true,
    },
    { // Glasses
        "56_bnb",
        "",
        glow::viewer::camera_transform(tg::pos3(-0.775124f, 0.869193f, 0.998298f), tg::pos3(-0.524951f, 0.528491f, 0.697396f)),
        tg::ivec2(1920, 1080),
        tg::vec3(0, 0, -90),
        true,
    },
    { // Glasses
        "56_praun",
        "",
        glow::viewer::camera_transform(tg::pos3(-0.775124f, 0.869193f, 0.998298f), tg::pos3(-0.524951f, 0.528491f, 0.697396f)),
        tg::ivec2(1920, 1080),
        tg::vec3(0, 0, -90),
        true,
    },
    { // Glasses
        "56_kraevoy",
        "",
        glow::viewer::camera_transform(tg::pos3(-0.775124f, 0.869193f, 0.998298f), tg::pos3(-0.524951f, 0.528491f, 0.697396f)),
        tg::ivec2(1920, 1080),
        tg::vec3(0, 0, -90),
        true,
    },
    { // Goat
        "385_praun",
        "",
        glow::viewer::camera_transform(tg::pos3(1.147075f, 0.318795f, 0.596279f), tg::pos3(0.059304f, -0.032330f, -0.000787f)),
        tg::ivec2(1920, 1080),
        tg::vec3(0, 0, 0),
        true,
    },
    { // Goat
        "385_kraevoy",
        "",
        glow::viewer::camera_transform(tg::pos3(1.147075f, 0.318795f, 0.596279f), tg::pos3(0.059304f, -0.032330f, -0.000787f)),
        tg::ivec2(1920, 1080),
        tg::vec3(0, 0, 0),
        true,
    },
    { // Goat
        "385_schreiner",
        "",
        glow::viewer::camera_transform(tg::pos3(1.147075f, 0.318795f, 0.596279f), tg::pos3(0.059304f, -0.032330f, -0.000787f)),
        tg::ivec2(1920, 1080),
        tg::vec3(0, 0, 0),
        true,
    },
    { // Goat
        "385_bnb",
        "",
        glow::viewer::camera_transform(tg::pos3(1.147075f, 0.318795f, 0.596279f), tg::pos3(0.059304f, -0.032330f, -0.000787f)),
        tg::ivec2(1920, 1080),
        tg::vec3(0, 0, 0),
        true,
    },
    { // Bearing
        "341_bnb",
        "",
        glow::viewer::camera_transform(tg::pos3(-1.053539f, 0.747928f, -0.390364f), tg::pos3(-0.729203f, 0.514880f, -0.269393f)),
        tg::ivec2(1920, 1080),
        tg::vec3(0, 0, 90),
        true,
    },
    { // Bearing
        "344_schreiner",
        "",
        glow::viewer::camera_transform(tg::pos3(0.621801f, 0.974902f, 0.925757f), tg::pos3(0.457668f, 0.770225f, 0.706502f)),
        tg::ivec2(1920, 1080),
        tg::vec3(0, 0, 0),
        true,
    },
    { // Bearing
        "344_bnb",
        "",
        glow::viewer::camera_transform(tg::pos3(0.621801f, 0.974902f, 0.925757f), tg::pos3(0.457668f, 0.770225f, 0.706502f)),
        tg::ivec2(1920, 1080),
        tg::vec3(0, 0, 0),
        true,
    },
    { // Giraffe
        "390_bnb",
        "",
        glow::viewer::camera_transform(tg::pos3(-1.041743f, 0.734063f, 1.134518f), tg::pos3(-0.000160f, 0.082310f, 0.029495f)),
        tg::ivec2(1920, 1080),
        tg::vec3(0, 0, 0),
        true,
    },
    { // Giraffe
        "390_kraevoy",
        "",
        glow::viewer::camera_transform(tg::pos3(-1.041743f, 0.734063f, 1.134518f), tg::pos3(-0.000160f, 0.082310f, 0.029495f)),
        tg::ivec2(1920, 1080),
        tg::vec3(0, 0, 0),
        true,
    },
    { // Armadillo
        "283_praun",
        "",
        glow::viewer::camera_transform(tg::pos3(0.178270f, 0.447518f, -1.393841f), tg::pos3(0.116999f, 0.274490f, -0.960800f)),
        tg::ivec2(1920, 1080),
        tg::vec3(0, 0, 0),
        true,
    },
    { // Armadillo
        "283_praun",
        "_zoom",
        glow::viewer::camera_transform(tg::pos3(0.178270f, 0.447518f, -1.393841f), tg::pos3(0.116999f, 0.274490f, -0.960800f)),
        tg::ivec2(1920, 1080) * 3,
        tg::vec3(0, 0, 0),
        true,
    },
    { // Armadillo
        "283_schreiner",
        "",
        glow::viewer::camera_transform(tg::pos3(0.178270f, 0.447518f, -1.393841f), tg::pos3(0.116999f, 0.274490f, -0.960800f)),
        tg::ivec2(1920, 1080),
        tg::vec3(0, 0, 0),
        true,
    },
    { // Armadillo
        "283_bnb",
        "",
        glow::viewer::camera_transform(tg::pos3(0.178270f, 0.447518f, -1.393841f), tg::pos3(0.116999f, 0.274490f, -0.960800f)),
        tg::ivec2(1920, 1080),
        tg::vec3(0, 0, 0),
        true,
    },
    { // Armadillo
        "283_bnb",
        "_zoom",
        glow::viewer::camera_transform(tg::pos3(0.178270f, 0.447518f, -1.393841f), tg::pos3(0.116999f, 0.274490f, -0.960800f)),
        tg::ivec2(1920, 1080) * 3,
        tg::vec3(0, 0, 0),
        true,
    },
    { // Armadillo
        "284_praun",
        "",
        glow::viewer::camera_transform(tg::pos3(0.218581f, 0.124159f, -1.514343f), tg::pos3(0.150450f, 0.028868f, -1.112853f)),
        tg::ivec2(1920, 1080),
        tg::vec3(0, 0, 0),
        true,
    },
    { // Armadillo
        "284_praun",
        "_zoom",
        glow::viewer::camera_transform(tg::pos3(0.218581f, 0.124159f, -1.514343f), tg::pos3(0.150450f, 0.028868f, -1.112853f)),
        tg::ivec2(1920, 1080) * 3,
        tg::vec3(0, 0, 0),
        true,
    },
    { // Armadillo
        "284_schreiner",
        "",
        glow::viewer::camera_transform(tg::pos3(0.218581f, 0.124159f, -1.514343f), tg::pos3(0.150450f, 0.028868f, -1.112853f)),
        tg::ivec2(1920, 1080),
        tg::vec3(0, 0, 0),
        true,
    },
    { // Armadillo
        "284_bnb",
        "",
        glow::viewer::camera_transform(tg::pos3(0.218581f, 0.124159f, -1.514343f), tg::pos3(0.150450f, 0.028868f, -1.112853f)),
        tg::ivec2(1920, 1080),
        tg::vec3(0, 0, 0),
        true,
    },
    { // Armadillo
        "284_bnb",
        "_zoom",
        glow::viewer::camera_transform(tg::pos3(0.218581f, 0.124159f, -1.514343f), tg::pos3(0.150450f, 0.028868f, -1.112853f)),
        tg::ivec2(1920, 1080) * 3,
        tg::vec3(0, 0, 0),
        true,
    },
    { // Bust
        "315_bnb",
        "",
        glow::viewer::camera_transform(tg::pos3(-0.093059f, 0.243482f, -1.088177f), tg::pos3(-0.064312f, 0.128970f, -0.765750f)),
        tg::ivec2(1920, 1080) / 4 * 3,
        tg::vec3(0, 0, 0),
        true,
    },
    { // Hand
        "199_schreiner",
        "",
        glow::viewer::camera_transform(tg::pos3(-1.196660f, 0.130550f, 0.641666f), tg::pos3(-0.871312f, 0.065849f, 0.472243f)),
        tg::ivec2(1920, 1080),
        tg::vec3(0, 0, 0),
        true,
    },
    { // Hand
        "199_schreiner",
        "_zoom",
        glow::viewer::camera_transform(tg::pos3(-1.196660f, 0.130550f, 0.641666f), tg::pos3(-0.871312f, 0.065849f, 0.472243f)),
        tg::ivec2(1920, 1080) * 3,
        tg::vec3(0, 0, 0),
        true,
    },
    { // Hand
        "199_bnb",
        "",
        glow::viewer::camera_transform(tg::pos3(-1.196660f, 0.130550f, 0.641666f), tg::pos3(-0.871312f, 0.065849f, 0.472243f)),
        tg::ivec2(1920, 1080),
        tg::vec3(0, 0, 0),
        true,
    },
    { // Ant
        "88_kraevoy",
        "",
        glow::viewer::camera_transform(tg::pos3(-0.282481f, 1.322519f, -0.863281f), tg::pos3(-0.219362f, 0.874872f, -0.629153f)),
        tg::ivec2(1920, 1080),
        tg::vec3(0, 0, -90),
        true,
    },
    { // Ant
        "88_praun",
        "",
        glow::viewer::camera_transform(tg::pos3(-0.282481f, 1.322519f, -0.863281f), tg::pos3(-0.219362f, 0.874872f, -0.629153f)),
        tg::ivec2(1920, 1080),
        tg::vec3(0, 0, -90),
        true,
    },
    { // Ant
        "88_schreiner",
        "",
        glow::viewer::camera_transform(tg::pos3(-0.282481f, 1.322519f, -0.863281f), tg::pos3(-0.219362f, 0.874872f, -0.629153f)),
        tg::ivec2(1920, 1080),
        tg::vec3(0, 0, -90),
        true,
    },
    { // Ant
        "88_bnb",
        "",
        glow::viewer::camera_transform(tg::pos3(-0.282481f, 1.322519f, -0.863281f), tg::pos3(-0.219362f, 0.874872f, -0.629153f)),
        tg::ivec2(1920, 1080),
        tg::vec3(0, 0, -90),
        true,
    },
};

std::optional<ViewSettings> find_settings(const fs::path& path)
{
    for (auto s : view_settings)
    {
        if (starts_with(path.filename(), s.input_prefix))
            return s;
    }

    return std::optional<ViewSettings>();
}

void view(const fs::path& path, const std::optional<ViewSettings>& settings, const bool layout, const bool screenshot)
{
    EmbeddingInput input;
    Embedding em(input);
    LE_ASSERT(em.load(path.parent_path() / path.stem()));

    // Apply transformation
    if (settings)
    {
        em.target_pos().apply([&] (auto& p)
        {
            p = tg::rotate_x(p, tg::angle::from_degree(settings->rotation_euler[0]));
            p = tg::rotate_x(p, tg::angle::from_degree(settings->rotation_euler[1]));
            p = tg::rotate_x(p, tg::angle::from_degree(settings->rotation_euler[2]));
        });
    }

    auto style = default_style();
    auto v = gv::view();

    // Apply view settings if available
    if (settings)
        v.configure(settings->cam_pos);

    if (screenshot)
    {
        // Screenshot mode
        const auto screenshot_path = output_dir /
                (path.filename().string() +
                 (layout ? "_layout" : "") +
                 (settings ? settings->output_suffix : "")
                 + ".png");
        fs::create_directories(output_dir);
        const auto screenshot_size = settings ? settings->screenshot_size : tg::ivec2(1920, 1080);
        v.configure(gv::headless_screenshot(screenshot_size, screenshot_samples, screenshot_path.string(), GL_RGBA8));
    }
    else
    {
        // Set caption
        gv::view(gv::make_renderable(std::vector<tg::pos3>()), gv::maybe_empty, path.filename());
    }

    if (layout)
    {
        view_layout(em);
    }
    else
    {
        if (settings && !settings->show_paths)
        {
            view_target_mesh(em);
            view_vertices_and_paths(em, false);
        }
        else
        {
            view_target(em);
        }
    }
}

void take_screenshots()
{
    for (const auto s : view_settings)
    {
        for (auto& f : fs::recursive_directory_iterator(input_dir))
        {
            if (starts_with(f.path().filename(), s.input_prefix) && ends_with(f.path(), suffix))
            {
                view(f.path(), s, false, true);
                view(f.path(), s, true, true);
            }
        }
    }
}

int main()
{
    register_segfault_handler();
    glow::glfw::GlfwContext ctx;

    take_screenshots();
}
