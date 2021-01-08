/*
 * Author: Patrick Schmidt
 */
#include "Video.hh"

#include <LayoutEmbedding/Util/Debug.hh>

namespace LayoutEmbedding
{

void render_video(
        const std::filesystem::path& _path,
        const int _fps,
        const std::string& _filename)
{
    // ffmpeg -r 15 -f image2 -i ...%06d.png -vcodec libx264 -pix_fmt yuv420p -y animation.mp4

    // Convert png files to mp4
    const auto ffmpeg = "ffmpeg -r " + std::to_string(_fps) + " -f image2 -i "
            + (_path / "%06d.png").string()
            + " -vcodec libx264 -pix_fmt yuv420p -y "
            + (_path / _filename).string();

    LE_DEBUG_OUT(ffmpeg);
    std::system(ffmpeg.c_str());
}

}
