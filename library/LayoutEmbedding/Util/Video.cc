/*
 * Author: Patrick Schmidt
 */
#include "Video.hh"

#include <LayoutEmbedding/Util/Assert.hh>
#include <LayoutEmbedding/Util/Debug.hh>

#include <cstdio>

namespace LayoutEmbedding
{

std::string format_frame_number(int _frame_number)
{
    LE_ASSERT_GEQ(_frame_number, 0);
    std::string result = std::to_string(_frame_number);
    while (result.size() < 6) {
        result = "0" + result;
    }
    return result;
}

void render_video(
        const std::filesystem::path& _path,
        const int _fps,
        const std::string& _filename)
{
    // ffmpeg -r 15 -f image2 -i ...%06d.png -vcodec libx264 -pix_fmt yuv420p -crf 17 -tune animation -y animation.mp4

    // Convert png files to mp4
    const auto ffmpeg = "ffmpeg -r " + std::to_string(_fps) + " -f image2 -i "
            + (_path / "%06d.png").string()
            + " -vcodec libx264 -pix_fmt yuv420p -crf 17 -tune animation -y "
            + (_path / _filename).string();

    LE_DEBUG_OUT(ffmpeg);
    std::system(ffmpeg.c_str());
}

}
