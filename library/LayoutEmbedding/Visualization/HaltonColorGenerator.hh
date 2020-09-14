#pragma once

// Original author: Marcel Campen

#include <typed-geometry/types/color.hh>
#include <vector>

namespace LayoutEmbedding {

class HaltonColorGenerator
{
public:
    explicit HaltonColorGenerator(int _skip = 250);

    tg::color3 generate_next_color();
    std::vector<tg::color3> generate_next_colors(int _n);

private:
    double halton(int _index);
    double random_interval(int _index, double _min, double _max);
    tg::color3 hsl2rgb(double _h, double _sl, double _l);

    int current[3]; // current Halton index
    int bases[3]; // Halton prime bases
    double inverse_bases[3];
};

}
