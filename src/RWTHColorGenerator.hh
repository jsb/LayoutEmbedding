#pragma once

#include <typed-geometry/types/color.hh>
#include <vector>

class RWTHColorGenerator
{
public:
    tg::color3 generate_next_color();
    std::vector<tg::color3> generate_next_colors(int n);

private:
    int current_index = 0;
};
