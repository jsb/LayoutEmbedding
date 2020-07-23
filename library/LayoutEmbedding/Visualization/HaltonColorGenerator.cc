#include "HaltonColorGenerator.hh"

#include <glow-extras/colors/color.hh> // For from_hsl

HaltonColorGenerator::HaltonColorGenerator(int _skip)
{
    // skip first 250 sequence elements to lower discrepancy even further.
    current[0] = _skip;
    current[1] = _skip;
    current[2] = _skip;

    // initialize prime bases for H,S,L. Especially the first should be small such that already
    // small numbers of generated colors are distributed over the whole color circle.
    bases[0] = 5;
    bases[1] = 13;
    bases[2] = 17;

    inverse_bases[0] = 1.0f / bases[0];
    inverse_bases[1] = 1.0f / bases[1];
    inverse_bases[2] = 1.0f / bases[2];
}

double HaltonColorGenerator::halton(int _index)
{
    int base = bases[_index];
    double inverse_base = inverse_bases[_index];
    double H = 0;
    double half = inverse_base;
    int I = current[_index];
    current[_index] += 1;
    while (I > 0) {
        int digit = I % base;
        H = H + half * digit;
        I = (int)(inverse_base * (I - digit));
        half *= inverse_base;
    }
    return H;
}

double HaltonColorGenerator::random_interval(int _index, double _min, double _max)
{
    return halton(_index) * (_max - _min) + _min;
}

tg::color3 HaltonColorGenerator::generate_next_color()
{
    double h = random_interval(0, 0.0, 0.9); // 0.9 instead of 1.0 to suppress natural bias towards red
    double s = random_interval(1, 0.4, 0.8); // saturation between 40% and 80%
    double l = random_interval(2, 0.3, 0.6); // lightness between 30% and 60%
    auto glow_color = glow::colors::color::from_hsl(360.0 * h, s, l);
    return tg::color3(glow_color.r, glow_color.g, glow_color.b);
}

std::vector<tg::color3> HaltonColorGenerator::generate_next_colors(int _n)
{
    std::vector<tg::color3> colors(_n);
    for (int i = 0; i < _n; ++i)
        colors[i] = generate_next_color();
    return colors;
}
