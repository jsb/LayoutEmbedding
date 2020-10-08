#pragma once

#include <LayoutEmbedding/Util/Debug.hh>

#include <glow-extras/viewer/configure.hh>

#include <typed-geometry/functions/basic/mix.hh>

#include <array>
#include <cmath>
#include <vector>

namespace LayoutEmbedding {

template <typename T>
T
interpolate(const T& _a, const T& _b, double _t)
{
    return tg::mix(_a, _b, _t);
}

template <>
glow::viewer::camera_transform
interpolate<glow::viewer::camera_transform>(const glow::viewer::camera_transform& _a, const glow::viewer::camera_transform& _b, double _t)
{
    return glow::viewer::camera_transform{ tg::mix(_a.pos, _b.pos, _t), mix(_a.target, _b.target, _t) };
}

template<typename T, int Degree>
T
uniform_de_boor(const std::vector<T>& _cp, double _t)
{
    const int k = static_cast<int>(_t); // "Knot interval"

    // Obtain the relevant set of control points.
    //std::array<T, Degree + 1> local_cp;
    std::vector<T> local_cp;
    for (int i = 0; i < Degree + 1; ++i) {
        int local_cp_i = i + k - Degree;
        // If local_cp_i lies outside the range of provided control points, we clamp to the first / last value.
        local_cp_i = std::max(local_cp_i, 0);
        local_cp_i = std::min(local_cp_i, static_cast<int>(_cp.size() - 1));
        //local_cp[i] = _cp[local_cp_i];
        local_cp.push_back(_cp[local_cp_i]);
    }

    // De Boor's recursion
    for (int r = 1; r < Degree + 1; ++r) {
        for (int j = Degree; j > r - 1; --j) {
            const double alpha = (_t - j - k + Degree) / (1 - r + Degree);
            local_cp[j] = interpolate(local_cp[j - 1], local_cp[j], alpha);
        }
    }
    return local_cp[Degree];
}

/// Uniform B-spline.
/// - Fill cp with at least two control points.
/// - eval(_t) takes arguments in [0, 1], regardless of number of control points.
/// - The spline will interpolate the first and last control point.
/// - Intermediate control points influence the curve but are not interpolated, in general.
/// - If you want to interpolate an intermediate control point, repeat it Degree times.
/// - If you need an estimate of the animation duration (i.e. number of uniform intervals), use duration().
template<typename T, int Degree = 3>
struct BSplineAnimation
{
    std::vector<T> cp;

    explicit BSplineAnimation(const std::initializer_list<T>& _cps) :
        cp(_cps)
    {
    }

    explicit BSplineAnimation(const std::vector<T>& _cps) :
        cp(_cps)
    {
    }

    BSplineAnimation() = default;
    BSplineAnimation(const BSplineAnimation&) = default;
    BSplineAnimation(BSplineAnimation&&) = default;
    BSplineAnimation& operator=(const BSplineAnimation&) = default;
    BSplineAnimation& operator=(BSplineAnimation&&) = default;

    /// _t in [0, 1].
    T eval(double _t) const
    {
        const double uniform_t_min = 1.0; // Where the influence of the second control point begins
        const double uniform_t_max = static_cast<int>(cp.size()) + Degree - 1; // Where the influence of the penultimate control point ends
        const double uniform_t = uniform_t_min + _t * (uniform_t_max - uniform_t_min);
        return uniform_de_boor<T, Degree>(cp, uniform_t);
    }

    /// Gives an estimate of the duration of the animation.
    /// This is the range of uniform B-spline parameter values that are traversed as _t in eval(_t) ranges from 0 to 1.
    double duration() const
    {
        return static_cast<int>(cp.size()) + Degree - 2.0;
    }

    constexpr int degree() const
    {
        return Degree;
    }

    bool valid() const
    {
        return cp.size() >= 2;
    }

    struct Frame
    {
        int i;
        double t;
    };

    /// Usage:
    ///     for (const auto& [i, t] : anim.frames(100)) {
    ///         result[i] = anim.eval(t);
    ///     }
    std::vector<Frame> frames(int _n_frames) const
    {
        if (_n_frames <= 0) {
            return {};
        }
        if (_n_frames == 1) {
            return {{0, 0.0}};
        }

        std::vector<Frame> result;
        for (int i = 0; i < _n_frames; ++i) {
            const auto t = static_cast<double>(i) / (_n_frames - 1);
            result.push_back({i, t});
        }
        return result;
    }
};

}
