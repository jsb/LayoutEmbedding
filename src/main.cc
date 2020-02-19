#include <glow-extras/glfw/GlfwContext.hh>
#include <glow-extras/viewer/view.hh>

#include <imgui/imgui.h>

#include <polymesh/formats.hh>

#include <typed-geometry/tg.hh>

int main()
{
    glow::glfw::GlfwContext ctx;
    
    pm::Mesh m;
    auto pos = m.vertices().make_attribute<tg::pos3>();
    load("/data/models/objects/machine parts/rockerArm.off", m, pos);
    
    gv::view(pos, gv::no_grid, gv::no_outline);
}
