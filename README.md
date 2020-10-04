# Prerequisites

This code was tested on Linux (Debian).

If not already present, install a C++17 compiler (GCC >= 8) and the following dependencies:
* CMake (`sudo apt-get install cmake`)
* OpenGL (`sudo apt-get install libgl1-mesa-dev mesa-utils`)
* GLFW build dependencies (`sudo apt-get build-dep glfw3`)

# Build Instructions

```
mkdir build
cd build
cmake ../
make -j4
```

# Run Experiments

Run the following executables to replicate individual experiments:

```
./teaser_pig
./homotopy_cube_figure
./quad_animals_figure
./optimization_timeline_figure
./jitter_evaluation
./quad_hands_figure
./inter_surface_map_figure
./disk_face_figure
./disk_hand_figure
```

Output files can be found at "build/output".
Some executables open one or multiple viewer windows:
* Rotate camera via left mouse drag.
* Center camera via double click.
* Close by pressing ESC.

Edit the respective file in the "applications" folder to surpress the viewer window.

To replicate the SHREC07 evaluation, run the following executables in the correct order:

```
./shrec07_generate_layouts
./shrec07_embed_layouts
./shrec07_figure
./shrec07_ablation
```

Note that `shrec07_embed_layouts` takes ~24h to run.
 
Run `./shrec07_view` to inspect the results of `shrec07_embed_layouts`.
Use the left and right arrow keys to navigate through the results.
Edit applications/shrec07_view.cc to start the viewer at a different result.

# Command Line Interface

```
./embed <path-to-layout> <path-to-target> <flags>

<path-to-layout>:
    Layout connectivity as polygonal mesh (e.g. obj).
    Vertices are projected to target surface to define landmark positions.

path-to-target:
    Target triangle mesh (e.g. obj).

flags:
    --greedy: Run greedy algorithm, always choosing shortest path.
    --praun: Run greedy algorithm w/ heuristic based on [Praun2001].
    --kraevoy: Run greedy algorithm w/ heuristic based on [Kraevoy2003] / [Kraevoy2004].
    --schreiner: Run greedy algorithm w/ heuristic based on [Schreiner2004].

    --smooth: Smoothing post-process based on [Praun2001].

    --view: Open viewer window.

Output files can be found in "build/output/embed".
```


```
./view_embedding <path_to_lem_file>

path_to_lem_file:
	Specify .lem file. E.g. in "build/output/embed"
```
