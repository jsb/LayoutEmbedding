# LayoutEmbedding

This is an implementation of the method presented in "**Layout Embedding via Combinatorial Optimization**" at Eurographics 2021.

This repository contains:
* The core `LayoutEmbedding` library.
* Example applications that replicate experiments and figures from the Eurographics 2021 paper.
* Command-line utilities.

## Build Instructions

Make sure to checkout all Git submodules:
Clone via `git clone --recursive https://github.com/jsb/LayoutEmbedding.git`
or do `git submodule update --init --recursive` afterwards.

If not already present, install a C++17 compiler (GCC >= 8) and the following dependencies:
* CMake (`sudo apt install cmake`)
* OpenGL (`sudo apt install libgl1-mesa-dev mesa-utils`)
* GLFW build dependencies (`sudo apt install build-dep glfw3`)

```
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make -j4
```

## Replication of Results

Source code for experiments and figures in the paper is found in the `apps/eg2021` folder.
Run the following executables to replicate the results:

* `pig_figure` (Fig. 1)
* `homotopy_cube_figure` (Fig. 2)
* `quad_animals_figure` (Fig. 3)
* `optimization_timeline_figure` (Fig. 7)
* `jitter_evaluation_figure` (Fig. 13)
* `quad_hands_figure` (Fig. 14)
* `inter_surface_map_figure` (Fig. 15)
* `disk_face_figure` (Fig. 16)
* `disk_hand_figure` (Fig. 16)

Output files and images will be written to the `build/output` directory.
The above executables accept an optional `--viewer` argument to open an interactive viewer widget.
Executables that produce multiple images will open several viewer widgets successively.
Use the following controls:
* Center camera via double click.
* Rotate camera via left mouse drag.
* Close by pressing Esc.

To replicate the SHREC07 evaluation (Figs. 10, 11, and 12), run the following executables in the correct order.
Note that `shrec07_embed_layouts` takes ~24h to run.

* `shrec07_generate_layouts`
* `shrec07_embed_layouts` (data for Fig. 10, requires `shrec07_generate_layouts`)
* `shrec07_figure` (Fig. 11, requires `shrec07_embed_layouts`)
* `shrec07_ablation` (Fig. 12, requires `shrec07_generate_layouts`)

Run `shrec07_view` to inspect the results of `shrec07_embed_layouts`.
Use the left and right arrow keys to navigate through the results.
You can pass the SHREC07 mesh ID as a command line argument to start at a specific model.

## Command Line Interface

```
./embed <path-to-layout> <path-to-target> <flags>

<path-to-layout>:
    Layout connectivity as polygonal mesh (e.g. obj).
    Vertices are projected to target surface to define landmark positions.

<path-to-target>:
    Target triangle mesh (e.g. obj).

<flags>:
    --greedy: Run greedy algorithm, always choosing shortest path.
    --praun: Run greedy algorithm w/ heuristic based on [Praun2001].
    --kraevoy: Run greedy algorithm w/ heuristic based on [Kraevoy2003] / [Kraevoy2004].
    --schreiner: Run greedy algorithm w/ heuristic based on [Schreiner2004].

    --smooth: Smoothing post-process based on [Praun2001].

    --noview: Don't open viewer window.

Output files can be found in "build/output/embed".
```


```
./view_embedding <path_to_lem_file>

<path_to_lem_file>:
        Specify .lem file. E.g. in "build/output/embed"
```
