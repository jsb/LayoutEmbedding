#pragma once

#include <filesystem>
#include <fstream>
#include <vector>
#include <set>

// Where to find the input data: The SHREC07 dataset (with landmarks provided by BIM).
const std::filesystem::path shrec_dir = std::filesystem::path(LE_DATA_PATH) / "watertight_shrec07";
const std::filesystem::path shrec_corrs_dir = shrec_dir / "Corrs";
const std::filesystem::path shrec_meshes_dir = shrec_dir / "Meshes";

// Output directories for generated layout and results of the embedding experiments.
const std::filesystem::path shrec_output_dir = LE_OUTPUT_PATH;
const std::filesystem::path shrec_layouts_dir = shrec_output_dir / "shrec07_layouts";
const std::filesystem::path shrec_results_dir = shrec_output_dir / "shrec07_results";

const std::vector<int> shrec_categories = {
     1, // Human     (36 points each, 14 before)          1- 20
//     2, // Cup       (11 points each, same as before)    21- 40
     3, // Glasses   (10 points each, same as before)    41- 60
     4, // Plane     (11 points each, 8 before)          61- 80
     5, // Ant       (21 points each, 12 before)         81-100
     6, // Chair     (17 points each, 11 before)        101-120
     7, // Octopus   (18 points each, same as before)   121-140
     8, // Table     (18 points each, 13 before)        141-160
     9, // Teddy     (24 points each, 16 before)        161-180
    10, // Hand      (17 points each, 13 before)        181-200
    11, // Plier     ( 7 points each, same as before)   201-220
    12, // Fish      (11 points each, same as before)   221-240
    13, // Bird      (12 points each, same as before)   241-260
    14, // Spring    ( 2 points each, same as before)   261-280
    15, // Armadillo (28 points each, 20 before)        281-300
    16, // Bust      (11 points each, 9 before)         301-320
    17, // Mech      (16 points each, same as before)   321-340
    18, // Bearing   ( 8 points each, same as before)   341-360
    19, // Vase      (missing landmarks?)               361-380
    20, // Fourleg   (21 points each, 15 before)        381-400
};

const std::vector<int> shrec_num_landmarks_bim = {
    36,
    11,
    10,
    11,
    21,
    17,
    18,
    18,
    24,
    17,
    7,
    11,
    12,
    2,
    28,
    11,
    16,
    8,
    0,
    21,
};

const std::set<int> shrec_flipped_landmarks = {
    // TODO: Table?
    183, 184, 185, 186, 190, 191, 193, 194, 195, 196, 197, 200, // Hands
};

const int shrec_meshes_per_category = 20;
