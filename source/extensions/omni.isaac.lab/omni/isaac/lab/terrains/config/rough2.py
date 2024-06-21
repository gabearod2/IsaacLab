# Copyright (c) 2022-2024, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Configuration for custom terrains."""

import omni.isaac.lab.terrains as terrain_gen

from ..terrain_generator_cfg import TerrainGeneratorCfg

# Define the ObjectCfg instances
object_params_start = terrain_gen.MeshRepeatedObjectsTerrainCfg.ObjectCfg(
    num_objects=5,
    height=0.1
)

object_params_end = terrain_gen.MeshRepeatedObjectsTerrainCfg.ObjectCfg(
    num_objects=10,
    height=0.2
)

ROUGH_TERRAINS_CFG2 = TerrainGeneratorCfg(
    size=(8.0, 8.0),
    border_width=20.0,
    num_rows=10,
    num_cols=20,
    horizontal_scale=0.1,
    vertical_scale=0.005,
    slope_threshold=0.75,
    use_cache=False,
    sub_terrains={
        "rails_terrain": terrain_gen.MeshRailsTerrainCfg(
            proportion = 0,
            rail_thickness_range = (0.05, 0.23),
            rail_height_range = (0.05, 0.23),
            platform_width = 3.0,
        ),
                        "open_stairs_terrain": terrain_gen.MeshRepeatedObjectsTerrainCfg(
                            proportion = 0.2,
                            object_type = "box",
                            object_params_start=object_params_start,
                            object_params_end=object_params_end,
                            max_height_noise=0.1,
                            platform_width=2.0,
                        ),
        "pit_terrain": terrain_gen.MeshPitTerrainCfg(
            proportion = 0,
            pit_depth_range = (0.1,0.2),
        ),
        "pyramid_stairs": terrain_gen.MeshPyramidStairsTerrainCfg(
            proportion=0.2,
            step_height_range=(0.05, 0.23),
            step_width=0.3,
            platform_width=3.0,
            border_width=0,
            holes=False,
        ),
        "pyramid_stairs_inv": terrain_gen.MeshInvertedPyramidStairsTerrainCfg(
            proportion=0.2,
            step_height_range=(0.05, 0.23),
            step_width=0.3,
            platform_width=4.0,
            border_width=0,
            holes=False,
        ),
        "boxes": terrain_gen.MeshRandomGridTerrainCfg(
            proportion=0.1, grid_width=0.45, grid_height_range=(0.05, 0.2), platform_width=2.0
        ),
        "random_rough": terrain_gen.HfRandomUniformTerrainCfg(
            proportion=0.1, noise_range=(0.02, 0.10), noise_step=0.02, border_width=0.25
        ),
        "hf_pyramid_slope": terrain_gen.HfPyramidSlopedTerrainCfg(
            proportion=0.1, slope_range=(0.0, 0.8), platform_width=2.0, border_width=0.25
        ),
        "hf_pyramid_slope_inv": terrain_gen.HfInvertedPyramidSlopedTerrainCfg(
            proportion=0.1, slope_range=(0.0, 0.8), platform_width=2.0, border_width=0.25
        ),
    },
)
"""Rough terrains configuration."""
