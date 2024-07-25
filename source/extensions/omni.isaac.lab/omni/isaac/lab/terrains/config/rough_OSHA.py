# Copyright (c) 2022-2024, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Configuration for custom terrains."""

import omni.isaac.lab.terrains as terrain_gen

from ..terrain_generator_cfg import TerrainGeneratorCfg, FlatPatchSamplingCfg

"""
Edited to include the optimal training environments for the power plant conditions, and finding flat patches for 
training navigation. 
- Gabriel Rodriguez
"""

ROUGH_TERRAINS_CFG_OSHA = TerrainGeneratorCfg(
    size=(8.0, 8.0),
    border_width=20.0,
    num_rows=10,
    num_cols=20,
    horizontal_scale=0.1,
    vertical_scale=0.005,
    slope_threshold=0.75,
    use_cache=False,
    sub_terrains={
        "flat_terrain": terrain_gen.MeshPlaneTerrainCfg(
            proportion=0.25,
            flat_patch_sampling={
                "target": FlatPatchSamplingCfg(
                    num_patches=1,
                    patch_radius=4.0,
                    max_height_diff=0.1,
                )
            }
        ),
        "pyramid_open_stairs": terrain_gen.MeshPyramidOpenStairsTerrainCfg(
            proportion=0.25,
            step_height=0.05,
            gap_height_range=(0.08, 0.24),
            step_width_range=(0.24, 0.50),
            platform_width=2.0,
            border_width=0.1,
            holes=False,
            flat_patch_sampling={
                "target": FlatPatchSamplingCfg(
                    num_patches=1,
                    patch_radius=1.0,
                    max_height_diff=0.1,
                )
            }
        ),
        "pyramid_open_stairs_inv": terrain_gen.MeshInvertedPyramidOpenStairsTerrainCfg(
            proportion=0.25,
            step_height=0.05,
            gap_height_range=(0.08, 0.24),
            step_width_range=(0.24, 0.50),
            platform_width=2.0,
            border_width=0.1,
            holes=False,
            flat_patch_sampling={
                "target": FlatPatchSamplingCfg(
                    num_patches=1,
                    patch_radius=1.0,
                    max_height_diff=0.1,
                )
            }
        ),
        "pyramid_stairs": terrain_gen.MeshPyramidStairsTerrainCfg(
            proportion=0.2,
            step_height_range=(0.13, 0.29),
            step_width_range=(0.24, 0.50),
            platform_width=2.0,
            border_width=0.1,
            holes=False,
            flat_patch_sampling={
                "target": FlatPatchSamplingCfg(
                    num_patches=1,
                    patch_radius=1.0,
                    max_height_diff=0.1,
                )
            }
        ),
        "pyramid_stairs_inv": terrain_gen.MeshInvertedPyramidStairsTerrainCfg(
            proportion=0.2,
            step_height_range=(0.13, 0.29),
            step_width_range=(0.24, 0.50),
            platform_width=2.0,
            border_width=0.1,
            holes=False,
            flat_patch_sampling={
                "target": FlatPatchSamplingCfg(
                    num_patches=1,
                    patch_radius=1.0,
                    max_height_diff=0.1,
                )
            }
        ),
        "boxes": terrain_gen.MeshRandomGridTerrainCfg(
            proportion=0.2, grid_width=0.45, grid_height_range=(0.05, 0.2), platform_width=2.0,
            flat_patch_sampling={
                "target": FlatPatchSamplingCfg(
                    num_patches=1,
                    patch_radius=1.0,
                    max_height_diff=0.1,
                )
            }
        ),
        "random_rough": terrain_gen.HfRandomUniformTerrainCfg(
            proportion=0.2, noise_range=(0.02, 0.10), noise_step=0.02, border_width=0.5,
            flat_patch_sampling={
                "target": FlatPatchSamplingCfg(
                    num_patches=1,
                    patch_radius=0.25,
                    max_height_diff=0.1,
                )
            }
        ),
        # "hf_pyramid_slope": terrain_gen.HfPyramidSlopedTerrainCfg(
        #     proportion=0.2, slope_range=(0.2, 0.3), platform_width=0, border_width=0.1,
        #     flat_patch_sampling={
        #         "slope_patches": FlatPatchSamplingCfg(
        #             num_patches=1,
        #             patch_radius=4.0,
        #             max_height_diff=0.1,
        #         )
        #     }
        # ),
        # "hf_pyramid_slope_inv": terrain_gen.HfInvertedPyramidSlopedTerrainCfg(
        #     proportion=0.2, slope_range=(0.2, 0.3), platform_width=0, border_width=0.1,
        #     flat_patch_sampling={
        #         "slope_inv_patches": FlatPatchSamplingCfg(
        #             num_patches=1,
        #             patch_radius=4.0,
        #             max_height_diff=0.1,
        #         )
        #     }
        # ),
    },
)
"""Rough terrains configuration."""