#!/bin/bash

if [ "$#" -ne 1 ]; then
    echo "Usage: $0 <image_path>"
    exit 1
fi

image_path = $1

feature_extractor \
   --database_path database.db \
   --image_path $image_path \
   --ImageReader.camera_refrac_model FLATPORT \
   --ImageReader.camera_params "2288.7603402891391, 2292.9420044120843, 2591.7799618650656, 1956.6306474253697, 0.041273304517127421, 0.042048703033920672, -0.032473682157410581, 0.008218188026781691, 0.00010828732086971683, 3.577836369485064e-06" \
   --ImageReader.camera_refrac_params "0.0034340874001528117, 0.0053178590745536457, 0.99997996350855389, 0.017195078129667741, 0.0019, 1, 1.4730000000000001, 1.3340000000000001" \
   --ImageReader.single_camera 1 \
   --ImageReader.camera_model METASHAPE_FISHEYE

colmap_underwater exhaustive_matcher \
   --database_path database.db \
   --TwoViewGeometry.enable_refraction 1

mkdir sparse

colmap_underwater mapper \
    --database_path database.db \
    --image_path $image_path \
    --output_path sparse \
    --Mapper.ba_refine_focal_length 0 \
    --Mapper.ba_refine_principal_point 0 \
    --Mapper.ba_refine_extra_params 0 \
    --Mapper.enable_refraction 1 \
    --Mapper.ba_refine_refrac_params 0

colmap_underwater image_undistorter \
    --image_path $image_path \
    --input_path sparse/0 \
    --output_path dense \
    --output_type COLMAP

colmap_underwater patch_match_stereo \
    --workspace_path ./dense \
    --workspace_format COLMAP \
    --PatchMatchStereo.geom_consistency true

colmap_underwater stereo_fusion \
    --workspace_path dense \
    --workspace_format COLMAP \
    --input_type geometric \
    --output_path dense/fused.ply

colmap_underwater poisson_mesher \
    --input_path dense/fused.ply \
    --output_path dense/meshed-poisson.ply

colmap_underwater delaunay_mesher \
    --input_path dense \
    --output_path dense/meshed-delaunay.ply
