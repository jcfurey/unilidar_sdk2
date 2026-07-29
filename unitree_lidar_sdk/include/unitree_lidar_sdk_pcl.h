/**********************************************************************
 Copyright (c) 2020-2024, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/

#pragma once

#ifndef PCL_NO_PRECOMPILE
#define PCL_NO_PRECOMPILE
#endif

// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/impl/search.hpp>
#include <pcl/range_image/range_image.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/registration/icp.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>

#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/passthrough.h>

#include <pcl/impl/instantiate.hpp>
#include <pcl/impl/point_types.hpp>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/impl/voxel_grid.hpp>
#include <pcl/filters/impl/approximate_voxel_grid.hpp>
#include <pcl/filters/impl/radius_outlier_removal.hpp>
#include <pcl/filters/impl/conditional_removal.hpp>
#include <pcl/filters/impl/passthrough.hpp>

// Eigen
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <cstdint>

#include "unitree_lidar_sdk.h"

// Pulling a whole namespace into the global one from a header leaks into every
// translation unit that includes it. It is kept for backwards compatibility -
// define UNILIDAR_SDK_PCL_NO_GLOBAL_NAMESPACE before including this header to
// opt out and qualify the SDK types explicitly instead.
#ifndef UNILIDAR_SDK_PCL_NO_GLOBAL_NAMESPACE
using namespace unilidar_sdk2;  // NOLINT(build/namespaces)
#endif

/**
 * @brief PCL Point Type
 */
struct PointType
{
    PCL_ADD_POINT4D
    PCL_ADD_INTENSITY
    std::uint16_t ring;
    float time;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT(PointType,
                                  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(std::uint16_t, ring, ring)(float, time, time))

PCL_INSTANTIATE(VoxelGrid, PointType)
PCL_INSTANTIATE(KdTree, PointType)
PCL_INSTANTIATE(RadiusOutlierRemoval, PointType)

/**
 * @brief Transform a Unitree cloud to PCL cloud
 *
 * @param cloudIn
 * @param cloudOut
 *
 * @note This has to be `inline`: without it, including this header from more
 * than one translation unit makes the link fail with a duplicate definition.
 */
inline void transformUnitreeCloudToPCL(const PointCloudUnitree &cloudIn, pcl::PointCloud<PointType>::Ptr cloudOut)
{
    const size_t num_points = cloudIn.points.size();

    // Size the cloud once instead of growing it one point at a time.
    cloudOut->clear();
    cloudOut->resize(num_points);
    cloudOut->width = static_cast<std::uint32_t>(num_points);
    cloudOut->height = 1;
    cloudOut->is_dense = true;

    for (size_t i = 0; i < num_points; i++)
    {
        PointType &pt = cloudOut->points[i];
        pt.x = cloudIn.points[i].x;
        pt.y = cloudIn.points[i].y;
        pt.z = cloudIn.points[i].z;
        pt.intensity = cloudIn.points[i].intensity;
        pt.time = cloudIn.points[i].time;
        pt.ring = static_cast<std::uint16_t>(cloudIn.points[i].ring);
        // PCL keeps a homogeneous coordinate in the 4th slot of the XYZ block and
        // several of its Eigen based algorithms expect it to be 1. It used to be
        // left holding whatever was on the stack.
        pt.data[3] = 1.0f;
    }
}