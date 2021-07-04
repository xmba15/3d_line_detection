/**
 * @file    HoughTransform.hpp
 *
 * @author  btran
 *
 */

#pragma once

#include <vector>

#include <pcl/ModelCoefficients.h>
#include <pcl/common/common.h>

#include "LineSegment3D.hpp"
#include "Sphere.hpp"

namespace perception
{
template <typename POINT_CLOUD_TYPE> class HoughTransform
{
 public:
    using PointCloudType = POINT_CLOUD_TYPE;
    using PointCloud = pcl::PointCloud<PointCloudType>;
    using PointCloudPtr = typename PointCloud::Ptr;
    using Sphere = geometry::Sphere;
    using LineSegment3D = geometry::LineSegment3D<PointCloudType>;
    using LineSegment3Ds = std::vector<LineSegment3D>;

    struct Param {
        std::size_t numRangeBin = 64;
        std::size_t sphereGranularity = 5;  // value from 0 to geometry::Sphere::MAX_SUBDIVISION-1
        std::size_t minNumVote = 10;
        float distanceToLineThresh = 0.2;
    };

    explicit HoughTransform(const Param& param);

    LineSegment3Ds run(const PointCloudPtr& cloud);

 private:
    int getLine(pcl::ModelCoefficients& coeffs, const float minRange, const float deltaRange,
                const std::vector<std::size_t>& accumulatorCellIndices) const;

    float calcNorm(const PointCloudType& point) const;

    void votePoint(const PointCloudType& point, const float minRange, const float deltaRange, const bool toAdd);

 private:
    Param m_param;
    std::vector<std::size_t> m_accumulator;
    const Sphere& m_sphere;
};
}  // namespace perception

#include "impl/HoughTransform.ipp"
