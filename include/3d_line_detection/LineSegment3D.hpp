/**
 * @file    LineSegment3D.hpp
 *
 * @author  btran
 *
 */

#pragma once

#include <pcl/ModelCoefficients.h>
#include <pcl/common/common.h>
#include <pcl/common/distances.h>
#include <pcl/filters/project_inliers.h>

#include <Eigen/Dense>

namespace geometry
{
template <typename POINT_CLOUD_TYPE> class LineSegment3D
{
 public:
    using PointCloudType = POINT_CLOUD_TYPE;
    using PointCloud = pcl::PointCloud<PointCloudType>;
    using PointCloudPtr = typename PointCloud::Ptr;

    LineSegment3D(const PointCloudPtr& cloud, const pcl::ModelCoefficients& coeffs)
        : m_cloud(cloud)
        , m_coeffs(coeffs)
        , m_inlierIndices()
    {
        if (m_coeffs.values.size() != 6) {
            throw std::runtime_error("invalid size of line coefficients");
        }
    }

    static std::vector<int> getPointIndicesCloseToLine(const PointCloudPtr& cloud, const pcl::ModelCoefficients& coeffs,
                                                       const float distanceToLineThresh,
                                                       const std::vector<char>& ignorePointIndices);

    bool refine(const float distanceToLineThresh, const std::vector<char>& ignorePointIndices);

    const auto& coeffs() const
    {
        return m_coeffs;
    }

    const auto& inlierIndices() const
    {
        return m_inlierIndices;
    }

    PointCloudPtr projectPointsOnLine(const bool sortAlongLinePositiveDirection = true) const;

 private:
    template <typename T>
    bool almostEquals(const T val, const T correctVal, const T epsilon = std::numeric_limits<T>::epsilon())
    {
        const T maxXYOne = std::max({static_cast<T>(1.0f), std::fabs(val), std::fabs(correctVal)});
        return std::fabs(val - correctVal) <= epsilon * maxXYOne;
    }

 private:
    const PointCloudPtr& m_cloud;
    pcl::ModelCoefficients m_coeffs;
    std::vector<int> m_inlierIndices;
};
}  // namespace geometry

#include "impl/LineSegment3D.ipp"
