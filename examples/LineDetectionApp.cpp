/**
 * @file    LineDetectionApp.cpp
 *
 * @author  btran
 *
 * @date    2021-07-04
 *
 */

#include <iostream>

#include <pcl/io/pcd_io.h>

#include <3d_line_detection/3d_line_detection.hpp>

#include "Timer.hpp"
#include "Utility.hpp"

namespace
{
using PointCloudType = pcl::PointXYZ;
using PointCloud = pcl::PointCloud<PointCloudType>;
using PointCloudPtr = PointCloud::Ptr;
using HoughTransform = perception::HoughTransform<PointCloudType>;
using LineSegment3D = HoughTransform::LineSegment3D;

void addLine(const pcl::visualization::PCLVisualizer::Ptr& viewer, const LineSegment3D& lineSegment,
             const std::string& lineLabel, const std::string& projectedLabel, const std::array<double, 3>& color);

void getUsage(int argc, char* argv[]);

auto viewer = util::initializeViewer();

util::Timer timer;

}  // namespace

int main(int argc, char* argv[])
{
    // -------------------------------------------------------------------------
    // get parameters
    // -------------------------------------------------------------------------
    if (argc < 3) {
        ::getUsage(argc, argv);
        return EXIT_FAILURE;
    }

    std::size_t numRangeBin = 64;
    std::size_t sphereGranularity = 5;
    std::size_t minNumVote = 10;
    float distanceToLineThresh = 0.2;

    const bool useDefaultSetting = std::atoi(argv[2]);
    if (!useDefaultSetting) {
        if (argc != 7) {
            ::getUsage(argc, argv);
            return EXIT_FAILURE;
        }
        sphereGranularity = std::atoi(argv[4]);

        if (sphereGranularity >= geometry::Sphere::MAX_SUBDIVISION) {
            std::cerr << "sphere granularity needs to be less than " << geometry::Sphere::MAX_SUBDIVISION << std::endl;
            return EXIT_FAILURE;
        }

        numRangeBin = std::atoi(argv[3]);
        minNumVote = std::atoi(argv[5]);
        distanceToLineThresh = std::atof(argv[6]);
    }

    // -------------------------------------------------------------------------
    // processing
    // -------------------------------------------------------------------------
    const std::string pclFilePath = argv[1];
    PointCloudPtr cloud(new PointCloud);
    if (pcl::io::loadPCDFile(pclFilePath, *cloud) == -1) {
        std::cerr << "Failed to load pcl file" << std::endl;
        return EXIT_FAILURE;
    }

    viewer->addPointCloud<PointCloudType>(cloud, "original_cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "original_cloud");

    HoughTransform::Param param({.numRangeBin = numRangeBin,
                                 .sphereGranularity = sphereGranularity,
                                 .minNumVote = minNumVote,
                                 .distanceToLineThresh = distanceToLineThresh});
    HoughTransform transformer(param);

    pcl::ModelCoefficients lineModelCoefficients;

    timer.start();
    HoughTransform::LineSegment3Ds lineSegments = transformer.run(cloud);
    std::cout << "processing time: " << timer.getMs() << "[ms]\n";
    std::cout << "number of detected line segments: " << lineSegments.size() << std::endl;

    const std::vector<std::array<double, 3>> colors = util::generateColorCharts(lineSegments.size());
    int count = 0;
    for (const auto& lineSegment : lineSegments) {
        const std::string lineLabel = "line" + std::to_string(count);
        const std::string projectedLabel = "projected" + std::to_string(count);
        const auto& color = colors[count];
        ::addLine(viewer, lineSegment, lineLabel, projectedLabel, color);
        count++;
    }

    while (!viewer->wasStopped()) {
        viewer->spinOnce();
    }

    return EXIT_SUCCESS;
}

namespace
{
void addLine(const pcl::visualization::PCLVisualizer::Ptr& viewer, const LineSegment3D& lineSegment,
             const std::string& lineLabel, const std::string& projectedLabel, const std::array<double, 3>& color)
{
    auto projectedCloud = lineSegment.projectPointsOnLine();
    viewer->addLine(projectedCloud->points.front(), projectedCloud->points.back(), color[0], color[1], color[2],
                    lineLabel);
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 1, lineLabel);

    viewer->addPointCloud<PointCloudType>(projectedCloud, projectedLabel);
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, color[0], color[1], color[2],
                                             projectedLabel);
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, projectedLabel);
}

void getUsage(int argc, char* argv[])
{
    std::cerr << "Usage: [app] [path/to/pcd] [use/default/setting] [num/range/bin] [sphere/granularity] "
                 "[min/num/vote] [distance/to/line/thresh]"
              << std::endl;
}
}  // namespace
