/**
 * @file    Utility.hpp
 *
 * @author  btran
 *
 */

#pragma once

#include <pcl/visualization/pcl_visualizer.h>

namespace util
{
inline pcl::visualization::PCLVisualizer::Ptr initializeViewer()
{
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    pcl::PointXYZ o(0.1, 0, 0);
    viewer->setBackgroundColor(0.05, 0.05, 0.05, 0);
    viewer->addCoordinateSystem(0.5);
    viewer->setCameraPosition(-26, 0, 3, 10, -1, 0.5, 0, 0, 1);

    return viewer;
}

inline std::vector<std::array<double, 3>> generateColorCharts(const std::uint16_t numSources,
                                                              const std::uint16_t seed = 2021)
{
    std::srand(seed);
    std::vector<std::array<double, 3>> colors(numSources);
    for (std::uint16_t i = 0; i < numSources; ++i) {
        colors[i] =
            std::array<double, 3>{(std::rand() % 256) / 255., (std::rand() % 256) / 255., (std::rand() % 256) / 255.};
    }
    return colors;
}
}  // namespace util
