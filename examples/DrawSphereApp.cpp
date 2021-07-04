/**
 * @file    DrawSphereApp.cpp
 *
 * @author  btran
 *
 */

#include "Utility.hpp"

#include <3d_line_detection/3d_line_detection.hpp>

namespace
{
void addSphere(const pcl::visualization::PCLVisualizer::Ptr& viewer, const geometry::Sphere& sphere,
               const float scaleFactor = 1.);
}  // namespace

int main(int argc, char* argv[])
{
    if (argc != 2) {
        std::cerr << "Usage: [app] [num/subdivision]" << std::endl;
        return EXIT_FAILURE;
    }

    const int numSubvision = std::atoi(argv[1]);
    if (numSubvision >= geometry::Sphere::MAX_SUBDIVISION) {
        std::cerr << "number of subdivision must be less than " << geometry::Sphere::MAX_SUBDIVISION << std::endl;
        return EXIT_FAILURE;
    }

    auto viewer = util::initializeViewer();

    const float scaleFactor = 2.0;
    geometry::Sphere sphere = geometry::Sphere::SPHERE_LOOK_UP_TABLE[numSubvision];
    std::cout << "number of vertices: " << sphere.vertices().size() << "\n";

    ::addSphere(viewer, sphere, scaleFactor);

    while (!viewer->wasStopped()) {
        viewer->spinOnce();
    }

    return EXIT_SUCCESS;
}

namespace
{
void addSphere(const pcl::visualization::PCLVisualizer::Ptr& viewer, const geometry::Sphere& sphere,
               const float scaleFactor)
{
    const std::vector<std::array<double, 3>> colorCharts = util::generateColorCharts(sphere.triangles().size());

    int i = 0;
    std::vector<std::string> alreadyDrawn;

    for (const auto& curTriangle : sphere.triangles()) {
        const std::array<double, 3>& curColor = colorCharts[i++];

        std::vector<pcl::PointXYZ> v(3);

        for (int j = 0; j < 3; ++j) {
            v[j].x = sphere.vertices()[curTriangle[j]][0] * scaleFactor;
            v[j].y = sphere.vertices()[curTriangle[j]][1] * scaleFactor;
            v[j].z = sphere.vertices()[curTriangle[j]][2] * scaleFactor;
        }

        for (int j = 0; j < 3; ++j) {
            for (int k = j + 1; k < 3; ++k) {
                std::string lineLabel = std::to_string(curTriangle[j]) + std::to_string(curTriangle[k]);
                if (std::find(alreadyDrawn.begin(), alreadyDrawn.end(), lineLabel) != alreadyDrawn.end()) {
                    continue;
                }
                viewer->addLine(v[j], v[k], curColor[0], curColor[1], curColor[2], lineLabel);
                alreadyDrawn.emplace_back(std::move(lineLabel));
            }
        }
    }
}
}  // namespace
