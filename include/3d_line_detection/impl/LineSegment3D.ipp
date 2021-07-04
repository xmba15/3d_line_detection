/**
 * @file    LineSegment3D.ipp
 *
 * @author  btran
 *
 */

namespace geometry
{
template <typename POINT_CLOUD_TYPE>
typename LineSegment3D<POINT_CLOUD_TYPE>::PointCloudPtr
LineSegment3D<POINT_CLOUD_TYPE>::projectPointsOnLine(const bool sortAlongLinePositiveDirection) const
{
    PointCloudPtr extracted(new PointCloud);
    pcl::copyPointCloud(*m_cloud, m_inlierIndices, *extracted);
    pcl::ProjectInliers<PointCloudType> projector;
    projector.setModelType(pcl::SACMODEL_LINE);
    projector.setInputCloud(extracted);
    projector.setModelCoefficients(pcl::ModelCoefficients::Ptr(new pcl::ModelCoefficients(m_coeffs)));
    projector.filter(*extracted);

    if (sortAlongLinePositiveDirection) {
        auto calcSignedDistToPlanePoint = [this](const PointCloudType& p) {
            return (p.x - m_coeffs.values[0]) * m_coeffs.values[3] + (p.y - m_coeffs.values[1]) * m_coeffs.values[4] +
                   (p.z - m_coeffs.values[2]) * m_coeffs.values[5];
        };
        std::sort(extracted->points.begin(), extracted->points.end(),
                  [calcSignedDistToPlanePoint](const auto& e1, const auto& e2) {
                      return calcSignedDistToPlanePoint(e1) < calcSignedDistToPlanePoint(e2);
                  });
    }

    return extracted;
}

template <typename POINT_CLOUD_TYPE>
std::vector<int> LineSegment3D<POINT_CLOUD_TYPE>::getPointIndicesCloseToLine(
    const PointCloudPtr& cloud, const pcl::ModelCoefficients& coeffs, const float distanceToLineThresh,
    const std::vector<char>& ignorePointIndices)
{
    if (cloud->empty()) {
        return {};
    }

    if (ignorePointIndices.size() != cloud->size()) {
        throw std::runtime_error("sizes mistmach");
    }

    if (coeffs.values.size() != 6) {
        throw std::runtime_error("invalid size of line model coefficients");
    }

    std::vector<int> pointIndices;

    const Eigen::Vector4f linePoint(coeffs.values[0], coeffs.values[1], coeffs.values[2], 0);
    const Eigen::Vector4f lineDirection(coeffs.values[3], coeffs.values[4], coeffs.values[5], 0);

    const double sqrLength = lineDirection.squaredNorm();
    Eigen::Vector4f eigenPoint;

    for (std::size_t i = 0; i < cloud->size(); ++i) {
        if (ignorePointIndices[i]) {
            continue;
        }
        const auto& curPoint = cloud->points[i];
        eigenPoint << curPoint.x, curPoint.y, curPoint.z, 0;
        if (pcl::sqrPointToLineDistance(eigenPoint, linePoint, lineDirection, sqrLength) < distanceToLineThresh) {
            pointIndices.emplace_back(i);
        }
    }

    return pointIndices;
}

template <typename POINT_CLOUD_TYPE>
bool LineSegment3D<POINT_CLOUD_TYPE>::refine(const float distanceToLineThresh,
                                             const std::vector<char>& ignorePointIndices)
{
    const auto pointIndices =
        LineSegment3D::getPointIndicesCloseToLine(m_cloud, m_coeffs, distanceToLineThresh, ignorePointIndices);

    if (pointIndices.empty()) {
        return false;
    }

    Eigen::MatrixXf pointMatrix = Eigen::MatrixXf::Constant(pointIndices.size(), 3, 0);

    int count = 0;
    for (const int pointIdx : pointIndices) {
        const auto& curPoint = m_cloud->points[pointIdx];
        pointMatrix.row(count++) << curPoint.x, curPoint.y, curPoint.z;
    }

    auto centroid = pointMatrix.colwise().mean();
    Eigen::MatrixXf centeredMatrix = pointMatrix.rowwise() - centroid;
    Eigen::MatrixXf scatterMatrix = (centeredMatrix.adjoint() * centeredMatrix);

    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXf> eig(scatterMatrix);
    if (this->almostEquals<float>(eig.eigenvalues()(2), 0)) {
        return false;
    }

    Eigen::MatrixXf eigVecs = eig.eigenvectors();
    m_coeffs.values[0] = centroid.x();
    m_coeffs.values[1] = centroid.y();
    m_coeffs.values[2] = centroid.z();
    m_coeffs.values[3] = eigVecs(0, 2);
    m_coeffs.values[4] = eigVecs(1, 2);
    m_coeffs.values[5] = eigVecs(2, 2);
    m_inlierIndices = std::move(pointIndices);

    return true;
}
}  // namespace geometry
