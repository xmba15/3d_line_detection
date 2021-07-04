/**
 * @file    HoughTransform.ipp
 *
 * @author  btran
 *
 */

namespace perception
{
template <typename POINT_CLOUD_TYPE>
HoughTransform<POINT_CLOUD_TYPE>::HoughTransform(const Param& param)
    : m_param(param)
    , m_accumulator()
    , m_sphere(Sphere::SPHERE_LOOK_UP_TABLE[m_param.sphereGranularity])
{
}

template <typename POINT_CLOUD_TYPE>
void HoughTransform<POINT_CLOUD_TYPE>::votePoint(const PointCloudType& point, const float minRange,
                                                 const float deltaRange, const bool toAdd)
{
    for (std::size_t i = 0; i < m_sphere.vertices().size(); ++i) {
        const Sphere::Vertex& b = m_sphere.vertices()[i];
        double beta = 1 / (1 + b[2]);

        double xNew = ((1 - (beta * (b[0] * b[0]))) * point.x) - ((beta * (b[0] * b[1])) * point.y) - (b[0] * point.z);
        double yNew = ((-beta * (b[0] * b[1])) * point.x) + ((1 - (beta * (b[1] * b[1]))) * point.y) - (b[1] * point.z);
        std::size_t xIndex = (xNew - minRange) / deltaRange;
        std::size_t yIndex = (yNew - minRange) / deltaRange;

        std::size_t index =
            (xIndex * m_param.numRangeBin * m_sphere.vertices().size()) + (yIndex * m_sphere.vertices().size()) + i;

        if (index < m_accumulator.size()) {
            toAdd ? m_accumulator[index]++ : m_accumulator[index]--;
        }
    }
}

template <typename POINT_CLOUD_TYPE>
int HoughTransform<POINT_CLOUD_TYPE>::getLine(pcl::ModelCoefficients& coeffs, const float minRange,
                                              const float deltaRange,
                                              const std::vector<std::size_t>& accumulatorCellIndices) const
{
    coeffs.values.resize(6);

    if (accumulatorCellIndices.empty()) {
        return -1;
    }

    auto maxIndicesIt = std::max_element(
        accumulatorCellIndices.begin(), accumulatorCellIndices.end(),
        [this](const std::size_t idx1, const std::size_t idx2) { return m_accumulator[idx1] < m_accumulator[idx2]; });

    std::size_t index = *maxIndicesIt;
    const int numVote = m_accumulator[index];

    std::size_t xIndex = index / (m_param.numRangeBin * m_sphere.vertices().size());
    float x = minRange + xIndex * deltaRange;

    index -= xIndex * m_param.numRangeBin * m_sphere.vertices().size();
    std::size_t yIndex = index / m_sphere.vertices().size();
    float y = minRange + yIndex * deltaRange;

    index -= yIndex * m_sphere.vertices().size();
    const auto& direction = m_sphere.vertices()[index];

    coeffs.values.resize(6);
    coeffs.values[0] = x * (1 - ((direction[0] * direction[0]) / (1 + direction[2]))) -
                       y * ((direction[0] * direction[1]) / (1 + direction[2]));
    coeffs.values[1] = x * (-((direction[0] * direction[1]) / (1 + direction[2]))) +
                       y * (1 - ((direction[1] * direction[1]) / (1 + direction[2])));
    coeffs.values[2] = -x * direction[0] - y * direction[1];
    coeffs.values[3] = direction[0];
    coeffs.values[4] = direction[1];
    coeffs.values[5] = direction[2];

    return numVote;
}

template <typename POINT_CLOUD_TYPE>
typename HoughTransform<POINT_CLOUD_TYPE>::LineSegment3Ds
HoughTransform<POINT_CLOUD_TYPE>::run(const PointCloudPtr& cloud)
{
    PointCloudType minBound;
    PointCloudType maxBound;

    pcl::getMinMax3D(*cloud, minBound, maxBound);
    float maxNorm = std::max<float>(this->calcNorm(minBound), this->calcNorm(maxBound));
    float range = 2 * maxNorm;
    float minRange = -maxNorm;
    float deltaRange = range / m_param.numRangeBin;
    m_accumulator.resize(m_param.numRangeBin * m_param.numRangeBin * m_sphere.vertices().size(), 0);

    for (const auto& point : cloud->points) {
        this->votePoint(point, minRange, deltaRange, true);
    }

    // only consider cells whose number of votes surpass vote number threshold
    std::vector<std::size_t> accumulatorCellIndices;
    for (std::size_t i = 0; i < m_accumulator.size(); ++i) {
        if (m_accumulator[i] >= m_param.minNumVote) {
            accumulatorCellIndices.emplace_back(i);
        }
    }

    LineSegment3Ds result;
    pcl::ModelCoefficients coeffs;
    std::size_t remainNumPoints = cloud->size();
    std::vector<char> ignorePointIndices(cloud->size(), false);
    while (remainNumPoints >= m_param.minNumVote) {
        std::size_t numVotes = this->getLine(coeffs, minRange, deltaRange, accumulatorCellIndices);
        if (numVotes < m_param.minNumVote) {
            break;
        }
        LineSegment3D lineSegment(cloud, coeffs);
        if (!lineSegment.refine(m_param.distanceToLineThresh, ignorePointIndices)) {
            break;
        }
        std::vector<int> pointIndicesToRemove = LineSegment3D::getPointIndicesCloseToLine(
            cloud, lineSegment.coeffs(), m_param.distanceToLineThresh, ignorePointIndices);
        for (const auto pointIdx : pointIndicesToRemove) {
            const auto& curPoint = cloud->points[pointIdx];
            this->votePoint(curPoint, minRange, deltaRange, false);
            ignorePointIndices[pointIdx] = true;
        }
        result.emplace_back(std::move(lineSegment));
        remainNumPoints -= pointIndicesToRemove.size();
    }

    return result;
}

template <typename POINT_CLOUD_TYPE> float HoughTransform<POINT_CLOUD_TYPE>::calcNorm(const PointCloudType& point) const
{
    return std::sqrt(point.x * point.x + point.y * point.y + point.z * point.z);
}
}  // namespace perception
