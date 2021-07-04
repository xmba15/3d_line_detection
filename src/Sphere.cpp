/**
 * @file    Sphere.cpp
 *
 * @author  btran
 *
 */

#include <algorithm>
#include <cmath>
#include <unordered_map>

#include <3d_line_detection/Sphere.hpp>

namespace
{
const float GOLDEN_RATIO = (1 + std::sqrt(5)) / 2.;
}  // namespace

namespace geometry
{
std::vector<Sphere> Sphere::createSphereLookUpTable()
{
    std::vector<Sphere> table;
    table.reserve(Sphere::MAX_SUBDIVISION);
    Sphere sphere = Sphere::getIcosahedron();
    table.emplace_back(sphere);
    table.back().makeUnique();

    for (int i = 1; i < Sphere::MAX_SUBDIVISION; ++i) {
        sphere.subdivide();
        table.emplace_back(sphere);
        table.back().makeUnique();
    }
    return table;
}

const std::vector<Sphere> Sphere::SPHERE_LOOK_UP_TABLE = Sphere::createSphereLookUpTable();

Sphere::Sphere(const std::vector<Vertex>& vertices, const std::deque<Triangle>& triangles)
    : m_vertices(vertices)
    , m_triangles(triangles)
{
}

Sphere Sphere::getIcosahedron()
{
    const float norm = std::sqrt(1 + ::GOLDEN_RATIO * ::GOLDEN_RATIO);
    const float v = 1 / norm;
    const float tau = ::GOLDEN_RATIO / norm;

    std::vector<Vertex> vertices = {{-v, tau, 0},  {v, tau, 0},  {0, v, -tau},  {0, v, tau},
                                    {-tau, 0, -v}, {tau, 0, -v}, {-tau, 0, v},  {tau, 0, v},
                                    {0, -v, -tau}, {0, -v, tau}, {-v, -tau, 0}, {v, -tau, 0}};

    std::deque<Triangle> triangles = {{0, 1, 2},  {0, 1, 3},  {0, 2, 4},   {0, 4, 6},   {0, 3, 6},
                                      {1, 2, 5},  {1, 3, 7},  {1, 5, 7},   {2, 4, 8},   {2, 5, 8},
                                      {3, 6, 9},  {3, 7, 9},  {4, 8, 10},  {8, 10, 11}, {5, 8, 11},
                                      {5, 7, 11}, {7, 9, 11}, {9, 10, 11}, {6, 9, 10},  {4, 6, 10}};

    return Sphere(vertices, triangles);
}

void Sphere::subdivide()
{
    const int curNumTriangle = m_triangles.size();
    std::unordered_map<std::string, int> newVerticesMap;

    for (int idx = 0; idx < curNumTriangle; ++idx) {
        const int curNumVertices = m_vertices.size();
        Triangle curTriangle = m_triangles.front();
        m_triangles.pop_front();

        Triangle newTriangle;

        int countNewVertices = 0;
        for (int i = 0; i < 3; ++i) {
            for (int j = i + 1; j < 3; ++j) {
                std::string newVertexLabel = std::to_string(curTriangle[i]) + "_" + std::to_string(curTriangle[j]);
                int newVertexIdx;

                auto it = newVerticesMap.find(newVertexLabel);
                if (it == newVerticesMap.end()) {
                    Vertex newVertex = this->createFrom(m_vertices[curTriangle[i]], m_vertices[curTriangle[j]]);
                    m_vertices.emplace_back(std::move(newVertex));
                    newVertexIdx = curNumVertices + countNewVertices++;
                    newVerticesMap.insert(std::make_pair(newVertexLabel, newVertexIdx));
                } else {
                    newVertexIdx = it->second;
                }
                newTriangle[i + j - 1] = newVertexIdx;
            }
        }

        m_triangles.emplace_back(newTriangle);
        std::sort(m_triangles.back().begin(), m_triangles.back().end());

        for (int i = 0; i < 3; ++i) {
            for (int j = i + 1; j < 3; ++j) {
                m_triangles.push_back({curTriangle[i + j - 1], newTriangle[i], newTriangle[j]});
                std::sort(m_triangles.back().begin(), m_triangles.back().end());
            }
        }
    }
}

Sphere::Vertex Sphere::createFrom(const Vertex& v1, const Vertex& v2) const
{
    Vertex v;
    float norm = 0.0;
    for (int i = 0; i < 3; ++i) {
        v[i] = v1[i] + v2[i];
        norm += v[i] * v[i];
    }
    norm = std::sqrt(norm);
    for (int i = 0; i < 3; ++i) {
        v[i] /= norm;
    }

    return v;
}

void Sphere::deleteTriangleHasVertex(const int vertexIdx)
{
    auto it = m_triangles.begin();
    while (it != m_triangles.end()) {
        if (std::find(it->begin(), it->end(), vertexIdx) != it->end()) {
            it = m_triangles.erase(it);
        } else {
            ++it;
        }
    }
}

void Sphere::makeUnique()
{
    std::vector<int> verticesToRemove;

    auto needToRemove = [](const Vertex& v) {
        return (v[2] < 0) || (v[2] == 0 && ((v[0] < 0) || (v[0] == 0 && v[1] == -1)));
    };

    for (std::size_t i = 0; i < m_vertices.size(); ++i) {
        if (needToRemove(m_vertices[i])) {
            verticesToRemove.emplace_back(i);
            this->deleteTriangleHasVertex(i);
        }
    }

    int count = 0;
    for (const int vertexToRemove : verticesToRemove) {
        m_vertices.erase(m_vertices.begin() + vertexToRemove - count);
        for (auto& curTriangle : m_triangles) {
            for (auto& vertexIdx : curTriangle) {
                if (vertexIdx > (vertexToRemove - count))
                    vertexIdx--;
            }
        }
        count++;
    }
}
}  // namespace geometry
