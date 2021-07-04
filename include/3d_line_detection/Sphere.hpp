/**
 * @file    Sphere.hpp
 *
 * @author  btran
 *
 */

#pragma once

#include <array>
#include <cmath>
#include <deque>
#include <vector>

namespace geometry
{
class Sphere
{
 public:
    using Vertex = std::array<float, 3>;
    using Triangle = std::array<int, 3>;

    static constexpr int MAX_SUBDIVISION = 6;

    static const std::vector<Sphere> SPHERE_LOOK_UP_TABLE;

    const std::vector<Vertex>& vertices() const
    {
        return m_vertices;
    }

    const std::deque<Triangle>& triangles() const
    {
        return m_triangles;
    }

 private:
    Sphere() = delete;

    Sphere(const std::vector<Vertex>& vertices, const std::deque<Triangle>& triangles);

    static std::vector<Sphere> createSphereLookUpTable();

    /**
     *  \brief create an icosahedron with 12 vertices
     */
    static Sphere getIcosahedron();

    /**
     *  \brief subdivide each triangle surface to make the sphere finer
     */
    void subdivide();

    /**
     *  \brief create a new vertex from two adjacent vertices in the subdivision step
     */
    Vertex createFrom(const Vertex& v1, const Vertex& v2) const;

    /**
     *  \brief delete triangle surfaces that contain a specific vertex
     */
    void deleteTriangleHasVertex(const int vertexIdx);

    /**
     *  \brief remove vertices to create unique direction vectors
     */
    void makeUnique();

 private:
    std::vector<Vertex> m_vertices;
    std::deque<Triangle> m_triangles;
};

}  // namespace geometry
