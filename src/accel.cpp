/*
    This file is part of Nori, a simple educational ray tracer

    Copyright (c) 2015 by Wenzel Jakob

    Nori is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License Version 3
    as published by the Free Software Foundation.

    Nori is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program. If not, see <http://www.gnu.org/licenses/>.
*/

#include <nori/accel.h>
#include <Eigen/Geometry>
#include <unordered_map>
#include <chrono>
NORI_NAMESPACE_BEGIN

void Accel::addMesh(Mesh *mesh)
{
    m_meshes.push_back(mesh);
    m_bbox.expandBy(mesh->getBoundingBox());
}

void Accel::build()
{
    /* Nothing to do here for now */

    auto start = std::chrono::high_resolution_clock::now();
    std::vector<std::pair<int, int>> triangles;
    for(int i = 0; i < m_meshes.size(); ++i)
    {
        for (int j = 0; j < m_meshes[i]->getTriangleCount(); ++j)
        {
            triangles.push_back(std::pair(i, j));
        }
    }

    OctreeNode root;
    root.bbox = m_bbox;
    m_octree.nodes.push_back(root);
    buildOctree(0, triangles, 0);
    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = end - start;
    std::cout << "Octree build time: " << elapsed.count() << "s " << std::endl;
    // find leaf node size
    int leafNodeCount = 0;
    for(auto node : m_octree.nodes)
    {
        if (node.triangles.size() > 0)
        {
            leafNodeCount++;
        }
    }
    std::cout << "Octree node count: " << m_octree.nodes.size() << std::endl;
    std::cout << "Leaf node count: " << leafNodeCount << std::endl;

}

void Accel::buildOctree(int nodeIndex, std::vector<std::pair<int, int>>& triangles, int depth)
{
    if (triangles.size() <= 10 || depth >= max_depth)
    {
        m_octree.nodes[nodeIndex].triangles = triangles;
        return;
    }
    BoundingBox3f bbox = m_octree.nodes[nodeIndex].bbox;
    Vector3f center = bbox.getCenter();
    std::vector<std::pair<int, int>> triangle_list[8];
    BoundingBox3f subBoxes[8];
    for (auto triangleIndex : triangles)
    {
        for (int i = 0; i < 8; ++i)
        {
            Vector3f corner = bbox.getCorner(i);
            Vector3f boxMin;
            Vector3f boxMax;
            for(int j = 0; j < 3; ++j)
            {
                boxMin[j] = std::min(corner[j], center[j]);
                boxMax[j] = std::max(corner[j], center[j]);
            }
            BoundingBox3f subBox(boxMin, boxMax);
            auto triangleBox = m_meshes[triangleIndex.first]->getBoundingBox(triangleIndex.second);
            if (subBox.overlaps(triangleBox))
            {
                triangle_list[i].push_back(triangleIndex);
                subBoxes[i] = subBox;
            }
        }
    }

    for (int i = 0; i < 8; ++i)
    {
        if (triangle_list[i].size() > 0)
        {
            OctreeNode octreeNode;
            octreeNode.bbox = subBoxes[i];
            m_octree.nodes[nodeIndex].children.push_back(m_octree.nodes.size());
            m_octree.nodes.push_back(octreeNode);
            buildOctree(m_octree.nodes.size() - 1, triangle_list[i], depth + 1);
        }
    }
}


bool Accel::rayIntersectOctree(const Ray3f &ray_, Intersection &its, bool shadowRay, uint32_t *faceIndex) const
{
    Ray3f ray(ray_);
    std::vector<int> nodesToVisit;
    nodesToVisit.push_back(0);
    bool foundIntersection = false;
    while (!nodesToVisit.empty())
    {
        int node = nodesToVisit.back();
        nodesToVisit.pop_back();
        if (m_octree.nodes[node].bbox.rayIntersect(ray))
        {
            if (m_octree.nodes[node].children.empty())
            {
                for (auto triangleIndex : m_octree.nodes[node].triangles)
                {
                    float u, v, t;
                    if (m_meshes[triangleIndex.first]->rayIntersect(triangleIndex.second, ray, u, v, t))
                    {
                        /* An intersection was found! Can terminate
                           immediately if this is a shadow ray query */
                        if (shadowRay)
                            return true;
                        ray.maxt = its.t = t;
                        its.uv = Point2f(u, v);
                        its.mesh = m_meshes[triangleIndex.first];
                        *faceIndex = triangleIndex.second;
                        foundIntersection = true;
                    }
                }
            }
            else
            {

                for (auto child : m_octree.nodes[node].children)
                {

                    nodesToVisit.push_back(child);
                }
            }
        }
    }
    return foundIntersection;
}

// bool Accel::rayIntersectOctreeNode(const Ray3f &ray_, Intersection &its, bool shadowRay, int index) const
// {
// }

bool Accel::rayIntersect(const Ray3f &ray_, Intersection &its, bool shadowRay) const
{
    bool foundIntersection = false; // Was an intersection found so far?
    uint32_t meshIndex = (uint32_t)-1;
    uint32_t f = (uint32_t)-1;      // Triangle index of the closest intersection

    // Ray3f ray(ray_); /// Make a copy of the ray (we will need to update its '.maxt' value)

    // /* Brute force search through all triangles */
    // for (uint32_t idx = 0; idx < m_mesh->getTriangleCount(); ++idx)
    // {
    //     float u, v, t;
    //     if (m_mesh->rayIntersect(idx, ray, u, v, t))
    //     {
    //         /* An intersection was found! Can terminate
    //            immediately if this is a shadow ray query */
    //         if (shadowRay)
    //             return true;
    //         ray.maxt = its.t = t;
    //         its.uv = Point2f(u, v);
    //         its.mesh = m_mesh;
    //         f = idx;
    //         foundIntersection = true;
    //     }
    // }
    // sort the node first 
    foundIntersection = rayIntersectOctree(ray_, its, shadowRay, &f);
    if (shadowRay) {
        return foundIntersection;
    }

    if (foundIntersection)
    {
        /* At this point, we now know that there is an intersection,
           and we know the triangle index of the closest such intersection.

           The following computes a number of additional properties which
           characterize the intersection (normals, texture coordinates, etc..)
        */

        /* Find the barycentric coordinates */
        Vector3f bary;
        bary << 1 - its.uv.sum(), its.uv;

        /* References to all relevant mesh buffers */
        const Mesh *mesh = its.mesh;
        const MatrixXf &V = mesh->getVertexPositions();
        const MatrixXf &N = mesh->getVertexNormals();
        const MatrixXf &UV = mesh->getVertexTexCoords();
        const MatrixXu &F = mesh->getIndices();

        /* Vertex indices of the triangle */
        uint32_t idx0 = F(0, f), idx1 = F(1, f), idx2 = F(2, f);

        Point3f p0 = V.col(idx0), p1 = V.col(idx1), p2 = V.col(idx2);

        /* Compute the intersection positon accurately
           using barycentric coordinates */
        its.p = bary.x() * p0 + bary.y() * p1 + bary.z() * p2;

        /* Compute proper texture coordinates if provided by the mesh */
        if (UV.size() > 0)
            its.uv = bary.x() * UV.col(idx0) +
                     bary.y() * UV.col(idx1) +
                     bary.z() * UV.col(idx2);

        /* Compute the geometry frame */
        its.geoFrame = Frame((p1 - p0).cross(p2 - p0).normalized());

        if (N.size() > 0)
        {
            /* Compute the shading frame. Note that for simplicity,
               the current implementation doesn't attempt to provide
               tangents that are continuous across the surface. That
               means that this code will need to be modified to be able
               use anisotropic BRDFs, which need tangent continuity */

            its.shFrame = Frame(
                (bary.x() * N.col(idx0) +
                 bary.y() * N.col(idx1) +
                 bary.z() * N.col(idx2))
                    .normalized());
        }
        else
        {
            its.shFrame = its.geoFrame;
        }
    }

    return foundIntersection;
}

NORI_NAMESPACE_END
