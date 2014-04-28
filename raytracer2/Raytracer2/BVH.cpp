/* Copyright (c) 2014, Sebastian Eriksson
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *    * Redistributions of source code must retain the above copyright
 *      notice, this list of conditions and the following disclaimer.
 *    * Redistributions in binary form must reproduce the above copyright
 *      notice, this list of conditions and the following disclaimer in the
 *      documentation and/or other materials provided with the distribution.
 *    * Neither the name of Sebastian Eriksson nor the
 *      names of its contributors may be used to endorse or promote products
 *      derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL SEBASTIAN ERIKSSON BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "BVH.h"

#include <Pane/Trace.h>

#include <stack>

namespace Raytracer2
{
    namespace
    {
        glm::aabb computeAABB(const MeshPtr & mesh, unsigned faceId)
        {
            glm::aabb bounds;
            const Face & face = mesh->faces[faceId];
            for(unsigned i = 0; i < face.size(); ++i)
                bounds.expand(mesh->vertices[face.vIndices[i]]);
            return bounds;
        }
    }

    void BVH::build()
    {
        Pane::info("BVH # Started building from mesh");

        std::vector<BVHPrimitive> data;
        data.reserve(mesh->faces.size());
        // TODO: reserve for primitives

        // Create data structures for BVH
        for(unsigned i = 0; i < mesh->faces.size(); i++)
            data.emplace_back(i, computeAABB(mesh, i));

        Pane::info("BVH # Finished creating bounding volumes");

        unsigned totalNodes = 0;

        // Build recursively
        BVHNode *root = buildRecursive(data, 0, mesh->faces.size(), primitives, totalNodes);

        Pane::info("BVH # Finished volume hierarchy build");

        // Allocate linear nodes
        nodes.resize(totalNodes);

        unsigned offset = 0;
        flatten(root, offset);

        Pane::info("BVH # Finished flattening volume hierarchy");
    }

    unsigned BVH::flatten(BVHNode *node, unsigned & offset)
    {
        // Create a new linear node at the given offset
        LinearBVHNode & linearNode = nodes[offset];
        linearNode.bounds = node->bounds;

        // advances offset by one and takes copy of original value
        const unsigned myOffset = offset++;

        // Check if leaf or interior node
        if(node->nPrimitives > 0)
        {
            // This is a leaf so just copy node data
            linearNode.primitivesOffset = node->offset;
            linearNode.nPrimitives = node->nPrimitives;
        }
        else
        {
            // Call recursively to flatten left and right subtrees
            linearNode.axis = node->split;
            linearNode.nPrimitives = 0;

            // Flatten left subtree
            flatten(node->children[0], offset);

            // Flatten right subtree
            linearNode.secondChildOffset = flatten(node->children[1], offset);
        }

        return myOffset;
    }

    BVHNode* BVH::buildRecursive(std::vector<BVHPrimitive> & data, unsigned start, unsigned end,
        std::vector<Face*> & orderedPrimitives, unsigned & totalNodes)
    {
        // Primitives in this branch
        const unsigned nPrimitives = end - start;

        // Allocate node (TODO: get from pool)
        BVHNode *node = new BVHNode;
        totalNodes++;

        // Calculate bounding box for all primitives given
        glm::aabb bounds;
        for(unsigned i = start; i < end; ++i)
            bounds = glm::aabb::combine(bounds, data[i].bounds);

        if(nPrimitives <= m_primitivesPerNode) // only n primitive left, create leaf node
        {
            const unsigned firstPrimitive = (unsigned)orderedPrimitives.size();
            for(unsigned i = start; i < end; ++i)
            {
                const unsigned primitive = data[i].primitive;
                orderedPrimitives.push_back(&mesh->faces[primitive]);
            }
            node->createLeaf(firstPrimitive, nPrimitives, bounds);
        }
        else
        {
            // Choose split dimension based on aabb centroids
            glm::aabb centroidBounds;
            for(unsigned i = start; i < end; ++i)
                centroidBounds.expand(data[i].centroid);
            if(centroidBounds.empty())
            {
                // All centroids the same, just create a leaf node

                const unsigned firstPrimitive = (unsigned)orderedPrimitives.size();
                for(unsigned i = start; i < end; ++i)
                {
                    const unsigned primitive = data[i].primitive;
                    orderedPrimitives.push_back(&mesh->faces[primitive]);
                }
                node->createLeaf(firstPrimitive, nPrimitives, bounds);

                return node;
            }

            const int dim = centroidBounds.largest_axis();                

            // Partition primitives to two subtrees
            const unsigned mid = (start + end) / 2;
            std::nth_element(&data[start], &data[mid], &data[end-1]+1,
                [=] (const BVHPrimitive & a, const BVHPrimitive & b)
                {
                    return a.centroid[dim] < b.centroid[dim];
                });

            node->createInterior(dim,
                buildRecursive(data, start, mid, orderedPrimitives, totalNodes),
                buildRecursive(data, mid, end, orderedPrimitives, totalNodes));
        }

        return node;
    }

    bool BVH::intersect(const glm::ray & ray, Intersection & isec) const
    {
        if(nodes.empty())
            return false;

        bool hit = false;
        real tmax = max_value;

        const glm::vec3 & p = ray.origin;
        const glm::vec3 invd(1/ray.dir.x, 1/ray.dir.y, 1/ray.dir.z);
        const glm::bvec3 dirIsNeg(invd.x < 0, invd.y < 0, invd.z < 0);

        std::stack<unsigned> todo;
        unsigned nodeNum = 0;

        while (true)
        {
            const LinearBVHNode & node = nodes[nodeNum];

            real tmin;
            if (glm::intersectRayAABB(p, invd, node.bounds, tmin, tmax))
            {
                if(node.nPrimitives > 0) // hit a leaf node
                {
                    // intersect ray with primitives in node
                    for(int i = 0; i < node.nPrimitives; ++i)
                    {
                        Face const* face = primitives[node.primitivesOffset+i];
                        if(face->intersect(ray, tmax, isec))
                        {
                            hit = true;
                            tmax = tmin; // update closest intersection
                        }
                    }

                    if(todo.empty())
                        break;
                    nodeNum = todo.top();
                    todo.pop();
                }
                else
                {
                    // put far BVH node on todo stack, advance to near node
                    if(dirIsNeg[node.axis])
                    {
                        // push left child and visit right child
                        todo.push(nodeNum+1);
                        nodeNum = node.secondChildOffset;
                    }
                    else
                    {
                        // push right child and visit left child
                        todo.push(node.secondChildOffset);
                        nodeNum = nodeNum + 1;
                    }
                }
            }
            else
            {
                if(todo.empty())
                    break;

                nodeNum = todo.top();
                todo.pop();
            }
        }

        return hit;
    }
}
