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

#ifndef BVH_H
#define BVH_H

#include "Constants.h"
#include "GLMExtensions.h"
#include "MeshImporter.h"
#include "SurfacePoint.h"

#include <algorithm>
#include <vector>

namespace Raytracer2
{

    // Stores pointer to actual primitive and info calculated from the primitive
    struct BVHPrimitive
    {
    public:
        BVHPrimitive(int _primitive, const glm::aabb & _bounds)
            : primitive(_primitive)
            , bounds(_bounds)
        {
            centroid = bounds.center();
        }

        int primitive; // face id in mesh->faces[]
        glm::aabb bounds;
        glm::vec3 centroid;
    };

    // BVHNode can denote a leaf or interior node.
    struct BVHNode
    {
        BVHNode()
        {
            children[0] = children[1] = nullptr;
        }

        void createLeaf(unsigned first, unsigned n, const glm::aabb & b)
        {
            offset = first;
            nPrimitives = n;
            bounds = b;
        }

        void createInterior(unsigned axis, BVHNode *left, BVHNode *right)
        {
            split = axis;
            children[0] = left;
            children[1] = right;
            bounds = glm::aabb::combine(left->bounds, right->bounds);
            nPrimitives = 0;
        }

        inline bool isLeaf() const
        {
            return children[0] == nullptr || children[1] == nullptr;
        }

        inline bool isInterior() const
        {
            return !isLeaf();
        }

        BVHNode *children[2];

        glm::aabb bounds; // contains all children
        unsigned split, offset, nPrimitives;
    };

    struct LinearBVHNode
    {
        glm::aabb bounds;
        union {
            uint32_t primitivesOffset; // for leaf
            uint32_t secondChildOffset;
        };

        uint8_t nPrimitives; // 0 -> interior node
        uint8_t axis;        // interior node: xyz
        uint8_t pad[2];      // pad to 32 byte
    };

    class BVH
    {
    public:
        BVH(MeshPtr _mesh, unsigned primitivesPerNode = 128)
            : mesh(_mesh)
            , m_primitivesPerNode(primitivesPerNode)
        {
        }

        // builds the BVH from the primitives
        void build();
        unsigned flatten(BVHNode *node, unsigned & offset);

        BVHNode* buildRecursive(std::vector<BVHPrimitive> & data, unsigned start, unsigned end,
            std::vector<Face*> & orderedPrimitives, unsigned & totalNodes);

        bool intersect(const glm::ray & ray, Intersection & isec) const;

    private:
        MeshPtr mesh;
        const unsigned m_primitivesPerNode;

        // Reordered vector of faces (i.e. those from the given mesh). In this representation all
        // leaf nodes are continously laid in memory
        std::vector<Face*> primitives;

        std::vector<LinearBVHNode> nodes;
    };
}

#endif // BVH_H