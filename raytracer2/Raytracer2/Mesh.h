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

#ifndef MESH_H
#define MESH_H

#include "Constants.h"
#include "Intersection.h"

#include <glm/glm.hpp>

#include <memory>
#include <vector>

namespace Raytracer2
{
    // Face that can represent triangles and quadrilaterals (or any other for that matter)
    // TODO: should combine faces and geometric surfaces to Primitive object
    struct Mesh;
    struct Face
    {
        Face()
        {}

        Face(size_t capacity)
        {
            vIndices.reserve(capacity);
            tIndices.reserve(capacity);
            nIndices.reserve(capacity);
        }

        size_t size() const
        {
            //assert(vIndices.size() == tIndices.size() && tIndices.size() == nIndices.size());
            return vIndices.size();
        }

        bool hasVertexNormals() const
        {
            return nIndices.size() != 0;
        }

        // Computes face normal. Assumes that the face is planar.
        glm::vec3 computeNormal() const;

        bool intersect(const glm::ray & ray, real maxd, Intersection & isec) const;

        std::vector<unsigned> vIndices; // indices for vertices
        std::vector<unsigned> tIndices; // indices for texcoords
        std::vector<unsigned> nIndices; // indices for normals

        // face normal
        glm::vec3 normal;

        // The mesh that owns this face
        Mesh *mesh;
    };

    struct Mesh
    {
        Mesh()
            : nVertices(0), nTexcoords(0), nNormals(0), nFaces(0)
            , smoothShading(true)
        {
        }

        size_t nVertices;
        std::vector<glm::vec3> vertices;

        size_t nTexcoords;
        std::vector<glm::vec2> texcoords;

        size_t nNormals;
        std::vector<glm::vec3> normals;

        size_t nFaces;
        std::vector<Face> faces;

        bool smoothShading;
    };
    typedef std::shared_ptr<Mesh> MeshPtr;
}

#endif // MESH_H