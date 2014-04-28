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

#include "MeshImporterPLY.h"

#include <Pane/Trace.h>

#include <cmath>
#include <fstream>
#include <regex>

namespace Raytracer2
{
    bool MeshImporterPLY::accepts(const std::string & ext) const
    {
        return !_stricmp(ext.c_str(), "ply");
    }

    MeshPtr MeshImporterPLY::loadFromFile(const std::string & filename, int flags) const
    {
        std::ifstream file(filename);

        if(!file.is_open())
        {
            Pane::error("Could not load PLY file (%s)", filename.c_str());
            return nullptr;
        }

        std::string line;

        MeshPtr mesh = std::make_shared<Mesh>();

        // Read header
        while (file.good())
        {
            std::getline(file, line);
            if (line == "end_header")
                break;

            static std::regex emVertex("element vertex (\\d+)");
            static std::regex emFace("element face (\\d+)");

            std::smatch sm;
            if (regex_search(line, sm, emVertex))
                mesh->nVertices = atoi(sm.str(1).c_str());
            else if(regex_search(line, sm, emFace))
                mesh->nFaces = atoi(sm.str(1).c_str());
        }

        // Use one normal for each vertex
        mesh->nNormals = mesh->nVertices;

        mesh->vertices.reserve(mesh->nVertices);
        mesh->faces.reserve(mesh->nFaces);
        mesh->normals.reserve(mesh->nNormals);

        // Read vertex data
        for(int i = 0; file.good() && i < mesh->nVertices; i++)
        {
            std::getline(file, line);

            glm::vec3 v;
            sscanf(line.c_str(), "%lf %lf %lf", &v.x, &v.y, &v.z);

            mesh->vertices.emplace_back(v);
            mesh->normals.emplace_back(glm::vec3(0));
        }

        // Read index data
        for(int i = 0; file.good() && i < mesh->nFaces; i++)
        {
            std::getline(file, line);

            glm::uvec3 u;
            sscanf(line.c_str(), "%*s %d %d %d", &u.x, &u.y, &u.z);

            // calculate normal
            const glm::vec3 & p = mesh->vertices[u.z] - mesh->vertices[u.x];
            const glm::vec3 & q = mesh->vertices[u.y] - mesh->vertices[u.x];
            const glm::vec3 & n = glm::normalize(glm::cross(p, q));

            // accumulate normals from all connecting faces
            mesh->normals[u.x] += n;
            mesh->normals[u.y] += n;
            mesh->normals[u.z] += n;

            // create face
            Face face(3);
            face.mesh = mesh.get();

            // indices to vertex array
            face.vIndices.push_back(u.x);
            face.vIndices.push_back(u.y);
            face.vIndices.push_back(u.z);

            // indices to normal array (we have one normal for each vertex)
            face.tIndices.push_back(u.x);
            face.tIndices.push_back(u.y);
            face.tIndices.push_back(u.z);
            
            // TODO: do we always want to compute face normals?
            face.normal = face.computeNormal();

            mesh->faces.push_back(std::move(face));
        }

        if (flags & ImporterFlags::CalculateVertexNormals)
        {
            // Because normals aren't explicitly given, we have to calculate a normal for each vertex
            // for each face. Each adjacent face might contribute to the vertex normal, so add all
            // contributing normals and renormalize afterwards

            mesh->normals.clear();
            mesh->normals.resize(mesh->vertices.size(), glm::vec3());

            for(auto & face : mesh->faces)
            {
                const glm::vec3 & n = face.normal;
                face.nIndices.resize(face.size());

                for(int i = 0; i < face.size(); i++)
                {
                    const unsigned idx = face.vIndices[i];
                    mesh->normals[idx] += n;

                    face.nIndices[i] = idx;
                }
            }

            // Renormalize normals
            for(auto & n : mesh->normals)
                n = glm::normalize(n);
        }

        return mesh;
    }

    MeshImporterRegistrar<MeshImporterPLY> RegisteredMeshImporterPLY;
    //REGISTER_IMPORTER(MeshImporterPLY)
}
