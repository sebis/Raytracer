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

#include "MeshImporterOBJ.h"
#include "Constants.h"

#include <Pane/Trace.h>

#include <fstream>
#include <regex>

namespace Raytracer2
{
    bool MeshImporterOBJ::accepts(const std::string & ext) const
    {
        return !_stricmp(ext.c_str(), "obj");
    }

    MeshPtr MeshImporterOBJ::loadFromFile(const std::string & filename, int flags) const
    {
        Pane::info("MeshImporterOBJ # Started parsing file %s", filename.c_str());

        std::ifstream file(filename);

        if(!file.is_open())
        {
            Pane::error("Could not load OBJ file (%s)", filename.c_str());
            return nullptr;
        }

        std::string line;
        unsigned lineNum = 0;

        // TODO: obj file can have multiple objects, should support that somehow
        MeshPtr mesh = std::make_shared<Mesh>();
        mesh->smoothShading = !(flags & ImporterFlags::FlatShading);

        while (file.good())
        {
            std::getline(file, line);
            lineNum++;

            // Match vertex attributes <name> <float>[1,4]
            //std::regex regex("v(t|n|p)?\\s+([-+]?[0-9]*\\.?[0-9]+\\s+){1,4}");
            /*std::regex regex("v(t|n|p)?\\s+?([-+]?[0-9]*\\.?[0-9]+\\s*?){1,4}");
            std::smatch match;

            if(std::regex_match(line, match, regex))
            {
                Pane::info("Match size: %d", match.size());
                for(int i = 0; i < match.size(); i++)
                {
                    Pane::info("Match: %s", match[i].str().c_str());
                }
            }*/

            if(line.empty())
                continue;

            char cmd[11];
            if(!sscanf(line.c_str(), "%s", cmd)) // nothing read
                continue;

            if(strcmp(cmd, "#") == 0) // comment
            {
                continue;
            }
            else if(strcmp(cmd, "v") == 0) // vertex
            {
                double x, y, z, w = 1.0; // w optional and defaults to 1.0
                int ret = sscanf(line.c_str(), "%*s %lf %lf %lf %lf", &x, &y, &z, &w);
                if(ret >= 3)
                {
                    mesh->vertices.emplace_back(x, y, z);
                    if (ret == 4)
                        Pane::warning("%d: Unsupported vertex: %s (only support 3D vectors for vertices)", lineNum, line.c_str());
                }
                else
                    Pane::error("%d: Unsupported vertex: %s", lineNum, line.c_str());
            }
            else if(strcmp(cmd, "vt") == 0) // texture coordinate
            {
                double u, v, w = 0; // all [0, 1], w optional and defaults to 0
                int ret = sscanf(line.c_str(), "%*s %lf %lf %lf", &u, &v, &w);
                if(ret >= 2)
                    mesh->texcoords.emplace_back(u, v);
                else
                    Pane::error("%d: Unsupported texture coordinate: %s", lineNum, line.c_str());
            }
            else if(strcmp(cmd, "vn") == 0) // normals
            {
                double x, y, z; // normals might not be unit length
                int ret = sscanf(line.c_str(), "%*s %lf %lf %lf", &x, &y, &z);
                if(ret == 3)
                    mesh->normals.emplace_back(x, y, z);
                else
                    Pane::error("%d: Unsupported normal: %s", lineNum, line.c_str());
            }
            else if(strcmp(cmd, "vp") == 0) // parameter space vertices
            {
                double u, v = 0, w = 0; // v, w optional
                int ret = sscanf(line.c_str(), "%*s %lf %lf %lf", &u, &v, &w);
                if(ret > 0)
                    Pane::warning("%d: Parameter space vertices not implemented", lineNum);
                else
                    Pane::error("%d: Unsupported parameter space vertex: %s", lineNum, line.c_str());
            }
            else if(strcmp(cmd, "f") == 0) // face
            {
                // Possible formats for faces:
                // f v1 v2 v3 v4 .... (only vertex indices)
                // f v1/vt1 v2/vt2 v3/vt3 ... (vertex/texture-coordinate)
                // f v1/vt1/vn1 v2/vt2/vn2 v3/vt3/vn3 .... (vertex/texture-coordinate/normal)
                // f v1//vn1 v2//vn2 v3//vn3 ... (vertex/normal)

                Face face(4);
                face.mesh = mesh.get();

                std::regex regex("(-?\\d+)(?:/(-?\\d*))?(?:/(-?\\d+))?");
                std::sregex_iterator it(line.begin(), line.end(), regex);
                std::sregex_iterator end;

                // loop over vertices (the size is 3 for triangles, 4 for quadrilaterals etc..)
                for(; it != end; ++it)
                {
                    //Pane::info("Found face vertex: %s", it->str().c_str());

                    // Get all indices (0 indicates missing)

                    int vIndex = atoi((*it)[1].str().c_str());
                    if(vIndex)
                    {
                        if(vIndex < 0) // relative index (nth last vertex)
                            vIndex = mesh->vertices.size() - glm::abs(vIndex);
                        else if(vIndex > 0) // absolute index (note: OBJ indexing starts from 1)
                            vIndex = vIndex - 1;
                        face.vIndices.push_back(vIndex);
                    }

                    int tIndex = atoi((*it)[2].str().c_str());
                    if(tIndex)
                    {
                        if(tIndex < 0)
                            tIndex = mesh->texcoords.size() - glm::abs(tIndex);
                        else if(tIndex > 0)
                            tIndex = tIndex - 1;
                        face.tIndices.push_back(tIndex);
                    }
                    
                    int nIndex = atoi((*it)[3].str().c_str());
                    if(nIndex)
                    {
                        if(nIndex < 0)
                            nIndex = mesh->normals.size() - glm::abs(nIndex);
                        else if (nIndex > 0)
                            nIndex = nIndex - 1;
                        face.nIndices.push_back(nIndex);
                    }
                }

                // OBJ never has face normals (only optional vertex normals)
                // TODO: do we always want to compute face normals?
                face.normal = face.computeNormal();

                mesh->faces.push_back(std::move(face));
            }
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

        mesh->nFaces = mesh->faces.size();
        mesh->nVertices = mesh->vertices.size();
        mesh->nNormals = mesh->normals.size();
        mesh->nTexcoords = mesh->texcoords.size();

        Pane::info("MeshImporterOBJ # Finishing loading file %s", filename.c_str());
        Pane::info("MeshImporterOBJ # Faces: %d, vertices: %d, normals: %d, texcoords: %d",
            mesh->nFaces, mesh->nVertices, mesh->nNormals, mesh->nTexcoords);

        return mesh;
    }

    MeshImporterRegistrar<MeshImporterOBJ> RegisteredMeshImporterOBJ;
}