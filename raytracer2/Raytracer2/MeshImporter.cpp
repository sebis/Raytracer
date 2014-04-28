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

#include "MeshImporter.h"

#include <Pane/Trace.h>

#include <algorithm>

#include "Constants.h"

namespace
{
    std::string ext(const std::string & filename)
    {
        size_t index = filename.find_last_of('.');
        if(index != std::string::npos)
            return filename.substr(index+1);
        return std::string();
    }

    using namespace Raytracer2;

    inline real scalarTriple(const glm::vec3 & u, const glm::vec3 & v, const glm::vec3 & w)
    {
        return glm::dot(glm::cross(u, v), w);
    }

    template <typename valueType>
    bool same_sign(typename valueType x, typename valueType y)
    {
        return (x >= 0) ^ (y < 0);
    }

    // Intersects ray r against plane (n, d)
    inline bool intersect_ray_plane(const glm::ray & r,
        const glm::vec3 & n, const real d, real & t)
    {
        t = (d - glm::dot(n, r.origin)) / glm::dot(n, r.dir);
        return t >= real(0);
    }
    // Intersects ray r against plane defined by triangle abc
    inline bool intersect_ray_plane(const glm::ray & r,
        const glm::vec3 & a, const glm::vec3 & b, const glm::vec3 & c,
        const glm::vec3 & n, real & t)
    {
        const real d = glm::dot(n, a);
        return intersect_ray_plane(r, n, d, t);
    }
    // Intersects ray r against plane defined by triangle abc
    inline bool intersect_ray_plane(const glm::ray & r,
        const glm::vec3 & a, const glm::vec3 & b, const glm::vec3 & c, real & t)
    {
        const glm::vec3 & n = glm::cross(b - a, c - a);
        const real d = glm::dot(n, a);

        return intersect_ray_plane(r, n, d, t);
    }

    // From Real-Time Collision Detection, pp. 191
    bool intersect_ray_triangle(const glm::vec3 & p, const glm::vec3 & q,
        const glm::vec3 & a, const glm::vec3 & b, const glm::vec3 & c, const glm::vec3 & normal,
        real & u, real & v, real & w, real & t)
    {
        const glm::vec3 & ab = b - a;
        const glm::vec3 & ac = c - a;
        const glm::vec3 & qp = p - q;

        // TODO: why does precomputed normal give wrong results?
        const glm::vec3 & n = glm::cross(ab, ac);

        const real d = glm::dot(qp, n);
        if (d <= real(0))
            return false;

        const glm::vec3 & ap = p - a;
        t = glm::dot(ap, n);
        if(t < real(0))
            return false;

        const glm::vec3 & e = glm::cross(qp, ap);
        v = glm::dot(ac, e);
        if (v < real(0) || v > d)
            return false;
        w = -glm::dot(ab, e);
        if(w < real(0) || v + w > d)
            return false;

        float ood = real(1) / d;
        t *= ood;
        v *= ood;
        w *= ood;
        u = real(1) - v - w;
        return true;
    }

    // From Real-Time Collision Detection, pp. 186
    bool intersect_ray_triangle(const glm::ray & r,
        const glm::vec3 & a, const glm::vec3 & b, const glm::vec3 & c,
        real & u, real & v, real & w, real & t)
    {
        const glm::vec3 & pq = r.dir;
        const glm::vec3 & pa = a - r.origin;
        const glm::vec3 & pb = b - r.origin;
        const glm::vec3 & pc = c - r.origin;

        const glm::vec3 & m = glm::cross(pq, pc);
        u = glm::dot(pb, m);
        if(u < real(0))
            return false;
        v = -glm::dot(pa, m);
        if(v < real(0))
            return false;
        w = scalarTriple(pq, pb, pa);
        if(w < real(0))
            return false;

        // Calculate t
        if(!intersect_ray_plane(r, a, b, c, t))
            return false;
        if(t < 0)
            return false;

        real denom = real(1) / (u + v + w);
        u *= denom;
        v *= denom;
        w *= denom;
        return true;
    }
    bool intersect_ray_triangle(const glm::ray & r,
        const glm::vec3 & a, const glm::vec3 & b, const glm::vec3 & c,
        real & t)
    {
        real u, v, w;
        return intersect_ray_triangle(r, a, b, c, u, v, w, t);
    }
    bool intersect_ray_triangle(const glm::ray & r,
        const glm::vec3 & a, const glm::vec3 & b, const glm::vec3 & c,
        const glm::vec3 & na, const glm::vec3 & nb, const glm::vec3 & nc,
        glm::vec3 & n, real & t)
    {
        real u, v, w;
        if(!intersect_ray_triangle(r, a, b, c, u, v, w, t))
            return false;

        n = u*na + v*nb + w*nc;

        return true;
    }

    // From Real-Time Collision Detection pp. 189
    inline bool intersect_ray_quad(const glm::ray & r,
        const glm::vec3 & a, const glm::vec3 & b, const glm::vec3 & c, const glm::vec3 & d,
        real & t)
    {
        const glm::vec3 & pq = r.dir;
        const glm::vec3 & pa = a - r.origin;
        const glm::vec3 & pb = b - r.origin;
        const glm::vec3 & pc = c - r.origin;

        const glm::vec3 & m = glm::cross(pc, pq);
        real v = glm::dot(pa, m);
        if(v >= real(0))
        {
            // Test intersection against triangle abc
            real u = -glm::dot(pb, m);
            if (u < real(0))
                return false;
            real w = scalarTriple(pq, pb, pa);
            if(w < real(0))
                return false;

            // Compute t
            if(!intersect_ray_plane(r, a, b, c, t))
                return false;
            if(t < 0)
                return false;
        }
        else
        {
            // Test intersection against triangle dac
            const glm::vec3 & pd = d - r.origin;
            real u = glm::dot(pd, m);
            if (u < real(0))
                return false;
            real w = scalarTriple(pq, pa, pd);
            if(w < real(0))
                return false;
            v = -v;

            // Compute t
            if(!intersect_ray_plane(r, d, a, c, t))
                return false;
            if(t < 0)
                return false;
        }

        return true;
    }
    // From Real-Time Collision Detection pp. 189
    // Same as above, but computes interpolated normal
    inline bool intersect_ray_quad(const glm::ray & r,
        const glm::vec3 & a, const glm::vec3 & b, const glm::vec3 & c, const glm::vec3 & d,
        const glm::vec3 & na, const glm::vec3 & nb, const glm::vec3 & nc, const glm::vec3 & nd,
        glm::vec3 & n, real & t)
    {
        const glm::vec3 & pq = r.dir;
        const glm::vec3 & pa = a - r.origin;
        const glm::vec3 & pb = b - r.origin;
        const glm::vec3 & pc = c - r.origin;

        const glm::vec3 & m = glm::cross(pc, pq);
        real v = glm::dot(pa, m);
        if(v >= real(0))
        {
            // Test intersection against triangle abc
            real u = -glm::dot(pb, m);
            if (u < real(0))
                return false;
            real w = scalarTriple(pq, pb, pa);
            if(w < real(0))
                return false;

            // Compute t
            if(!intersect_ray_plane(r, a, b, c, t))
                return false;
            if(t < 0)
                return false;

            n = u*na + v*nb + w*nc;
        }
        else
        {
            // Test intersection against triangle dac
            const glm::vec3 & pd = d - r.origin;
            real u = glm::dot(pd, m);
            if (u < real(0))
                return false;
            real w = scalarTriple(pq, pa, pd);
            if(w < real(0))
                return false;
            v = -v;

            // Compute t
            if(!intersect_ray_plane(r, d, a, c, t))
                return false;
            if(t < 0)
                return false;

            n = u*na + v*nd + w*nc;
        }

        return true;
    }
}

namespace Raytracer2
{
    ////////////////////////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////////////////////////

    bool Face::intersect(const glm::ray & ray, real maxd, Intersection & isec) const
    {
        if(size() == 3) // triangle
        {
            const glm::vec3 & v1 = mesh->vertices[vIndices[0]];
            const glm::vec3 & v2 = mesh->vertices[vIndices[1]];
            const glm::vec3 & v3 = mesh->vertices[vIndices[2]];

            // If we have vertex normals, compute interpolated normal
            if(hasVertexNormals() && mesh->smoothShading)
            {
                const glm::vec3 & n1 = mesh->normals[nIndices[0]];
                const glm::vec3 & n2 = mesh->normals[nIndices[1]];
                const glm::vec3 & n3 = mesh->normals[nIndices[2]];

                glm::vec3 n;
                real t;
                if(::intersect_ray_triangle(ray, v1, v2, v3, n1, n2, n3, n, t) && t < maxd)
                {
                    isec.hit(ray, t);
                    isec.normal = n;

                    return true;
                }
            }
            else
            {
                real t;
                if(::intersect_ray_triangle(ray, v1, v2, v3, t) && t < maxd)
                {
                    isec.hit(ray, t);
                    isec.normal = normal;

                    return true;
                }
            }
        }
        else if(size() == 4) // quadrilateral
        {
            const glm::vec3 & v1 = mesh->vertices[vIndices[0]];
            const glm::vec3 & v2 = mesh->vertices[vIndices[1]];
            const glm::vec3 & v3 = mesh->vertices[vIndices[2]];
            const glm::vec3 & v4 = mesh->vertices[vIndices[3]];

            // If we have vertex normals, compute interpolated normal
            if(hasVertexNormals() && mesh->smoothShading)
            {
                const glm::vec3 & n1 = mesh->normals[nIndices[0]];
                const glm::vec3 & n2 = mesh->normals[nIndices[1]];
                const glm::vec3 & n3 = mesh->normals[nIndices[2]];
                const glm::vec3 & n4 = mesh->normals[nIndices[3]];

                glm::vec3 n;
                real t;
                if(::intersect_ray_quad(ray, v1, v2, v3, v4, n1, n2, n3, n4, n, t) && t < maxd)
                {
                    isec.hit(ray, t);
                    isec.normal = n;

                    return true;
                }
            }
            else
            {
                real t;
                if(::intersect_ray_quad(ray, v1, v2, v3, v4, t) && t < maxd)
                {
                    isec.hit(ray, t);
                    isec.normal = normal;

                    return true;
                }
            }
        }

        return false;
    }

    glm::vec3 Face::computeNormal() const
    {
        const glm::vec3 *n1, *n2, *n3;

        if(size() == 3)
        {
            n1 = &mesh->vertices[vIndices[0]];
            n2 = &mesh->vertices[vIndices[1]];
            n3 = &mesh->vertices[vIndices[2]];
        }
        else if(size() == 4)
        {
            // triangle (v1, v2, v4)
            // Assumes that the quadrilateral is planar, so only need to calculate for first triangle
            n1 = &mesh->vertices[vIndices[0]];
            n2 = &mesh->vertices[vIndices[1]];
            n3 = &mesh->vertices[vIndices[3]];
        }
        else
        {
            Pane::error("computeNormals() not implemented for face with %d vertices", size());
            return glm::vec3();
        }

        glm::vec3 n = glm::cross(*n2 - *n1, *n3 - *n1);
        glm::vec3::value_type sqr = n.x*n.x + n.y*n.y + n.z*n.z;
        if(sqr < epsilon)
        {
            Pane::warning("Degenerate triangle detected");
            return glm::vec3(0);
        }

        return n * glm::inversesqrt(sqr);
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////////////////////////

    MeshImporterRegistry::MeshImporterRegistry()
    {}

    MeshImporterRegistry & MeshImporterRegistry::instance()
    {
        static MeshImporterRegistry instance;
        return instance;
    }

    void MeshImporterRegistry::registerImporter(MeshImporterPtr importer)
    {
        m_importers.push_back(importer);
    }

    MeshImporterPtr MeshImporterRegistry::find(const std::string & filename) const
    {
        const std::string ext = ::ext(filename);

        auto it = std::find_if(m_importers.begin(), m_importers.end(),
            [&filename, &ext] (const MeshImporterPtr & importer) { return importer->accepts(ext); });

        if(it != m_importers.end())
            return *it;

        return nullptr;
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////////////////////////

    MeshPtr MeshImporter::load(const std::string & filename, int flags)
    {
        MeshImporterPtr importer = MeshImporterRegistry::instance().find(filename);
        if (importer)
            return importer->loadFromFile(filename, flags);

        Pane::error("Could not find importer for: %s", filename.c_str());
        return nullptr;
    }

    

    
}
