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

#include "Triangle.h"

#include "SurfacePoint.h"

#include <glm/gtx/intersect.hpp>

namespace Raytracer2
{
    Triangle::Triangle(const std::array<glm::vec3, 3> & vertices)
        : m_vertices(vertices)
    {
        const glm::vec3 & v1 = m_vertices[0];
        const glm::vec3 & v2 = m_vertices[1];
        const glm::vec3 & v3 = m_vertices[2];

        m_normal = glm::normalize(glm::cross(glm::normalize(v2 - v1), glm::normalize(v3 - v1)));
    }

    bool Triangle::intersect(const glm::ray & ray, real & t0, real & t1) const
    {
        // intersection with a triangle is simply one point
        glm::vec3 p;
        const bool hit = glm::intersectRayTriangle(ray.origin, ray.dir, m_vertices[0], m_vertices[1], m_vertices[2], p);

        // note: x and y in p contain barycentric coordinates, z is ray scalar factor
        t0 = t1 = p.z;

        return hit;
    }

    bool Triangle::hit(const glm::ray & ray, SurfacePoint & sp, real & t) const
    {
        glm::vec3 p;
        if(glm::intersectRayTriangle(ray.origin, ray.dir, m_vertices[0], m_vertices[1], m_vertices[2], p))
        {
            // note: x and y in p contain barycentric coordinates, z is ray scalar factor
            t = p.z;

            // convert to cartesian coordinates
            sp.point = ray.origin + t*ray.dir;
            return true;
        }
        return false;
    }

    void Triangle::hitSurfacePoint(SurfacePoint & sp) const
    {
        sp.point = sp.point;
        sp.uv = glm::vec2(0, 0);
        sp.normal = m_normal;
    }
}