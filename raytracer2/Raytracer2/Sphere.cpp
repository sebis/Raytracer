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

#include "Sphere.h"

#include "Constants.h"
#include "SurfacePoint.h"

#include <glm/gtx/intersect.hpp>

namespace
{
    using namespace Raytracer2;

    bool intersectRaySphere(const glm::vec3 & rayOrigin, const glm::vec3 & rayNormDir,
        const glm::vec3 & sc, real r2, real & t0, real & t1)
    {
        const glm::vec3 & p = rayOrigin;
        const glm::vec3 & d = rayNormDir;
        const real zero = real(0);

        const glm::vec3 m = p - sc;

        const real b = glm::dot(m, d);
        const real c = glm::dot(m, m) - r2;
        if(c > zero && b > zero)
            return false;
        const real discr = b*b - c;
        if(discr < zero)
            return false;
        else if(discr > zero)
        {
            // two roots
            const real sqrt = glm::sqrt(discr);
            t0 = glm::max(zero, -b - sqrt);
            t1 = -b + sqrt;
        }
        else
        {
            // one root
            t0 = glm::max(zero, -b);
            t1 = -b;
            
        }
        return true;
    }
}

namespace Raytracer2
{
    Sphere::Sphere()
        : m_center(glm::vec3())
        , m_radius(1)
    {
    }
    Sphere::Sphere(const glm::vec3 & center, glm::float_t radius)
        : m_center(center)
        , m_radius(radius)
    {
    }
    
    bool Sphere::intersect(const glm::ray & ray, real & t0, real & t1) const
    {
        return ::intersectRaySphere(ray.origin, ray.dir, m_center, m_radius*m_radius, t0, t1);
    }

    bool Sphere::hit(const glm::ray & ray, SurfacePoint & sp, real & t) const
    {
        if (glm::intersectRaySphere(ray.origin, ray.dir, m_center, m_radius*m_radius, t))
        {
            sp.point = ray.origin + t*ray.dir;
            return true;
        }
        return false;
    }

    void Sphere::hitSurfacePoint(SurfacePoint & sp) const
    {
        sp.point = sp.point;
        sp.normal = glm::normalize(sp.point - m_center);
        sp.uv = glm::vec2(0, 0);
    }

    std::ostream & operator<<(std::ostream & out, const Sphere & s)
    {
        out << s.m_center.x << ","
            << s.m_center.y << ","
            << s.m_center.z << ","
            << s.m_radius;
        return out;
    }
}