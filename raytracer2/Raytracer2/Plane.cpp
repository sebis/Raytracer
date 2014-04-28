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

#include "Plane.h"

#include "Constants.h"
#include "SurfacePoint.h"

namespace
{
    using namespace Raytracer2;

    bool hit(const glm::vec3 & rp, const glm::vec3 & rd, const glm::vec3 & n, glm::float_t d, glm::float_t & t)
    {
        const real dn = glm::dot(rd, n);
        if (dn < zero)
        {
            const real pn = glm::dot(rp, n);
	        t = -(d + pn) / dn;
	        if (t >= zero)
	            return true;
        }
        return false;
    }

    bool intersect(const glm::vec3 & rp, const glm::vec3 & rd, const glm::vec3 & n, const real d, real & t0, real & t1)
    {
        const real dn = glm::dot(rd, n);
        if(dn == zero) // parallel to plane
        {
            t0 = zero;
            t1 = infinity;
            return true;
        }

        const real t = (d - glm::dot(n, rp)) / dn;
        if (dn > zero) // same direction as plane normal
        {
            if(t < 0) // check if ray starts on the positive side of plane
                t0 = zero;
            else
                t0 = t;
            t1 = infinity;
            return true;
        }
        else if (dn < zero) // opposite direction as plane normal
        {
            if(t < 0) // starts on negative side and points away
                return false; 

            t0 = zero;
            t1 = t;
            return true;
        }
        
        return false; // shouldn't happen
    }
}

namespace Raytracer2
{
    Plane::Plane(const glm::vec3 & n, glm::float_t d)
        : m_n(n), m_d(d)
    {
    }

    bool Plane::intersect(const glm::ray & ray, real & t0, real & t1) const
    {
        return ::intersect(ray.origin, ray.dir, m_n, m_d, t0, t1);
    }

    bool Plane::hit(const glm::ray & ray, SurfacePoint & sp, real & t) const
    {
        if(::hit(ray.origin, ray.dir, m_n, m_d, t))
        {
            sp.point = ray.origin + t*ray.dir;
            return true;
        }
        return false;
    }

    void Plane::hitSurfacePoint(SurfacePoint & sp) const
    {
        sp.point = sp.point;
        sp.normal = m_n;
        sp.uv = glm::vec2(0, 0);
    }
}