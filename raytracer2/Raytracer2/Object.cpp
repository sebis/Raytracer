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

#include "Object.h"

#include "SurfacePoint.h"

#include <glm/glm.hpp>

namespace Raytracer2
{
    Object::Object(const Transform & transform, const Geometry & geometry, const Shader & shader)
        : m_transform(transform)
        , m_geometry(geometry)
        , m_shader(shader)
    {
    }

    bool Object::hit(const glm::ray & ray, SurfacePoint & sp, real & t) const
    {
        const glm::mat4 & T = m_transform.transformInv();
        
        const glm::vec3 & p = glm::vec3(T * glm::vec4(ray.origin, 1));
        const glm::vec3 & d = glm::vec3(T * glm::vec4(ray.dir, 0));

        // TODO: is this renormalization of t correct?
        const real invd = 1 / glm::length(d);

        bool success = m_geometry.hit(glm::ray(p, d * invd), sp, t);
        t *= invd;

        return success;
    }

    void Object::hitSurfacePoint(SurfacePoint & sp) const
    {
        // assume sp is in object space
        m_geometry.hitSurfacePoint(sp);

        // convert sp to world space
        auto T = m_transform.transform();
        auto invT = m_transform.transformInv();

        sp.point = glm::vec3(T * glm::vec4(sp.point, 1.0));

        const glm::mat4 & nT = glm::transpose(invT);
        sp.normal = glm::normalize(glm::vec3(nT * glm::vec4(sp.normal, 0.0)));

        sp.shader = &m_shader;
    }
}