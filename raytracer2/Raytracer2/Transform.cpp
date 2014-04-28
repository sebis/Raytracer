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

#include "Transform.h"

#include <glm/gtc/matrix_transform.hpp>

#include <utility>

namespace Raytracer2
{
    Transform::Transform(const glm::vec3 & position,
                         const glm::quat & rotation,
                         const glm::vec3 & scale)
        : m_position(position)
        , m_rotation(rotation)
        , m_scale(scale)
        , m_cached(false)
        , m_cachedInv(false)
    {
    }

    const glm::vec3 & Transform::position() const
    {
        return m_position;
    }
    void Transform::setPosition(const glm::vec3 & position)
    {
        m_position = position;
        m_cached = false;
    }

    const glm::quat & Transform::rotation() const
    {
        return m_rotation;
    }

    void Transform::setRotation(const glm::quat & rotation)
    {
        m_rotation = rotation;
        m_cached = false;
    }

    const glm::vec3 & Transform::scale() const
    {
        return m_scale;
    }
    void Transform::setScale(const glm::vec3 & scale)
    {
        m_scale = scale;
        m_cached = false;
    }

    const glm::mat4 & Transform::transform() const
    {
        if(m_cached)
            return m_transform;
        
        const glm::mat4 & m =
            glm::scale(glm::rotate(glm::translate(glm::mat4(),
                m_position),
                glm::angle(m_rotation), glm::axis(m_rotation)),
                m_scale);

        m_transform = std::move(m);
        m_cached = true;

        return m_transform;
    }

    void Transform::setTransform(const glm::mat4 & transform)
    {
        m_transform = transform;
        m_cached = true;
    }

    const glm::mat4 & Transform::transformInv() const
    {
        if(m_cachedInv)
            return m_transformInv;

        const glm::mat4 & m = glm::inverse(transform());

        m_transformInv = std::move(m);
        m_cachedInv = true;

        return m_transformInv;
    }
}
