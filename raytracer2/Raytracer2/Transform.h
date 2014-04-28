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

#ifndef TRANSFORM_H
#define TRANSFORM_H

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

namespace Raytracer2
{
    class Transform
    {
    public:
        Transform(const glm::vec3 & position = glm::vec3(0.0f),
            const glm::quat & rotation = glm::quat(),
            const glm::vec3 & scale = glm::vec3(1.0f));

        const glm::vec3 & position() const;
        void setPosition(const glm::vec3 & position);

        const glm::quat & rotation() const;
        void setRotation(const glm::quat & rotation);

        const glm::vec3 & scale() const;
        void setScale(const glm::vec3 & scale);

        const glm::mat4 & transform() const;
        void setTransform(const glm::mat4 & transform);

        const glm::mat4 & transformInv() const;

    private:
        glm::vec3 m_position;
        glm::quat m_rotation;
        glm::vec3 m_scale;

        mutable glm::mat4 m_transform;
        mutable glm::mat4 m_transformInv;

        mutable bool m_cached;
        mutable bool m_cachedInv;
    };
}

#endif // TRANSFORM_H