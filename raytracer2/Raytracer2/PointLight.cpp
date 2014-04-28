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

#include "PointLight.h"

#include "Constants.h"
#include "Sampling.h"

namespace Raytracer2
{
    PointLight::PointLight(const glm::vec3 & position,
                           const glm::vec3 & intensity)
        : m_position(position)
        , m_intensity(intensity)
    {
    }

    void PointLight::illuminate(const glm::vec3 & point,
                    unsigned,
                    LightSamples & samples) const
    {
        glm::vec3 dir = m_position - point;

        const real d = glm::length(dir);
        const real normInv = real(1) / (d*d);

        dir *= (1 / d);

        const glm::vec3 radiance = m_intensity * normInv;

        samples.resize(1);
        samples[0] = LightSample(m_position, dir, d, real(1), radiance);
    }

    void PointLight::generateDir(unsigned numSamples,
                                 LightSamples & samples) const
    {
        samples.resize(numSamples);
        
        for(int i = 0; i < numSamples; i++)
        {
            Sampling::Sample sample = Sampling::isotropic();
            samples[i].direction = sample.dir;
            samples[i].pdf = sample.pdf;
            samples[i].radiance = m_intensity;
        }
    }

    const glm::vec3 & PointLight::position() const
    {
        return m_position;
    }
    void PointLight::setPosition(const glm::vec3 & position)
    {
        m_position = position;
    }

    const glm::vec3 & PointLight::intensity() const
    {
        return m_intensity;
    }
    void PointLight::setIntensity(const glm::vec3 & intensity)
    {
        m_intensity = intensity;
    }
}
