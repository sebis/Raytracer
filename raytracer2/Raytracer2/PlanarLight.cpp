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

#include "PlanarLight.h"

#include "Sampling.h"

#include <glm/gtx/fast_square_root.hpp>

namespace Raytracer2
{
    PlanarLight::PlanarLight(const glm::vec3 & center,
                             const glm::vec3 & axis0,
                             const glm::vec3 & axis1,
                             const glm::vec3 & intensity)
                             : m_center(center), m_intensity(intensity)
    {
        m_axes[0] = axis0;
        m_axes[1] = axis1;

        m_normal = glm::cross(glm::normalize(m_axes[0]), glm::normalize(m_axes[1]));
        m_area = (2 * glm::length(m_axes[0])) * (2 * glm::length(m_axes[1]));
    }

    void PlanarLight::illuminate(const glm::vec3 & point,
                                 unsigned numSamples,
                                 LightSamples & samples) const
    {
        samples.resize(numSamples);

        const unsigned n = (unsigned)glm::sqrt(float(numSamples));

        const glm::vec3 d0 = (2.0 / n) * m_axes[0];
        const glm::vec3 d1 = (2.0 / n) * m_axes[1];

        const glm::vec3 min = m_center - m_axes[0] - m_axes[1];

        for (unsigned i = 0; i < n; ++i)
        {
            for (unsigned j = 0; j < n; ++j)
            {
                const glm::vec2 offset = Sampling::uniformArea();

                const glm::float_t x = i + offset.x;
                const glm::float_t y = j + offset.y;
                const glm::vec3 p = min + x * d0 + y * d1;

                // from surface point to light sample
                glm::vec3 dir = p - point;
                const real d = glm::length(dir);

                // normalize
                dir *= (1.f / d);

                const glm::float_t cos_theta = glm::max(real(0), glm::dot(m_normal, -dir));
                
                // sampling probability
                const glm::float_t pdf = 1.f / m_area;
                
                const glm::vec3 radiance = cos_theta * (1.f / (d * d)) * m_intensity;

                samples[i + n*j] = LightSample(p, dir, d, pdf, radiance);
            }
        }
    }

    void PlanarLight::generateDir(unsigned numSamples,
                                  LightSamples & samples) const
    {
        samples.resize(numSamples);

        const glm::vec3 min = m_center - m_axes[0] - m_axes[1];

        for(int i = 0; i < numSamples; i++)
        {
            // Map to [-1, 1]
            const glm::vec2 offset = (Sampling::uniformArea() - real(0.5)) * real(2);
            const glm::vec3 point = m_center + offset[0] * m_axes[0] + offset[1] * m_axes[1];

            const Sampling::Sample sample = Sampling::hemisphere(m_normal);

            samples[i].point = point;
            samples[i].radiance = m_intensity;
            samples[i].direction = sample.dir;
            samples[i].pdf = (real(1) / m_area) * sample.pdf;
        }

    }

}
