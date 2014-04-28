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

#include "LambertShader.h"

#include "GLMExtensions.h"
#include "Sampling.h"
#include "SurfacePoint.h"

#include <glm/gtc/random.hpp>
#include <glm/gtc/constants.hpp>
#include <glm/gtc/matrix_transform.hpp>

namespace
{
    glm::vec3 CosineSampleHemisphere(float u1, float u2)
    {
        const float r = glm::sqrt(u1);
        const float theta = 2 * glm::pi<float>() * u2;
 
        const float x = r * glm::cos(theta);
        const float y = r * glm::sin(theta);
 
        return glm::vec3(x, y, glm::sqrt(glm::max(0.0f, 1 - u1)));
    }

    glm::vec3 sample()
    {
        const float r1 = glm::linearRand(0.f, 1.f);
        const float r2 = glm::linearRand(0.f, 1.f);

        const float pi = glm::pi<float>();

        const float x = glm::cos(2.f * pi * r1) * glm::sqrt(1.f - r2);
        const float y = glm::sin(2.f * pi * r1) * glm::sqrt(1.f - r2);
        const float z = glm::sqrt(r2);

        return glm::vec3(x, y, z);
    }
}

namespace Raytracer2
{
    LambertShader::LambertShader()
        : m_diffuse(glm::vec3(1, 1, 1))
    {
        type = Type::Diffuse;
    }

    LambertShader::LambertShader(const glm::vec3 & diffuse)
        : m_diffuse(diffuse)
    {
        type = Type::Diffuse;
    }

    glm::vec3 LambertShader::eval(const SurfacePoint & sp,
                            const glm::vec3 & in,
                            const glm::vec3 & out) const
    {
        return m_diffuse / pi;
    }

    void LambertShader::generateDir(const SurfacePoint & point,
                                const glm::vec3 & outDir,
                                unsigned numSamples,
                                BSDFSamples & samples) const
    {
        samples.resize(numSamples);

        for(unsigned i = 0; i < samples.size(); ++i)
        {
            const Sampling::Sample sample = Sampling::lambert(point.normal);
            const glm::vec3 value = m_diffuse / pi;

            samples[i] = BSDFSample(sample.dir, value, sample.pdf);
        }
    }

    const glm::vec3 & LambertShader::diffuse() const
    {
        return m_diffuse;
    }

    void LambertShader::setDiffuse(const glm::vec3 & diffuse)
    {
        m_diffuse = diffuse;
    }

    glm::float_t LambertShader::absorption() const
    {
        return glm::max(m_diffuse.x, glm::max(m_diffuse.y, m_diffuse.z));
    }

    glm::vec3 LambertShader::reflectivity() const
    {
        return m_diffuse;
    }
}
