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

#include "PhongShader.h"

#include "Constants.h"
#include "GLMExtensions.h"
#include "Sampling.h"
#include "SurfacePoint.h"

#include <glm/gtc/constants.hpp>
#include <glm/gtc/matrix_transform.hpp>

namespace 
{
    using namespace Raytracer2;

    // cos_alpha is cos(Or, Os)
    glm::vec3 brdf(const glm::vec3 & kS, glm::float_t n, glm::float_t cos_alpha)
    {
        cos_alpha = glm::max(glm::float_t(0), cos_alpha);
        return kS * ((n + 2) / two_pi) * glm::pow(cos_alpha, n);
    }
}

namespace Raytracer2
{
    PhongShader::PhongShader(const glm::vec3 & diffuse, const glm::vec3 & specular, glm::float_t shininess)
        : m_diffuse(diffuse)
        , m_specular(specular)
        , m_shininess(shininess)
    {
        type = Diffuse;
    }

    glm::vec3 PhongShader::eval(const SurfacePoint & sp,
                               const glm::vec3 & in,
                               const glm::vec3 & out) const
    {

        const glm::float_t cos_alpha = glm::dot(out, sp.reflect(in));

        const glm::vec3 diff = (m_diffuse / glm::pi<glm::float_t>());
        return diff + ::brdf(m_specular, m_shininess, cos_alpha);
    }

    void PhongShader::generateDir(const SurfacePoint & sp,
                                  const glm::vec3 & out,
                                  unsigned numSamples,
                                  BSDFSamples & samples) const
    {
        samples.resize(numSamples);

        const glm::float_t pd = glm::max(m_diffuse.x, glm::max(m_diffuse.y, m_diffuse.z));
        const glm::float_t ps = glm::max(m_specular.x, glm::max(m_specular.y, m_specular.z));

        for (unsigned i = 0; i < numSamples; ++i)
        {
            // use russian roulette to send specular or diffuse reflection
            const glm::float_t roulette = Sampling::uniform();
            if(roulette < pd)
            {
                // diffuse
                const auto sample = Sampling::lambert(sp.normal);
                const glm::vec3 brdf = (1 / pd) * m_diffuse / glm::pi<glm::float_t>();

                samples[i] = BSDFSample(sample.dir, brdf, sample.pdf);
            }
            else if(roulette < pd + ps)
            {
                // specular
                const glm::vec3 r = sp.reflect(out);
                const auto sample = Sampling::phong(r, m_shininess);
                
                const glm::float_t cos_alpha = glm::dot(r, sample.dir);                
                const glm::vec3 brdf = (1 / (pd + ps)) * ::brdf(m_specular, m_shininess, cos_alpha);

                samples[i] = BSDFSample(sample.dir, brdf, sample.pdf);
            }
            else
            {
                // no radiance is reflected
                samples[i] = BSDFSample();
            }
        }
    }
}