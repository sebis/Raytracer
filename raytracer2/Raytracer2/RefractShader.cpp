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

#include "RefractShader.h"

#include "Constants.h"
#include "Sampling.h"
#include "SurfacePoint.h"
#include <Pane/Trace.h>
#include <glm/gtc/constants.hpp>
#include <glm/gtc/epsilon.hpp>

namespace
{
    glm::float_t schlick(glm::float_t n1, glm::float_t n2, glm::float_t cosi)
    {
        glm::float_t R0 = (n1 - n2) / (n1 + n2);
        R0 = R0 * R0;

        const glm::float_t c = (1 - cosi);
        return R0 + (1 - R0) * c * c * c * c * c;
    }

    // Calculates the Fresnel reflection coefficient for unpolarized light
    glm::float_t fresnel(glm::float_t n1, glm::float_t n2, glm::float_t cosi, glm::float_t cost)
    {
        const glm::float_t Rs = (n1 * cosi - n2 * cost) / (n1 * cosi + n2 * cost);
        const glm::float_t Rp = (n2 * cosi - n1 * cost) / (n2 * cosi + n1 * cost);
        return 0.5 * (Rs * Rs + Rp * Rp);
    }

    bool refract(glm::float_t n1, glm::float_t n2, const glm::vec3 & normal,
        const glm::vec3 & incident, glm::vec3 & refracted, glm::float_t & refl)
    {
        const glm::float_t n = n1 / n2;

        const glm::float_t cosi = glm::dot(-incident, normal);
        const glm::float_t cost2 = 1 - n * n * (1 - cosi * cosi);

        // check for total internal reflection
        if (cost2 < 0)
            return false;
            
        const glm::float_t cost = glm::sqrt(cost2);

        refracted = n * incident + (n * cosi - cost) * normal;
        refl = fresnel(n1, n2, cosi, cost);
        //refl = schlick(n1, n2, cosi);

        return true;
    }

    glm::vec3 reflect(const glm::vec3 & normal, const glm::vec3 & incident)
    {
        return incident - glm::float_t(2) * normal * glm::dot(normal, incident);
    }
}

namespace Raytracer2
{
    RefractShader::RefractShader(float index, const glm::vec3 & reflectance, const glm::vec3 & transmittance)
        : m_index(index)
        , m_reflectance(reflectance)
        , m_transmittance(transmittance)
    {
        type = Shader::Mirror;
    }

    glm::vec3 RefractShader::eval(const SurfacePoint & point,
                                          const glm::vec3 & inDir,
                                          const glm::vec3 & outDir) const
    {
        return glm::vec3(0);
    }

    void RefractShader::generateDir(const SurfacePoint & sp,
                                const glm::vec3 & out,
                                unsigned numSamples,
                                BSDFSamples & samples) const
    {
        samples.resize (numSamples);

        // assume refraction index 1.0 for outer medium
        glm::float_t n1 = 1.0f;
        glm::float_t n2 = m_index;

        // normal should point to medium n1
        glm::vec3 normal = sp.normal;
        const glm::vec3 incident = -out;

        // Flip normal and count new n if the ray is coming from 
        // inside the surface (ie. from n2 to n1)
        if (sp.cos_theta(out) < 0) {
            normal = -normal;
            std::swap(n1, n2);
        }

        // n = n1 / n2 where ray comes from n1 to n2
        /*const glm::float_t n = n1 / n2;
        const glm::float_t cosI = glm::dot(normal, -incident);*/

        for (unsigned i = 0; i < numSamples; ++i)
        {
            glm::vec3 result, value;
                
            glm::vec3 refracted;
            glm::float_t refl;

            if(refract(n1, n2, normal, incident, refracted, refl))
            {
                // always reflect
                //refl = 1.0f;
                // always transmit
                //refl = 0.0f;

                assert(0 <= refl && refl <= 1);
                // choose between refracted and reflected rays based on fresnel term
                const glm::float_t roulette = Sampling::uniform();
                if(roulette < refl)
                {
                    // reflect
                    result = reflect(normal, incident);
                    value = (1 / refl) * (refl) * m_reflectance / sp.cos_theta(result);
                }
                else
                {
                    const real cos_theta = glm::dot(normal, result);
                    // transmit
                    result = refracted;
                    value = (1 / (1 - refl)) * (1 - refl) * m_transmittance / sp.cos_theta(result);
                }
            }
            else
            {
                result = reflect(normal, incident);
                value = m_reflectance / sp.cos_theta(result);
            }
            
            samples[i] = BSDFSample(result, value, 1);
        }
    }

    glm::float_t RefractShader::absorption() const
    {
        glm::float_t refl = glm::max(m_reflectance.x, glm::max(m_reflectance.y, m_reflectance.z));
        glm::float_t trans = glm::max(m_transmittance.x, glm::max(m_transmittance.y, m_transmittance.z));
        return glm::max(refl, trans);
    }
}