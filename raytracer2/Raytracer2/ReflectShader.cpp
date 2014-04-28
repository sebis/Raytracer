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

#include "ReflectShader.h"

#include "GLMExtensions.h"
#include "SurfacePoint.h"

namespace Raytracer2
{
    ReflectShader::ReflectShader(const glm::vec3 & reflectance)
        : m_reflectance(reflectance)
    {
        type = Shader::Mirror;
    }

    glm::vec3 ReflectShader::eval(const SurfacePoint & sp,
                                  const glm::vec3 & in,
                                  const glm::vec3 & out) const
    {
        // brdf is a dirac-delta function so no point in evaluating it 
        return glm::vec3(0);
    }

    void ReflectShader::generateDir(const SurfacePoint & sp,
                                    const glm::vec3 & out,
                                    unsigned numSamples,
                                    BSDFSamples & samples) const
    {
        samples.resize(1);

        const glm::vec3 reflect = sp.reflect(out);

        // brdf: d(theta_i-theta_o)d(phi_i-phi_o) / cos(theta_i)
        const glm::vec3 value = m_reflectance / sp.cos_theta(out);

        samples[0] = BSDFSample(reflect, value, 1);
    }

    glm::float_t ReflectShader::absorption() const
    {
        return glm::max(m_reflectance.x, glm::max(m_reflectance.y, m_reflectance.z));
    }
}