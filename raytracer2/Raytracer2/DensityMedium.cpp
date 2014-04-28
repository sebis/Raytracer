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

#include "DensityMedium.h"

namespace Raytracer2
{
    DensityMedium::DensityMedium(
            const Geometry & geometry, const PhaseFunction & phaseFunction, const Density & density,
            real sigma_a, real sigma_s, const glm::vec3 & emission)
        : Medium(geometry, phaseFunction)
        , m_density(density)
        , m_sigma_a(sigma_a)
        , m_sigma_s(sigma_s)
        , m_emission(emission)
    {}

    real DensityMedium::sigma_a(const glm::vec3 & point) const
    {
        return m_density.sample(point) * m_sigma_a;
    }

    real DensityMedium::sigma_s(const glm::vec3 & point) const
    {
        return m_density.sample(point) * m_sigma_s;
    }
        
    real DensityMedium::sigma_t_max() const
    {
        return m_density.max() * (m_sigma_a + m_sigma_s);
    }

    glm::vec3 DensityMedium::emit(const glm::vec3 & point) const
    {
        return m_density.max() * m_emission;
    }

    real DensityMedium::tau(const glm::ray & ray, real t0, real t1) const
    {
        // Use trapezoidal rule to compute approximation
        const real a = t0;
        const real b = t1;

        const unsigned N = 10;
        const real h = (b - a) / N;

        real sum = real(0);
        sum += sigma_t(ray.point(a));
        for(int k = 1; k < N; k++)
        {
            const real t = a + k * h;
            sum += 2*sigma_t(ray.point(t));
        }
        sum += sigma_t(ray.point(b));

        const real tau = sum * (b - a) / (2*N);

        return tau;
    }
}
