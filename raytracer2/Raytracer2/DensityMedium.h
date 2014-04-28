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

#ifndef NOISE_MEDIUM_H
#define NOISE_MEDIUM_H

#include "Medium.h"
#include "noise1234.h"

namespace Raytracer2
{
    struct Density
    {
        virtual real sample(const glm::vec3 & point) const
        {
            return real(0.1);
        }

        virtual real max() const
        {
            return real(0.1);
        }
    };

    struct NoiseDensity : public Density
    {
        NoiseDensity(real size)
            : m_size(size)
        {}

        real sample(const glm::vec3 & point) const override
        {
            const glm::vec3 p = m_size * point;
            return Noise1234::noise(p.x, p.y, p.z);

            // map to [0, 1]
            /*glm::vec3 p = real(0.5) * point * real(0.1) + real(0.5);
            p *= real(10);
        
            real noise = Noise1234::noise(p.x, p.y, p.z);

            // map from [-1, 1] to [0, 1]
            noise = real(0.5) * noise + real(0.5);
            noise = logistics(noise);
            noise *= real(0.25);
        
            return noise;*/
        }

    private:
        real m_size;

        /*real exponentialEaseIn(real t) const
        {
            return powf(2.f, 10.f * (t - 1.f) );
        }

        real logistics(real t) const
        {
            real w = real(0.05);
            return 0.5 + 0.5 * tanh((t - 0.5) / (2.0 * w));
        }*/
    };

    class DensityMedium : public Medium
    {
    public:
        DensityMedium(
            const Geometry & geometry, const PhaseFunction & phaseFunction, const Density & density,
            real sigma_a, real sigma_s, const glm::vec3 & emission = glm::vec3(0));

        // Returns absorption coefficient at point x
        virtual real sigma_a(const glm::vec3 & point) const override;
        // Returns scattering coefficient at point x
        virtual real sigma_s(const glm::vec3 & point) const override;
        
        // Returns the max sigma_t in the whole medium
        virtual real sigma_t_max() const override;

        virtual glm::vec3 emit(const glm::vec3 & point) const override;

        virtual real tau(const glm::ray & ray, real t0, real t1) const override;

    private:
        const Density & m_density;

        real m_sigma_a;
        real m_sigma_s;

        glm::vec3 m_emission;
    };
}

#endif // NOISE_MEDIUM_H