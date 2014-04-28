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

#include "Sampling.h"

#include "Constants.h"
#include "GLMExtensions.h"

#include <glm/gtc/constants.hpp>
#include <glm/gtc/random.hpp>

#include <random>

namespace
{
    using namespace Raytracer2;
    using namespace Raytracer2::Sampling;

    // Global random generators
    std::default_random_engine generator;
    std::uniform_real_distribution<real> uniform_distribution(real(0), real(1));

    // Rotates v to be aligned with axis
    glm::vec3 transform(const glm::vec3 & axis, const glm::vec3 & v)
    {
        return glm::rotate(glm::vec3(0, 0, 1), axis) * v;
    }

    // Generate random direction on unit hemisphere proportional to solid angle
    // from Global Illumination Compendium item 34
    Sample sampleUnitHemisphereSolidAngle()
    {
        Sample sample;
        glm::float_t x, y, z;

        // random numbers in [0,1]
        const glm::float_t r1 = uniform(), r2 = uniform();

        const glm::float_t omega = two_pi * r1;
        const glm::float_t s = glm::sqrt(1 - r2*r2);

        x = glm::cos(omega) * s;
        y = glm::sin(omega) * s;
        z = r2;

        sample.dir = glm::vec3(x, y, z);
        sample.pdf = 1 / two_pi;

        return sample;
    }

    // Generate random direction on unit hemisphere proportional to cosine-weighted solid angle
    // from Global Illuination Compendium item 35
    Sample sampleUnitHemisphereCosineSolidAngle()
    {
        Sample sample;
        glm::float_t x, y, z;

        // random numbers in [0,1]
        const glm::float_t r1 = uniform(), r2 = uniform();

        const glm::float_t omega = two_pi * r1;
        const glm::float_t r = glm::sqrt(r2);

        x = glm::cos(omega) * r;
        y = glm::sin(omega) * r;
        z = glm::sqrt(1 - r2);

        sample.dir = glm::vec3(x, y, z);
        sample.pdf = z / pi;

        return sample;
    }

    // Generate random direction on unit hemisphere proportional to cosine lobe around normal
    // from Global Illumination Compendium item 36
    Sample sampleUnitHemisphereCosineLobeAroundNormal(glm::float_t cos_theta, glm::float_t n)
    {
        Sample sample;
        glm::float_t x, y, z;

        //// random numbers in [0,1]
        const glm::float_t r1 = uniform(), r2 = uniform();

        const glm::float_t omega = two_pi * r1;
        const glm::float_t s = glm::sqrt(1 - glm::pow(r2, 2 / (n + 1)));

        x = glm::cos(omega) * s;
        y = glm::sin(omega) * s;
        z = glm::pow(r2, 1 / (n + 1));

        sample.pdf = ((n + 1) / two_pi) * glm::pow(cos_theta, n);
        sample.dir = glm::vec3(x, y, z);

        return sample;
    }

    // Generate random direction on spherical digon; density proportional to cos_alpha^n; alpha angle from off-normal axis
    // from Global Illumination Compendium item 38
    Sample sampleUnitHemisphereSpecularLobeBRDF(glm::float_t n)
    {
        Sample sample;
        glm::float_t x, y, z;

        //// random numbers in [0,1]
        const glm::float_t r1 = uniform(), r2 = uniform();

        const glm::float_t omega = two_pi * r1;
        const glm::float_t s = glm::sqrt(1 - glm::pow(r2, 2 / (n + 1)));

        x = glm::cos(omega) * s;
        y = glm::sin(omega) * s;
        z = glm::pow(r2, 1 / (n + 1));

        sample.dir = glm::vec3(x, y, z);

        const glm::float_t cos_alpha = z;
        sample.pdf = ((n + 1) / two_pi) * glm::pow(cos_alpha, n);

        return sample;
    }

    Sample sampleUnitSphereUniform()
    {
        Sample sample;
        real x, y, z;

        // random numbers in [0,1]
        const real r1 = uniform(), r2 = uniform();
        
        const real omega = two_pi * r1; // Map to [0, 2PI]
        const real u = real(2) * (r2 - real(0.5)); // Map to [-1, 1]
        const real s = glm::sqrt(1 - u*u);

        x = glm::cos(omega) * s;
        y = glm::sin(omega) * s;
        z = u;

        sample.dir = glm::vec3(x, y, z);
        sample.pdf = 1 / four_pi;

        return sample;
    }

    Sample1D sampleExponentialDistribution(real lambda)
    {
        Sample1D sample;
        real t;

        // random number in [0,1)
        const real r1 = uniform();

        t = -glm::log(r1) / lambda;
        
        sample.t = t;
        sample.pdf = lambda*glm::exp(-lambda * t);

        return sample;
    }
}

namespace Raytracer2
{
    namespace Sampling
    {
        Sample lambert(const glm::vec3 & normal)
        {
            Sample sample = ::sampleUnitHemisphereCosineSolidAngle();
            sample.dir = ::transform(normal, sample.dir);
            return sample;
        }

        Sample phong(const glm::vec3 & specularDir, glm::float_t n)
        {
            Sample sample = ::sampleUnitHemisphereSpecularLobeBRDF(n);
            sample.dir = ::transform(specularDir, sample.dir);
            return sample;
        }

        Sample hemisphere(const glm::vec3 & normal)
        {
            Sample sample = ::sampleUnitHemisphereSolidAngle();
            sample.dir = ::transform(normal, sample.dir);
            return sample;
        }

        Sample isotropic()
        {
            return ::sampleUnitSphereUniform();
        }

        Sample1D distance(real sigma)
        {
            return ::sampleExponentialDistribution(sigma);
        }

        Sample1D woodcock(const glm::ray & ray, const Medium & m)
        {
            Sample1D sample;

            const real sigma_t_max = m.sigma_t_max();

            real t = -glm::log(uniform()) / sigma_t_max;
            while((m.sigma_t(ray.point(t)) / sigma_t_max) < uniform())
                t -= glm::log(uniform()) / sigma_t_max;

            sample.t = t;

            const real sigma_t = m.sigma_t(ray.point(t));
            const real tau = m.tau(ray, 0, t);
            sample.pdf = sigma_t * glm::exp(-tau);

            return sample;
        }

        real uniform()
        {
            return uniform_distribution(generator);

            // The following actually generate numbers between [0,1] instead of [0,1)
            // and the numbers are actually skewed towards small values
            //return glm::linearRand(real(0), real(1));
            //return rand() / (real)RAND_MAX;
        }

        glm::vec2 uniformArea()
        {
            return glm::vec2(uniform(), uniform());
        }
    }
}
