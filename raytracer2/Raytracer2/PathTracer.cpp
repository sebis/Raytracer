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

#include "PathTracer.h"

#include "Sampling.h"
#include "SurfacePoint.h"
#include "Utils.h"

namespace Raytracer2
{
    PathTracer::PathTracer(const OptionsPtr & opt, const ScenePtr & scene)
        : Renderer(opt, scene)
    {}

    PathTracer::~PathTracer()
    {}

    glm::vec3 PathTracer::radiance(const glm::ray & ray) const
    {
        return trace(ray, std::numeric_limits<float>::max(), 0, false);
    }

    glm::vec3 PathTracer::trace(const glm::ray & ray,
                        glm::float_t maxd, unsigned depth, bool indirect, bool inside) const
    {
        glm::vec3 color;

        SurfacePoint sp;
        if(hitSurface(ray, maxd, sp))
            color = shade(-ray.dir, sp, depth, indirect);
        else
            color = shadeWorld(ray);

        // Ignore medium if we are inside an object (e.g. glass)
        if (!inside)
        {
            VolumeRay vr;
            // returns null if surface is closer than medium
            const Medium * medium = hitMedium(ray, vr, maxd);
            if (medium)
                color = shadeMedium(-ray.dir, color, *medium, vr, depth);
        }

        return color;
    }

    glm::vec3 PathTracer::shade(const glm::vec3 & out,
            const SurfacePoint & sp,
            unsigned depth, bool indirect) const
    {
        glm::vec3 color = sp.emit(out);

        if(opt().s_directLighting && sp.shader->type == Shader::Emissive)
        {
            // discard ray if it is indirect and hits a light, else return emitted radiance
            return indirect ? glm::vec3() : color;
        }

        if(opt().s_directLighting)
            color += directLighting(out, sp);

        real survival;
        if(russianRoulette(sp.shader->absorption(), survival))
        {
            BSDFSamples samples;
            sp.generateDir(out, 1, samples);

            glm::vec3 colorIndirect;

            for(auto const & sample : samples)
            {
                bool indirect = sp.shader->type == Shader::Diffuse;
                bool inside = glm::dot(sp.normal, sample.direction) < 0;

                const glm::ray ray(sp.point, sample.direction, opt().s_fudge);
                const glm::vec3 Lr = trace(ray, max_value, depth+1, indirect, inside);
                const glm::vec3 brdf = sample.value;
                const glm::float_t cos_theta = sp.cos_theta(sample.direction);

                colorIndirect += survival * Lr * brdf * (cos_theta / sample.pdf);
            }

            if(samples.size())
                color += colorIndirect * (1.0/samples.size());
        }

        return color;
    }

    glm::vec3 PathTracer::scatter(const glm::vec3 & point, const glm::vec3 & out, const Medium & medium, const VolumeRay & vr, unsigned depth) const
    {
        // Sample emittance at point
        glm::vec3 Le = medium.emit(point);

        // Incoming light from scattering events
        glm::vec3 Li;

        if(opt().s_singleScattering)
        {
            Li += singleScatter(point, out, medium, vr);
        }
        else // Proceed with recursive volumetric path tracing
        {
            real survival;
            if(russianRoulette(medium.absorption(point), survival))
            {
                glm::vec3 L;

                // Generate scatter rays according to phase function
                PFSamples samples;
                vr.samplePF(out, 1, samples);

                for (auto const & sample : samples)
                {
                    // Create scatter ray from sampled distance
                    const glm::ray ray(point, sample.direction);

                    // phase function value (TODO: should come from PFSample
                    const glm::vec3 pf = vr.eval(point, sample.direction, out);

                    // in-scattered radiance (multiple scattering)
                    const glm::vec3 Ld = trace(ray, max_value, depth+1, false);

                    L += (pf * Ld) / sample.pdf;
                }

                if (samples.size())
                    Li += (survival / samples.size()) * L;
            }
        }

        // Compute scattering term
        const real sigma_a = medium.sigma_a(point);
        const real sigma_s = medium.sigma_s(point);
        return sigma_a * Le + sigma_s * Li;
    }

    glm::vec3 PathTracer::shadeMedium(const glm::vec3 & out, const glm::vec3 & surfaceColor, const Medium & medium, const VolumeRay & vr, unsigned depth) const
    {
        // calculate attenuation for tau(0, s_max)

        glm::vec3 color;

        // Compute reduced surface radiance by transmittance
        const glm::vec3 Tr = glm::vec3(glm::exp(-medium.tau(vr.ray, 0, vr.s)));
        color = Tr * surfaceColor;

        // Sample distance d along the ray for a scattering event
        const unsigned int nSamples = 1;
        for(int i = 0; i < nSamples; i++)
        {
            //const Sampling::Sample1D dSample = Sampling::distance(medium->sigma_t_max());
            const Sampling::Sample1D dSample = Sampling::woodcock(vr.ray, medium);

            // Sample was generated outside medium, ignore..
            if (dSample.t > vr.s)
                continue;

            // Get point at the sampled distance
            const glm::vec3 point = vr.ray.point(dSample.t);

            // Compute in-scattered radiance at the sampled point
            const glm::vec3 Li = scatter(point, out, medium, vr, depth);

            // Transmittance along ray to sampled point tau(0, d)
            const glm::vec3 Tr = glm::vec3(glm::exp(-medium.tau(vr.ray, 0, dSample.t)));

            // Compute total radiance transfered along ray from sampled point
            color += (real(1) / nSamples) * (Tr * Li) / dSample.pdf;
        }

        return color;
    }

}