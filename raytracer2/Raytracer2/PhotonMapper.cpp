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

#include "PhotonMapper.h"

#include "SurfacePoint.h"
#include "Utils.h"

#include <Pane/Trace.h>

#include <ctime>

namespace
{
    clock_t s_timer;
    void start(clock_t & c)
    {
        c = clock();
    }

    double since(const clock_t & c)
    {
        return double(clock()-c)/CLOCKS_PER_SEC;
    }
}

namespace Raytracer2
{
    PhotonMapper::PhotonMapper(const OptionsPtr & opt, const ScenePtr & scene)
        : Renderer(opt, scene)
    {}

    PhotonMapper::~PhotonMapper()
    {}

    void PhotonMapper::initialize()
    {
        mapCaustics.reserve(opt().s_photons);
        mapIndirect.reserve(opt().s_photons);
        mapVolume.reserve(opt().s_photons);

        ::start(s_timer);

        // TODO: should parallelize photon emission
        emitPhotons(opt().s_photons);

        Pane::info("Photon tracing took %.2fs", ::since(s_timer));

        ::start(s_timer);

        mapCaustics.balance();
        mapIndirect.balance();
        mapVolume.balance();

        Pane::info("Photon map construction took %.2fs", ::since(s_timer));

        if(opt().s_beamEstimate)
        {
            ::start(s_timer);

            mapVolume.constructBBH(opt().s_queryPhotons);

            Pane::info("Photon bounding box hierarchy took %.2fs", ::since(s_timer));
        }

        Pane::info("Photon maps constructed: caustics (%d), global (%d), volume (%d)", mapCaustics.size(), mapIndirect.size(), mapVolume.size());
    }

    glm::vec3 PhotonMapper::radiance(const glm::ray & ray) const
    {
        real maxd = max_value;

        glm::vec3 color;

        SurfacePoint sp;
        if(hitSurface(ray, maxd, sp))
            color = traceRay(-ray.dir, sp);
        else
            color = shadeWorld(ray);

        // Handle medium
        VolumeRay vr;
        const Medium* medium = hitMedium(ray, vr, maxd);
        if (medium)
            color = shadeMedium(-ray.dir, color, *medium, vr);

        return color;
    }

    void PhotonMapper::emitPhotons(int numPhotons)
    {
        // TODO: photons should also be able to be emitted from participating media

        // select light
        // TODO: select light by probability

        const auto & light = lights().at(0);

        emitPhotons(numPhotons, light);
    }

    void PhotonMapper::emitPhotons(int numPhotons, std::shared_ptr<Light> const & light)
    {
        LightSamples samples;
        light->generateDir(numPhotons, samples);

        for (auto const & sample : samples)
        {
            // TODO: what should power be?
            const glm::vec3 power = sample.radiance / sample.pdf * (real(1) / samples.size());
            //const glm::vec3 power = sample.radiance;

            // trace photon
            Path path = { 0, 0, true };
            tracePhoton(glm::ray(sample.point, sample.direction, opt().s_fudge), power, max_value, path);
        }
    }

    void PhotonMapper::tracePhoton(const glm::ray & ray, const glm::vec3 & power, real maxd, Path & path)
    {
        // Begin by checking intersection with a surface
        SurfacePoint sp;
        const Object* obj = hitSurface(ray, maxd, sp);

        // Then check intersection with media (maxd is now the closest intersection with a surface)
        VolumeRay vr;
        const Medium* medium = hitMedium(ray, vr, maxd);
        if (medium)
        {
            // Generate a distance sample for interaction point
            const Sampling::Sample1D dSample = Sampling::distance(medium->sigma_t_max());

            // Check if interaction point is inside medium and before first surface
            if(dSample.t <= vr.s)
            {
                const glm::vec3 point = vr.ray.point(dSample.t);

                // Store photon to volume photon map (only if scattered atleast once)
                if(!opt().s_singleScattering || path.scatters > 0)
                {
                    Photon *p = new Photon;
                    p->position = point;
                    p->power = power;
                    p->incident = -ray.dir;
                    mapVolume.store(p);
                }

                // Photon interacts with medium -> use russian roulette to determine if it is
                // scattered or absorbed
                const real albedo = medium->sigma_s(point) / medium->sigma_t(point);

                if(Sampling::uniform() <= albedo) // Scatter
                {
                    path.scatters++;

                    // Trace new photons according to the phase function
                    PFSamples samples;
                    vr.samplePF(-ray.dir, 1, samples);

                    for (auto const & sample : samples)
                    {
                        tracePhoton(glm::ray(point, sample.direction, opt().s_fudge), power, max_value, path);
                    }
                }
                else // Absorb
                {
                    // Do nothing, the photon is absorbed and vanishes
                }

                // If the photon interacts with the medium, we should not continue tracing the
                // current direction, so return from this function
                return;
            }
        }

        // Photon did not interact with medium, or interaction point is outside the medium
        // so continue to trace the photon to the surface
        if(obj)
        {
            // store photon only if it hits a diffuse surface
            if(sp.shader->type == Shader::Diffuse)
            {
                //Pane::info("Generated point: %f %f %f", ray.origin.x, ray.origin.y, ray.origin.z);

                Photon *p = new Photon;
                p->position = sp.point;
                p->power = power;
                p->incident = -ray.dir;

                //Pane::info("Storing photon: %f %f %f", p->position.x, p->position.y, p->position.z);

                // Select photon map. Use direct map if photon has at least one bounce and has
                // only hit specular surfaces, otherwise use indirect map
                const bool indirect = !path.bounces || !path.specular;
                PhotonMap & photonMap = indirect ? mapIndirect : mapCaustics;
                photonMap.store(p);

                // specular remains true only if all photons are reflected specularily
                path.specular = false;
            }

            // reflect photon
            path.bounces++;

            /*real survival;
            if (russianRoulette(sp.shader->absorption(), survival))*/
            glm::vec3 survival;
            if (russianRoulette(sp.shader->reflectivity(), power, survival))
            {
                const unsigned numSamples = 1;

                BSDFSamples samples;
                sp.generateDir(-ray.dir, numSamples, samples);

                for (auto const & sample : samples)
                {
                    tracePhoton(glm::ray(sp.point, sample.direction, opt().s_fudge), power, max_value, path);
                }
            }
        }
    }

    // Traces ray until first diffuse surface and returns outgoing radiance
    glm::vec3 PhotonMapper::traceRay(const glm::vec3 & out, const SurfacePoint & sp) const
    {
        glm::vec3 color;

        // recursive ray tracing until we hit non-specular surface
        if(sp.shader->type == Shader::Mirror)
        {
            BSDFSamples samples;
            sp.generateDir(out, 1, samples);

            for (const auto & sample : samples)
            {
                const glm::ray ray(sp.point, sample.direction, opt().s_fudge);

                const glm::vec3 Lr = radiance(ray);
                const glm::vec3 brdf = sample.value;
                const real cos_theta = sp.cos_theta(sample.direction);

                color += Lr * brdf * (cos_theta / (sample.pdf * samples.size()));
            }
        }
        else
        {
            // if we hit non-specular surface, shade the current surface point
            color += shade(out, sp);
        }

        return color;
    }

    glm::vec3 PhotonMapper::shade(glm::vec3 out, const SurfacePoint & sp) const
    {
        glm::vec3 color;

        // emitted radiance
        color += sp.emit(out);

        if (!opt().s_finalGather)
        {
            // estimate radiance straight from photon maps
            color += 
                estimateRadiance(out, sp, mapCaustics) +
                estimateRadiance(out, sp, mapIndirect);
        }
        else // perform final gather step
        {
            // sample lights
            color += directLighting(out, sp);
            //color += estimateRadiance(out, sp, mapIndirect);

            // sample caustics from caustics map
            color += estimateRadiance(out, sp, mapCaustics);

            // Generate final gather rays
            color += finalGather(out, sp);
        }

        return color;
    }

    // Performs recursive adaptive raymarching
    glm::vec3 PhotonMapper::raymarch(const glm::vec3 & out, const Medium & medium, const VolumeRay & vr, real t, const glm::vec3 & surfaceColor) const
    {
        // out-going radiance in the direction of out
        glm::vec3 radiance;

        // Get starting point for this segment
        const glm::vec3 & point = vr.ray.point(t);

        // generate next point along ray
        const Sampling::Sample1D dSample = Sampling::distance(medium.sigma_t(point));
        real tn = t + dSample.t;
        //real tn = t + real(0.1);

        // Incoming radiance to the end of this segment
        // If this is not the last segment, gather radiance recursively, otherwise use surface radiance
        const glm::vec3 Ln = (tn <= vr.s) ? 
            raymarch(out, medium, vr, tn, surfaceColor) :
            surfaceColor;
        
        tn = glm::min(tn, vr.s); // clamp to medium boundary if sample lands outside medium

        // Length of this segment
        const real dt = tn - t;

        // Test that the segment is not degenerate
        if (dt < epsilon)
            radiance += Ln; // if so, pass the incoming radiance as-is
        else
        {
            // Compute single scatter radiance from light sources for this segment
            if(opt().s_singleScattering)
            {
                const glm::vec3 Ls = singleScatter(point, out, medium, vr);
                radiance += dt * medium.sigma_s(point) * Ls;
            }

            // Compute multiple scatter radiance from volume photon map for this segment
            const glm::vec3 Lm = estimateRadiance(out, point, vr, mapVolume);
            radiance += dt * Lm;

            // Transmitted radiance through this segment
            const glm::vec3 Tr = glm::vec3(glm::exp(-medium.sigma_t(point) * dt));
            radiance += Tr * Ln;
        }
        
        return radiance;
    }

    glm::vec3 PhotonMapper::shadeMedium(const glm::vec3 & out, const glm::vec3 & surfaceColor, const Medium & medium, const VolumeRay & vr) const
    {
        if(opt().s_beamEstimate)
        {
            glm::vec3 radiance;

            // starting point in medium
            const glm::vec3 x = vr.ray.origin;

            // Transmitted radiance through this segment
            const glm::vec3 Tr = glm::vec3(glm::exp(-medium.tau(vr.ray, 0, vr.s)));
            radiance += Tr * surfaceColor;

            // Inscattered radiance
            PhotonMap::Photons photons;
            mapVolume.intersectPhotons(vr.ray, vr.s, photons);

            const unsigned N = photons.size();
            assert(N);

            for(int i = 0; i < N; i++)
            {
                const Photon* p = photons[i];
                
                // Project photon i onto ray
                const real t = glm::dot(p->position - x, vr.ray.dir);
                const glm::vec3 xi = vr.ray.point(t);
                
                // Properties at xi for computing radiance estimate
                const glm::vec3 Tr = glm::vec3(glm::exp(-medium.tau(vr.ray, 0, t)));
                const real sigma_s = medium.sigma_s(xi);
                const glm::vec3 pf = vr.eval(p->position, p->incident, out);
                const glm::vec3 alpha = p->power * real(opt().s_photons);

                // Closest distance from photon i to the ray
                const real d = glm::distance(p->position, xi);
                const real r = p->radius;

                // Compute kernel
                real K = 0;
                if(d >= 0 && d <= r)
                {
                    const real x = d/r;

                    // Silverman's biweight kernel
                    const real tmp = (1 - x*x);
                    const real K2 = (3 / pi) * tmp*tmp;

                    K = K2 / (r*r);
                }

                radiance += K * Tr * sigma_s * pf * alpha * (real(1) / N);
            }

            return radiance;
        }
        else
        {
            // Start recursive raymarching from t = 0
            return raymarch(out, medium, vr, 0, surfaceColor);
        }
    }

    glm::vec3 PhotonMapper::finalGather(const glm::vec3 & out, const SurfacePoint & sp) const
    {
        glm::vec3 color;

        BSDFSamples samples;
        sp.generateDir(out, opt().s_finalGatherRays, samples);

        for(auto const & sample : samples)
        {
            const glm::ray ray(sp.point, sample.direction, opt().s_fudge);
            real maxd = max_value;

            SurfacePoint sp2;
            if(hitSurface(ray, maxd, sp2))
            {
                // estimate reflected radiance using photon maps
                const glm::vec3 Lr =
                    estimateRadiance(-ray.dir, sp2, mapCaustics) +
                    estimateRadiance(-ray.dir, sp2, mapIndirect);

                const glm::vec3 brdf = sample.value;
                const glm::float_t cos_theta = sp.cos_theta(sample.direction);

                color += Lr * brdf * (cos_theta / (sample.pdf * samples.size()));
            }
        }

        return color;
    }

    glm::vec3 PhotonMapper::estimateRadiance(const glm::vec3 & out, const SurfacePoint & sp, const PhotonMap & photonMap) const
    {
        if (photonMap.empty())
            return glm::vec3();

        PhotonMap::Photons photons;
        real rr = photonMap.locatePhotons(sp.point, opt().s_queryPhotons, photons, opt().s_photonDistance);

        //glm::vec3 radiance = sp.emit(out);
        glm::vec3 radiance;

        UniformFilter f;
        //GaussianFilter f;

        glm::vec3 reflRadiance;
        for(auto const & p : photons)
        {
            const glm::vec3 fr = sp.eval(p->incident, out);
            const glm::vec3 phi = p->power;

            const glm::vec3 & diff = p->position - sp.point;
            const real dd = glm::dot(diff, diff);

            reflRadiance += fr * phi * f.w();
        }
        radiance += f.norm(rr) * reflRadiance;

        return radiance;
    }

    glm::vec3 PhotonMapper::estimateRadiance(const glm::vec3 & out, const glm::vec3 & point, const VolumeRay & vr, const PhotonMap & photonMap) const
    {
        // TODO: can we combine these two esimation functions?

        if (photonMap.empty())
            return glm::vec3();

        PhotonMap::Photons photons;
        const real rr = photonMap.locatePhotons(point, opt().s_queryPhotons, photons, opt().s_photonDistance);

        // inverse volume of the sphere
        const real norm = real(3) / (real(4) * pi * rr * sqrt(rr));

        //glm::vec3 radiance = sp.emit(out);
        glm::vec3 radiance;

        for(auto const & p : photons)
        {
            const glm::vec3 pf = vr.eval(point, p->incident, out);
            const glm::vec3 phi = p->power;

            radiance += pf * phi * norm;
        }

        return radiance;
    }
}
