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

#include "Renderer.h"

#include "Sampling.h"
#include "SurfacePoint.h"

namespace Raytracer2
{
    Renderer::Renderer(const OptionsPtr & opt, const ScenePtr & scene)
        : m_options(opt)
        , m_scene(scene)
    {}

    glm::vec3 Renderer::tracePixel(const glm::mat4 & projModelInv, const glm::vec4 & viewport, unsigned x, unsigned y) const
    {
        const unsigned raysPerPixel = opt().s_raysPerPixel;
        double n = glm::sqrt<double>(raysPerPixel);

        double dx = 1/n;
        double dy = 1/n;

        glm::vec3 color;

        /*for (double sx = 0.0; sx < 1.0; sx += dx)
        {
            for (double sy = 0.0; sy < 1.0; sy += dy)
            {
                double px = x + sx;
                double py = y + sy;

                glm::vec3 near = glm::unProject(glm::vec3(px, py, 0), projModelInv, viewport);
                glm::vec3 far = glm::unProject(glm::vec3(px, py, 1), projModelInv, viewport);

                glm::vec3 dir = glm::normalize(far - near);

                color += m_d->trace(near, dir, std::numeric_limits<float>::max(), depth);
            }
        }*/

        for(int i = 0; i < n; i++)
        {
            for(int j = 0; j < n; j++)
            {
                double px = x + Sampling::uniform();
                double py = y + Sampling::uniform();;

                const glm::vec3 near = glm::unProject(glm::vec3(px, py, 0), projModelInv, viewport);
                const glm::vec3 far = glm::unProject(glm::vec3(px, py, 1), projModelInv, viewport);

                const glm::ray ray(near, glm::normalize(far - near));

                color += radiance(ray);
            }
        }

        return color * (1.0/raysPerPixel);
    }

    glm::vec3 Renderer::shadeWorld(const glm::ray &) const
    {
        // TODO: Implement environment maps
        return glm::vec3(0.f, 0.f, 0.f);
    }

    const Object* Renderer::hitSurface(const glm::ray & ray,
                            real & maxd,
                            SurfacePoint & sp) const
    {
        const Object* obj = hitObject(ray, maxd, sp);
        if(obj)
            obj->hitSurfacePoint(sp);
        return obj;
    }

    const Object* Renderer::hitObject(const glm::ray & ray, real & maxd, SurfacePoint & sp) const
    {
        Object* obj = nullptr;

        SurfacePoint sp2;
        for(auto const & o : objects())
        {
            real d;
            if(o->hit(ray, sp2, d) && d < maxd)
            {
                obj = o.get();
                sp = sp2;
                maxd = d;
            }
        }

        return obj;
    }

    const Medium* Renderer::hitMedium(const glm::ray & ray, VolumeRay & vr, real maxd) const
    {
        Medium *med = nullptr;
        real t0, t1;
        real d = maxd;

        for(auto const & m : media())
        {
            const bool hit = m->intersect(ray, t0, t1);
            if(hit && t0 < d)
            {
                med = m.get();
                vr.t0 = t0;
                vr.t1 = glm::min(maxd, t1);

                vr.s = vr.t1 - vr.t0;
                vr.ray = glm::ray(ray.point(vr.t0), ray.dir);

                d = t0;
            }
        }

        if(med)
            med->hitVolumePoint(vr);

        return med;
    }

    bool Renderer::occludes(const glm::ray & ray, glm::float_t maxd) const
    {
        SurfacePoint sp;
        for(auto const & o : objects())
        {
            real d;
            if(o->hit(ray, sp, d) && d < maxd)
                return true;
        }
        return false;
    }

    glm::vec3 Renderer::directLighting(const glm::vec3 & out, const SurfacePoint & sp) const
    {
        glm::vec3 radiance;

        for(auto const & light : lights())
        {
            LightSamples samples;
            light->illuminate(sp.point, opt().s_lightSamples, samples);

            for (auto const & sample : samples)
            {
                const glm::ray ray(sp.point, sample.direction, opt().s_fudge);
                const real d = sample.distance - opt().s_fudge;

                // FIXME: ray might actually start inside object, which results in no
                // intersections
                if(!occludes(ray, d - opt().s_fudge))
                {
                    // Compute transmittance through medium
                    VolumeRay vr;
                    const Medium* medium = hitMedium(ray, vr, d);
                    const glm::vec3 Tr = (!medium) ? glm::vec3(1) : glm::vec3(glm::exp(-medium->tau(vr.ray, 0, vr.s)));

                    const glm::vec3 brdf = sp.eval(sample.direction, out);
                    const real cos_theta = glm::max(real(0), sp.cos_theta(sample.direction));

                    radiance += Tr * brdf * sample.radiance * (cos_theta / sample.pdf);
                }
            }

            if (samples.size())
                radiance *= (real(1) / samples.size());
        }

        return radiance;
    }

    glm::vec3 Renderer::singleScatter(const glm::vec3 & point, const glm::vec3 & out, const Medium & medium, const VolumeRay & vr) const
    {
        glm::vec3 radiance;

        for(auto const & light : lights())
        {
            LightSamples samples;
            light->illuminate(point, opt().s_lightSamples, samples);

            for (auto const & sample : samples)
            {
                const glm::ray ray(point, sample.direction, opt().s_fudge);
                const real d = sample.distance - opt().s_fudge;

                if(!occludes(ray, d - opt().s_fudge))
                {
                    // Intersect shadow ray with medium
                    real t0, t1;
                    const bool hit = medium.intersect(ray, t0, t1);
                    assert(hit); // should always hit

                    // Compute attenuation and phase function value
                    const glm::vec3 Tr = glm::vec3(glm::exp(-medium.tau(ray, t0, glm::min(t1, d))));
                    const glm::vec3 pf = vr.eval(point, sample.direction, out);

                    radiance += Tr * pf * sample.radiance * (1 / sample.pdf);
                }
            }

            if (samples.size())
                radiance *= (real(1) / samples.size());
        }

        return radiance;
    }

    Scene::Objects & Renderer::objects()
    {
        return m_scene->objects();
    }

    const Scene::Objects & Renderer::objects() const
    {
        return m_scene->objects();
    }

    Scene::Lights & Renderer::lights()
    {
        return m_scene->lights();
    }

    const Scene::Lights & Renderer::lights() const
    {
        return m_scene->lights();
    }

    Scene::Media & Renderer::media()
    {
        return m_scene->media();
    }

    const Scene::Media & Renderer::media() const
    {
        return m_scene->media();
    }
}
