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

#ifndef PHOTONMAPPER_H
#define PHOTONMAPPER_H

#include "Renderer.h"
#include "PhotonMap.h"

namespace Raytracer2
{
    class PhotonMapper : public Renderer
    {
    public:
        PhotonMapper(const OptionsPtr & opt, const ScenePtr & scene);
        virtual ~PhotonMapper();

        void initialize() override;

    private:
        glm::vec3 radiance(const glm::ray & ray) const override;

        struct Path
        {
            int bounces;
            int scatters;
            bool specular;
        };

        void emitPhotons(int numPhotons);
        void emitPhotons(int numPhotons, std::shared_ptr<Light> const & light);

        void tracePhoton(const glm::ray & ray, const glm::vec3 & power, real maxd, Path & path);

        // Traces ray until first diffuse surface and returns outgoing radiance
        glm::vec3 traceRay(const glm::vec3 & out, const SurfacePoint & sp) const;

        glm::vec3 shade(glm::vec3 out, const SurfacePoint & sp) const;

        glm::vec3 raymarch(const glm::vec3 & out, const Medium & medium, const VolumeRay & vr, real t, const glm::vec3 & surfaceColor) const;

        glm::vec3 shadeMedium(const glm::vec3 & out, const glm::vec3 & surfaceColor, const Medium & medium, const VolumeRay & vr) const;

        glm::vec3 finalGather(const glm::vec3 & out, const SurfacePoint & sp) const;

        // estimates the out-going radiance from the photon map
        glm::vec3 estimateRadiance(const glm::vec3 & out, const SurfacePoint & sp, const PhotonMap & photonMap) const;

        glm::vec3 estimateRadiance(const glm::vec3 & out, const glm::vec3 & point, const VolumeRay & vr, const PhotonMap & photonMap) const;

        PhotonMap mapCaustics;
        PhotonMap mapIndirect;
        PhotonMap mapVolume;
    };
}

#endif // PHOTONMAPPER_H