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

#ifndef OPTIONS_H
#define OPTIONS_H

namespace Raytracer2
{
    struct Options
    {
        enum RendererType
        {
            RendererRayTracer,
            RendererPathTracer,
            RendererPhotonMapper
        };

        Options()
        {
            default();
        }

        static const Options & default()
        {
            static Options opt;

            // General options
            opt.s_renderer = RendererPathTracer;

            opt.s_name = "unnamed";
            opt.s_width = 512;
            opt.s_height = 512;

            opt.s_fudge = 0.005;
            opt.s_raysPerPixel = 128;
            opt.s_lightSamples = 1;

            // Path tracer options
            opt.s_directLighting = true;
            opt.s_singleScattering = true;

            // Photon mapper options
            opt.s_finalGather = true;
            opt.s_finalGatherRays = 4;
            opt.s_photons = 200000;
            opt.s_queryPhotons = 500;
            opt.s_photonDistance = real(100.0);
            opt.s_beamEstimate = false;

            return opt;
        }

        // General options
        RendererType s_renderer;

        std::string s_name;
        unsigned s_width;
        unsigned s_height;

        real s_fudge;
        unsigned s_raysPerPixel;
        unsigned s_lightSamples;

        // Path tracer options
        bool s_directLighting;
        bool s_singleScattering;

        // Photon mapper options
        bool s_finalGather;
        unsigned s_finalGatherRays;
        unsigned s_photons;
        unsigned s_queryPhotons;
        real s_photonDistance;
        bool s_beamEstimate;
    };
    typedef std::shared_ptr<Options> OptionsPtr;
}

#endif // OPTIONS_H