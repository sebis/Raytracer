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

#ifndef PHOTON_MAP_H
#define PHOTON_MAP_H

#include "Constants.h"
#include "GLMExtensions.h"

#include <glm/glm.hpp>

#include <vector>

namespace Raytracer2
{
    struct Photon
    {
        glm::vec3 position;
        glm::vec3 power;
        glm::vec3 incident;
        char flag;
        real radius; // for beam estimation
        glm::aabb bbox; // for creating photon BBH
    };

    class PhotonMap
    {
    public:
        typedef std::vector<Photon*> KdTree;
        typedef std::vector<const Photon*> Photons;
    public:
        PhotonMap();
        PhotonMap(int capacity);

        unsigned size() const;
        bool empty() const;
        void reserve(int capacity);

        void store(Photon *p);
        void balance();

        void constructBBH(unsigned n);
        bool intersectPhotons(const glm::ray & ray, real maxd, Photons & photons) const;

        real locatePhotons(const glm::vec3 & point, int numPhotons, Photons & photons, const real d = real(1)) const;
    private:
        KdTree kdTree;
    };

    struct UniformFilter
    {
        // normalization factor (rr is squared radius)
        inline real norm(real rr)
        {
            return real(1) / (pi * rr);
        }

        // weighting kernel
        inline real w()
        {
            return real(1);
        }
    };

    struct ConeFilter
    {
        real k;

        // normalization factor (rr is squared radius)
        inline real norm(real rr)
        {
            assert(k >= 1);
            return real(1) / ((real(1) - real(2)/(real(3)*k)) * pi * rr);
        }

        // weighting kernel
        inline real w(real d, real r)
        {
            return real(1) - (d / (k*r));
        }
    };

    struct GaussianFilter
    {
        // normalization factor (rr is squared radius)
        inline real norm()
        {
            return real(1);
        }

        // weighting kernel
        inline real w(real dd, real rr)
        {
            const real alpha = real(0.918);
            const real beta = real(1.953);

            const real a = real(1) - glm::exp(-beta*(dd/(2*rr)));
            const real b = real(1) - glm::exp(-beta);
            return alpha * (real(1) - a/b);
        }
    };
}

#endif // PHOTON_MAP_H