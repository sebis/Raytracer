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

#ifndef MEDIUM_H
#define MEDIUM_H

#include "Constants.h"
#include "Geometry.h"
#include "GLMExtensions.h"
#include "PhaseFunction.h"
#include "VolumeRay.h"

#include <memory>

namespace Raytracer2
{
    // TODO: add transform
    class Medium
    {
    public:
        Medium(const Geometry & geometry, const PhaseFunction & phaseFunction);

        bool intersect(const glm::ray & ray, real & t0, real & t1) const;
        void hitVolumePoint(VolumeRay & vr) const;

        // Returns absorption coefficient at point x
        virtual real sigma_a(const glm::vec3 & point) const = 0;
        // Returns scattering coefficient at point x
        virtual real sigma_s(const glm::vec3 & point) const = 0;
        // Returns extinction coefficient at point x (default is sigma_a + sigma_s)
        virtual real sigma_t(const glm::vec3 & point) const;
        // Returns the max sigma_t in the whole medium
        virtual real sigma_t_max() const = 0;

        virtual glm::vec3 emit(const glm::vec3 & point) const = 0;

        // Returns optical thickness along ray from t0 to t1
        virtual real tau(const glm::ray & ray, real t0, real t1) const = 0;

        real absorption(const glm::vec3 & point) const;

    private:
        const Geometry & m_geometry;
        const PhaseFunction & m_phaseFunction;
    };
}

#endif // MEDIUM_H