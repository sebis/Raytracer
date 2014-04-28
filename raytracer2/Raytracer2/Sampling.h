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

#ifndef SAMPLING_H
#define SAMPLING_H

#include "Constants.h"
#include "Medium.h"

#include <glm/glm.hpp>

#include <functional>

namespace Raytracer2
{
    // TODO: rename to Distribution
    namespace Sampling
    {
        struct Sample
        {
            glm::vec3 dir;
            real pdf;
        };

        struct Sample1D
        {
            real t;
            real pdf;
        };

        typedef std::function<real(
            const glm::vec3 &,
            const glm::vec3 &)> sigma_t_fun;

        Sampling::Sample lambert(const glm::vec3 & normal);
        Sampling::Sample phong(const glm::vec3 & specularDir, glm::float_t n);

        Sampling::Sample hemisphere(const glm::vec3 & normal);
        Sampling::Sample isotropic();
        Sampling::Sample1D distance(real sigma);

        Sampling::Sample1D woodcock(const glm::ray & ray, const Medium & medium);

        real uniform();
        glm::vec2 uniformArea();
    }
}

#endif // SAMPLING_H