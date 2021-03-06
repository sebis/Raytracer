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

#ifndef LIGHT_H
#define LIGHT_H

#include "Constants.h"

#include <glm/glm.hpp>

#include <memory>
#include <vector>

namespace Raytracer2
{
    struct LightSample
    {
        LightSample()
        {}

        LightSample(const glm::vec3 & _point,
                    const glm::vec3 & _direction,
                    real _distance,
                    real _pdf,
                    const glm::vec3 & _radiance)
            : point(_point)
            , direction(_direction)
            , distance(_distance)
            , pdf(_pdf)
            , radiance(_radiance)
        {}

        glm::vec3 point; // point on light
        glm::vec3 direction; // direction from surface to point
        real distance;
        real pdf;
        glm::vec3 radiance;
    };
    typedef std::vector<LightSample> LightSamples;

    class Light
    {
    public:
        virtual void illuminate(const glm::vec3 & point,
                                unsigned numSamples,
                                LightSamples & samples) const = 0;

        virtual void generateDir(unsigned numSamples,
                                 LightSamples & samples) const = 0;
    };
    typedef std::shared_ptr<Light> LightPtr;
}

#endif // LIGHT_H