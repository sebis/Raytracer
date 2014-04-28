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

#ifndef SHADER_H
#define SHADER_H

#include <glm/glm.hpp>

#include <vector>

namespace Raytracer2
{   
    struct SurfacePoint;

    struct BSDFSample
    {
        BSDFSample()
            : discard(true)
        {}

        BSDFSample(const glm::vec3 & _direction, const glm::vec3 & _value, glm::float_t _pdf)
            : direction(_direction)
            , value(_value)
            , pdf(_pdf)
            , discard(false)
        {}

        glm::vec3 direction;
        glm::vec3 value;
        glm::float_t pdf;
        bool discard;
    };
    typedef std::vector<BSDFSample> BSDFSamples;

    class Shader
    {
    public:
        enum Type
        {
            Diffuse,
            Emissive,
            Mirror,
        };

        virtual glm::vec3 emit(const SurfacePoint & /*sp*/, const glm::vec3 & /*out*/) const
        {
            return glm::vec3(0);
        }

        virtual glm::vec3 eval(const SurfacePoint & point,
                               const glm::vec3 & inDir,
                               const glm::vec3 & outDir) const = 0;

        virtual void generateDir(const SurfacePoint & point,
                                 const glm::vec3 & outDir,
                                 unsigned numSamples,
                                 BSDFSamples & samples) const = 0;

        virtual glm::float_t absorption() const { return 0.5f; }
        virtual glm::vec3 reflectivity() const { return glm::vec3(0.5f); }

        Type type;
    };
}

#endif // SHADER_H