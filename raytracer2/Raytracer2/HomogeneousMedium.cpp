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

#include "HomogeneousMedium.h"

namespace Raytracer2
{
    HomogeneousMedium::HomogeneousMedium(
        const Geometry & geometry, const PhaseFunction & phaseFunction,
        real sigma_a, real sigma_s, const glm::vec3 & emission)
        : Medium(geometry, phaseFunction)
        , m_sigma_a(sigma_a)
        , m_sigma_s(sigma_s)
        , m_emission(emission)
    {
    }

    real HomogeneousMedium::sigma_a(const glm::vec3 & point) const
    {
        return m_sigma_a;
    }

    real HomogeneousMedium::sigma_s(const glm::vec3 & point) const
    {
        return m_sigma_s;
    }

    real HomogeneousMedium::sigma_t(const glm::vec3 & point) const
    {
        return m_sigma_a + m_sigma_s;
    }

    real HomogeneousMedium::sigma_t_max() const
    {
        return m_sigma_a + m_sigma_s;
    }

    glm::vec3 HomogeneousMedium::emit(const glm::vec3 & point) const
    {
        return m_emission;
    }

    real HomogeneousMedium::tau(const glm::ray & ray, real t0, real t1) const
    {
        // Value is constant, so integral is simply distance times constant
        const real d = t1 - t0;
        return d * sigma_t_max();
    }
}