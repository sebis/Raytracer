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

#include "MeshGeometry.h"

#include "Constants.h"
#include "SurfacePoint.h"

#include <Pane/Trace.h>

#include <glm/gtx/intersect.hpp>

namespace Raytracer2
{
    MeshGeometry::MeshGeometry(const MeshPtr & mesh)
        : m_mesh(mesh)
    {
        m_bvh = std::make_shared<BVH>(mesh);
        m_bvh->build();
    }

    bool MeshGeometry::intersect(const glm::ray & ray, real & t0, real & t1) const
    {
        // TODO: not implemented
        assert(false);

        return false;
    }

    bool MeshGeometry::hit(const glm::ray & ray, SurfacePoint & sp, real & t) const
    {
        t = max_value;

        if(m_bvh)
        {
            // Use BVH
            Intersection isec;
            if(m_bvh->intersect(ray, isec))
            {
                sp.point = isec.point;
                sp.normal = isec.normal;
                t = isec.t;
            }
        }
        else
        {
            // Enumerate all faces
            for(int i = 0; i < m_mesh->nFaces; i++)
            {
                const Face & face = m_mesh->faces[i];

                Intersection isec;
                if (face.intersect(ray, t, isec))
                {
                    sp.point = isec.point;
                    sp.normal = isec.normal;
                    t = isec.t;
                }
            }
        }

        return t < max_value;
    }

    void MeshGeometry::hitSurfacePoint(SurfacePoint & sp) const
    {
        // Do nothing
    }
}
