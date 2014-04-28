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

#ifndef GLMEXTENSIONS_H
#define GLMEXTENSIONS_H

#include <glm/glm.hpp>

#include <glm/gtc/constants.hpp>
#include <glm/gtc/epsilon.hpp>
#include <glm/gtc/quaternion.hpp>

#include <glm/gtx/intersect.hpp>

#include <glm/core/type_float.hpp>
#include <glm/core/setup.hpp>

namespace glm
{
    namespace detail
    {
        template <typename T>
        struct trect
        {
            typedef tvec2<T> value_type;
            typedef std::size_t size_type;

            value_type min;
            value_type max;

            GLM_FUNC_DECL trect()
            {
                // TODO: use something else to denote 'no value'
                min = value_type(-1);
                max = value_type(-1);
            }

		    GLM_FUNC_DECL trect(trect<T> const & r)
            {
                min = r.min;
                max = r.max;
            }

            GLM_FUNC_DECL explicit trect(
                value_type const & _min,
                value_type const & _max)
                : min(_min), max(_max)
            {
            }

            GLM_FUNC_DECL explicit trect(
                T minX, T minY, T maxX, T maxY)
                : min(minX, minY), max(maxX, maxY)
            {
            }

            GLM_FUNC_QUALIFIER GLM_CONSTEXPR size_type width() const
            {
                return max[0] - min[0];
            }

            GLM_FUNC_QUALIFIER GLM_CONSTEXPR size_type height() const
            {
                return max[1] - min[1];
            }

            GLM_FUNC_QUALIFIER GLM_CONSTEXPR bool empty() const
            {
                return !height() || !width();
            }

            GLM_FUNC_QUALIFIER void expand(const tvec2<T> & v)
            {
                if(min[0] < 0) min[0] = v[0];
                else min[0] = glm::min(min[0], v[0]);
                if(min[1] < 0) min[1] = v[1];
                else min[1] = glm::min(min[1], v[1]);
                if(max[0] < 0) max[0] = v[0];
                else max[0] = glm::max(max[0], v[0]);
                if(max[1] < 0) max[1] = v[1];
                else max[1] = glm::max(max[1], v[1]);
            }
        };

        template <typename T>
        struct taabb
        {
            typedef tvec3<T> value_type;
            typedef std::size_t size_type;

            value_type min, max;

            GLM_FUNC_DECL taabb()
                : min(std::numeric_limits<T>::infinity())
                , max(-std::numeric_limits<T>::infinity())
            {}

            GLM_FUNC_DECL taabb(const taabb<T> & other)
                : min(other.min)
                , max(other.max)
            {}

            GLM_FUNC_DECL taabb(const value_type & p)
                : min(p), max(p)
            {}

            GLM_FUNC_DECL taabb(const value_type & a, const value_type & b)
            {
                min = glm::min(a, b);
                max = glm::max(a, b);
            }

            GLM_FUNC_DECL taabb(const value_type & c, const T & r)
            {
                min = c - value_type(r);
                max = c + value_type(r);
            }

            GLM_FUNC_QUALIFIER GLM_CONSTEXPR value_type center() const
            {
                return 0.5 * (min + max);
            }

            GLM_FUNC_QUALIFIER GLM_CONSTEXPR value_type size() const
            {
                return max - min;
            }

            GLM_FUNC_QUALIFIER GLM_CONSTEXPR int largest_axis() const
            {
                const value_type size = max - min;
                if(size.x >= size.y) // x or z
                {
                    if(size.x >= size.z)
                        return 0; // x-axis
                    return 2; // z-axis
                }
                else // y or z
                {
                    if(size.y >= size.z)
                        return 1; // y-axis
                    return 2; // z-axis
                }
            }

            GLM_FUNC_QUALIFIER void expand(const value_type & v)
            {
                min = glm::min(min, v);
                max = glm::max(max, v);
            }

            GLM_FUNC_QUALIFIER void expand(const taabb<T> & a)
            {
                min = glm::min(min, a.min);
                max = glm::max(max, a.max);
            }

            GLM_FUNC_QUALIFIER GLM_CONSTEXPR bool empty(int dim) const
            {
                return max[dim] <= min[dim];
            }

            GLM_FUNC_QUALIFIER GLM_CONSTEXPR bool empty() const
            {
                return empty(0) && empty(1) && empty(2);
            }

            static taabb<T> combine(const taabb<T> & a, const taabb<T> & b)
            {
                taabb<T> c(a);
                c.expand(b);
                return c;
            }
        };

        template <typename T>
        struct tray
        {
            typedef tvec3<T> value_type;
            typedef std::size_t size_type;

            value_type origin;
            value_type dir;

            GLM_FUNC_DECL tray()
            {
            }

		    GLM_FUNC_DECL tray(tray<T> const & r)
            {
                origin = r.origin;
                dir = r.dir;
            }

            GLM_FUNC_DECL explicit tray(
                value_type const & _origin,
                value_type const & _dir,
                T const & _fudge = T(0))
                : origin(_origin + _fudge * _dir), dir(_dir)
            {
            }

            GLM_FUNC_QUALIFIER GLM_CONSTEXPR value_type point(T t) const
            {
                return origin + t * dir;
            }
        };
    }

    typedef detail::taabb<float_t> aabb;

    typedef detail::trect<int_t> irect;
    typedef detail::trect<float_t> frect;

    typedef detail::tray<float_t> ray;

    // cbrt
	template <typename genType>
	GLM_FUNC_QUALIFIER genType cbrt
	(
		genType const & x
	)
	{
		GLM_STATIC_ASSERT(detail::type<genType>::is_float, "'cbrt' only accept floating-point input");

		return genType(::std::pow(x, 1.f/3.f)); // VS2012 does not have std::cbrt
	}

    /// Return quaternion to rotate one unit vector to another.
    template <typename T>
    GLM_FUNC_QUALIFIER detail::tquat<T> rotate
        (
        detail::tvec3<T> const & u,
        detail::tvec3<T> const & v
        )
    {
        const T uu = dot(u, u);
        const T vv = dot(v, v);
        const T uv = dot(u, v);
        const T w = sqrt(uu * vv) + uv;

        const detail::tquat<T> q(w, cross(u, v));
        return normalize(q);
    }
    /// Return matrix to rotate one unit vector to another.
    // TODO: this is broken
    /*template <typename T>
    GLM_FUNC_QUALIFIER detail::tmat3x3<T> rotate
        (
        detail::tvec3<T> const & u,
        detail::tvec3<T> const & v
        )
    {
        T e = epsilon<T>();
        detail::tvec3<T> a = cross(u, v);
        if(length(a) < e)
        {
            // Check if no rotation or 180 degree rotation.
            // In case of 180 degrees find some axis to rotate.
            if (dot(u, v) > 0)
                return detail::tmat3x3<T>();
            else if (abs(u.x) > e || abs(u.y) > e)
                a = normalize(detail::tvec3<T>(-u.y, u.x, 0));
            else 
                a = detail::tvec3<T>(u.z, 0, 0);
        }
        detail::tvec3<T> uxa = cross(u, a);
        detail::tvec3<T> vxa = cross(v, a);
        return detail::tmat3x3<T>(v, a, vxa) * transpose(detail::tmat3x3<T>(u, a, uxa));
    }*/

    /// Generate cosine-weighted random sample from  unit hemisphere z > 0.
    template <typename T>
    GLM_FUNC_QUALIFIER detail::tvec3<T> hemiRand
        (
        )
    {
        T x, y, z;
        do
        {
            x = T(2) * (rand() / (T)RAND_MAX) - T(1);
            y = T(2) * (rand() / (T)RAND_MAX) - T(1);
        } while (x*x + y*y > T(1));
        z = sqrt(T(1) - x*x - y*y);
        assert(!isnan(z));
        return detail::tvec3<T>(x, y, z);
    }

    template <typename T, typename U>
	GLM_FUNC_QUALIFIER detail::tvec3<T> unProject
	(
		detail::tvec3<T> const & win, 
		detail::tmat4x4<T> const & projModelInv,
		detail::tvec4<U> const & viewport
	)
	{
		const detail::tmat4x4<T> & inverse = projModelInv;

		detail::tvec4<T> tmp = detail::tvec4<T>(win, T(1));
		tmp.x = (tmp.x - T(viewport[0])) / T(viewport[2]);
		tmp.y = (tmp.y - T(viewport[1])) / T(viewport[3]);
		tmp = tmp * T(2) - T(1);

		detail::tvec4<T> obj = inverse * tmp;
		obj /= obj.w;

		return detail::tvec3<T>(obj);
	}

    template <typename genType>
    GLM_FUNC_QUALIFIER genType fuzzyDirac
        (
            genType x,
            genType y
        )
    {
        if(glm::epsilonEqual(x, y, 0.01))
            return genType(1);
        return genType(0);
    }

    // intersects a ray with a quadrilateral (given by counter-clockwise ordering)
    // p.w > 0 if hit (1,2,4) triangle and p.w < 0 if hit (4,2,3) triangle
    template <typename genType>
	GLM_FUNC_QUALIFIER bool intersectRayQuadrilateral
	(
		genType const & orig, genType const & dir,
		genType const & v1, genType const & v2, genType const & v3, genType const & v4,
        detail::tvec4<typename genType::value_type> & baryPosition
	)
    {
        genType tmpBaryPosition;
        bool hit = intersectRayTriangle(orig, dir, v1, v2, v4, tmpBaryPosition);

        genType tmpBaryPosition2;
        bool hit2 = intersectRayTriangle(orig, dir, v4, v2, v3, tmpBaryPosition2);

        if(hit && (!hit2 || tmpBaryPosition.z < tmpBaryPosition2.z))
        {
            baryPosition = detail::tvec4<typename genType::value_type>(tmpBaryPosition, -1);
            return true;
        }
        else if(hit2 && (!hit || tmpBaryPosition2.z < tmpBaryPosition.z))
        {
            baryPosition = detail::tvec4<typename genType::value_type>(tmpBaryPosition2, 1);
            return true;
        }
        return false;
    }

    // Implementation from Real-Time Collision Detection by Chrisopher Ericson
    // Section 5.3, pp. 180-181
    // Note: There is an error in the book. The line "if (t2 > tmax) tmax = t2;"
    // should be "if (t2 < tmax) tmax = t2;". Here the comparisons are replaced by
    // min() and max().
    template <typename genType>
	GLM_FUNC_QUALIFIER
    bool intersectRayAABB(const genType & p, const genType & invd,
        const glm::detail::taabb<typename genType::value_type> & a,
        typename genType::value_type & tmin = typename genType::value_type(0),
        typename genType::value_type tmax = std::numeric_limits<typename genType::value_type>::max())
    {
        typename genType::value_type Epsilon = std::numeric_limits<typename genType::value_type>::epsilon();
        tmin = typename genType::value_type(0);

        for(int i = 0; i < 3; i++)
        {
            if(abs(invd[i]) < Epsilon)
            {
                // Ray is parallel to slab, no hit if origin is not within slab
                if(p[i] < a.min[i] || p[i] > a.max[i])
                    return false;
            }
            else
            {
                // Compute intersection t value of ray with near and far plane of slab
                typename genType::value_type t1 = (a.min[i] - p[i]) * invd[i];
                typename genType::value_type t2 = (a.max[i] - p[i]) * invd[i];
                // Make t1 be intersection with near plane, t2 with far plane
                if(t1 > t2)
                    std::swap(t1, t2);
                // Compute the intersection of slab intersection intervals
                tmin = max(tmin, t1);
                tmax = min(tmax, t2);
                // Exit with no collision as soon as slab intersection becomes empty
                if (tmin > tmax)
                    return false;
            }
        }
        // Ray intersects all 3 slabs. Return point (q) and intersection t value (tmin)
        return true;
    }
}

#endif // GLMEXTENSIONS_H