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

#include "PhotonMap.h"
#include "Constants.h"
#include "GLMExtensions.h"

#include <Pane/Trace.h>

#include <glm/glm.hpp>

#include <algorithm>
#include <mutex>
#include <queue>

namespace
{
    

/*
 *  This Quickselect routine is based on the algorithm described in
 *  "Numerical recipes in C", Second Edition,
 *  Cambridge University Press, 1992, Section 8.5, ISBN 0-521-43108-5
 *  This code by Nicolas Devillard - 1998. Public domain.
 */


int lbm(int N)
{
    float n = int(glm::log2(float(N)));
    int M = (int)glm::pow(float(2), n); // largest M = 2^n where M <= N
    int R = N - (M - 1);
    int lbm = (M - 2)/2 + glm::min(R, M/2);
    return lbm;
}

typedef Raytracer2::Photon * elem_type;
int quick_select(elem_type arr[], int n, int dim)
{
#define ELEM_SWAP(a,b) { register elem_type t=(a);(a)=(b);(b)=t; }
#define ELEM_GET(i) arr[i]->position[dim]

    int low, high ;
    int median;
    int middle, ll, hh;

    low = 0 ; high = n-1 ; median = lbm(n);
    for (;;)
    {
        if (high <= low) /* One element only */
            return median ;

        if (high == low + 1) /* Two elements only */
        {
            if (ELEM_GET(low) > ELEM_GET(high))
                ELEM_SWAP(arr[low], arr[high]) ;
            return median ;
        }

        /* Find median of low, middle and high items; swap into position low */
        middle = (low + high) / 2;
        if (ELEM_GET(middle) > ELEM_GET(high))
            ELEM_SWAP(arr[middle], arr[high]) ;
        if (ELEM_GET(low) > ELEM_GET(high))
            ELEM_SWAP(arr[low], arr[high]) ;
        if (ELEM_GET(middle) > ELEM_GET(low))
            ELEM_SWAP(arr[middle], arr[low]) ;

        /* Swap low item (now in position middle) into position (low+1) */
        ELEM_SWAP(arr[middle], arr[low+1]) ;

        /* Nibble from each end towards middle, swapping items when stuck */
        ll = low + 1;
        hh = high;
        for (;;) {
            do ll++; while (ELEM_GET(low) > ELEM_GET(ll)) ;
            do hh--; while (ELEM_GET(hh)  > ELEM_GET(low)) ;

            if (hh < ll)
            break;

            ELEM_SWAP(arr[ll], arr[hh]) ;
        }

        /* Swap middle item (in position low) back into correct position */
        ELEM_SWAP(arr[low], arr[hh]) ;

        /* Re-set active partition */
        if (hh <= median)
            low = ll;
        if (hh >= median)
            high = hh - 1;
    }
#undef ELEM_GET
#undef ELEM_SWAP
}

    int median_sort(elem_type a[], int n, int dim)
    {
        auto cmp = [dim] (const elem_type & a, const elem_type & b)
            {
                return a->position[dim] < b->position[dim];
            };
        std::sort(a, a + n, cmp);
        return n / 2;
    }

    #define median(a,n,dim) ::quick_select(a,n,dim)
    //#define median(a,n,dim) ::median_sort(a,n,dim)

    std::mutex s_mutex;
}

namespace Raytracer2
{
    namespace
    {
        void buildBalancedKdTree(Photon **input, Photon **output, unsigned index, int start, int end)
        {
            // Find cube surrounding the input
            glm::vec3 min = glm::vec3(max_value), max = glm::vec3(min_value);
            for(int i = start; i <= end; i++)
            {
                min = glm::min(min, input[i]->position);
                max = glm::max(max, input[i]->position);
            }

            // Select dim in which the cube is the largest
            const glm::vec3 & size = max - min;
            const char dim = size[0] >= size[1] ? (size[0] >= size[2] ? 0 : 2) : (size[1] >= size[2] ? 1 : 2);

            // Find median in the selected dimension
            const int n = end - start + 1;
            const int m = start + median(input + start, n, dim);

            // Save result in output array
            Photon *p = input[m];
            p->flag = dim;

            output[index] = p;

            // left tree
            if (m > start)
            {
                if (start < m - 1)
                    buildBalancedKdTree(input, output, 2*index, start, m - 1);
                else
                    output[2*index] = input[start];
            }
            // right tree
            if (m < end)
            {
                if (m + 1 < end)
                    buildBalancedKdTree(input, output, 2*index+1, m + 1, end);
                else
                    output[2*index+1] = input[end];
            }
        }

        struct PhotonQuery
        {
            PhotonQuery(const PhotonMap::KdTree & _map)
                : map(_map)
            {
                //queue.reserve(nMaxPhotons);
            }

            glm::vec3 point; // query point
            int numPhotons; // max number of result photons

            real d; // largest squared distance

            const PhotonMap::KdTree & map;

            typedef std::pair<real, const Photon*> PhotonEntry;
            std::priority_queue<PhotonEntry> queue;
        };

        void locatePhotons(PhotonQuery & q, int index)
        {
            const Photon *p = q.map[index];

            // if p has at least one child
            if(2*index < q.map.size())
            {
                const real delta = q.point[p->flag] - p->position[p->flag];
                if(delta < 0)
                {
                    // left side of plane

                    locatePhotons(q, 2*index);
                    if(2*index+1 < q.map.size())
                    {
                        if(delta*delta < q.d)
                            locatePhotons(q, 2*index+1);
                    }
                }
                else
                {
                    // right side of splitting plane

                    if(2*index+1 < q.map.size())
                        locatePhotons(q, 2*index+1);
                    if(delta*delta < q.d)
                        locatePhotons(q, 2*index);
                }
            }
            
            // squared distance from current photon to query point
            const glm::vec3 & diff = q.point - p->position;
            const real delta = glm::dot(diff, diff);

            if(delta < q.d)
            {
                q.queue.push(std::make_pair(delta, p));

                // Pop the worst photon
                if(q.queue.size() > q.numPhotons)
                    q.queue.pop();

                // Use the now worst photon to update search distance
                if(q.queue.size() == q.numPhotons)
                {
                    // the worst photon (longest distance to x) in the queue
                    const Photon *t = q.queue.top().second;

                    // Adjust max search distance
                    const glm::vec3 & diff = q.point - t->position;
                    q.d = glm::dot(diff, diff);
                }
            }
        }

        // Recursively construct bounding box hierarchy from photon map
        const glm::aabb & constructBBH(const PhotonMap::KdTree & kdTree, unsigned index)
        {
            Photon *p = kdTree[index];
            p->bbox = glm::aabb(p->position, p->radius);
        
            if(2*index < kdTree.size()) // left child
                p->bbox.expand(constructBBH(kdTree, 2*index));
            if(2*index+1 < kdTree.size()) // right child
                p->bbox.expand(constructBBH(kdTree, 2*index+1));

            return p->bbox;
        }

        // Assigns radii for the photons in the kDtree by approximating local density of photons
        void assignRadii(const PhotonMap::KdTree & kdTree, unsigned n)
        {
            const real m = glm::sqrt(real(n)); // only use m << n photons in the local density estimation
            const real e = glm::cbrt(n/m);

            for(int i = 1; i < kdTree.size(); i++)
            {
                Photon *p = kdTree[i];

                PhotonQuery query(kdTree);

                query.numPhotons = m+1; // m + the query photon
                query.point = p->position;
                query.d = real(10); // TODO: could optimize this

                locatePhotons(query, 1);

                p->radius = glm::sqrt(query.d) * e;
            }
        }

        // Intersects photon bounding boxes with given ray
        // Instead of finding the closest interesction, this function obtains all intersections
        void intersectPhoton(const glm::ray & ray, real maxd, const PhotonMap::KdTree & kdTree, unsigned index, PhotonMap::Photons & photons)
        {
            const Photon *p = kdTree[index];

            const glm::vec3 & o = ray.origin;
            const glm::vec3 invd(1/ray.dir.x, 1/ray.dir.y, 1/ray.dir.z);

            real tmin = real(0), tmax = maxd;
            if(glm::intersectRayAABB(o, invd, p->bbox, tmin, tmax))
            {
                photons.push_back(p);

                if(2*index < kdTree.size()) // left child
                    intersectPhoton(ray, maxd, kdTree, 2*index, photons);
                if(2*index+1 < kdTree.size()) // right child
                    intersectPhoton(ray, maxd, kdTree, 2*index+1, photons);
            }
        }
    }

    PhotonMap::PhotonMap()
    {}

    PhotonMap::PhotonMap(int capacity)
    {
        reserve(capacity);
    }

    unsigned PhotonMap::size() const
    {
        return kdTree.size();
    }

    bool PhotonMap::empty() const
    {
        return kdTree.empty();
    }

    void PhotonMap::reserve(int capacity)
    {
        kdTree.reserve(capacity);
    }

    void PhotonMap::store(Photon *p)
    {
        s_mutex.lock();

        if(kdTree.size() < kdTree.capacity())
            kdTree.push_back(p);

        s_mutex.unlock();
    }

    void PhotonMap::balance()
    {
        if (empty())
            return;

        // New KdTree to hold output (the original input will be modified)
        KdTree output(kdTree.size()+1, nullptr);
        
        Pane::info("Start building KDTree (size %d)", kdTree.size());

        buildBalancedKdTree(&kdTree[0], &output[0], 1, 0, kdTree.size()-1);

        kdTree = std::move(output);

        Pane::info("Finished building KDTree");
    }

    void PhotonMap::constructBBH(unsigned n)
    {
        assignRadii(kdTree, n);

        glm::aabb root = Raytracer2::constructBBH(kdTree, 1);

        const glm::vec3 & c = root.center();
        const glm::vec3 & s = root.size();
        Pane::info("Building BBH finished (root (%f, %f, %f) (%f, %f, %f))", c.x, c.y, c.z, s.x, s.y, s.z);
    }

    bool PhotonMap::intersectPhotons(const glm::ray & ray, real maxd, Photons & photons) const
    {
        Raytracer2::intersectPhoton(ray, maxd, kdTree, 1, photons);
        return photons.size();
    }

    real PhotonMap::locatePhotons(const glm::vec3 & point, int numPhotons, Photons & photons, const real d) const
    {
        /*// Test:
        const glm::vec3 & testPoint = glm::vec3(3.8f, -10.0f, 5.6f);
        for(auto const & p : kdTree)
        {
            if(!p) continue;
            const real d = glm::distance(p->position, testPoint);
            Pane::info("Photon (%f, %f, %f), d = %f, d2 = %f", p->position.x, p->position.y, p->position.z, d, d*d);
        }*/

        PhotonQuery query(kdTree);

        query.numPhotons = numPhotons;
        query.point = point;
        query.d = d;

        Raytracer2::locatePhotons(query, 1);

        photons.reserve(query.queue.size());

        while(query.queue.size() > 0)
        {
            auto p = query.queue.top();
            query.queue.pop();

            //Pane::info("Found: (%f, %f, %f) at distance %f", p.second->position.x, p.second->position.y, p.second->position.z, p.first);
            photons.push_back(p.second);
        }

        return query.d;
    }
}