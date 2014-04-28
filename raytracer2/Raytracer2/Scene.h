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

#ifndef SCENE_H
#define SCENE_H

#include "Object.h"
#include "Light.h"
#include "Medium.h"

#include <memory>
#include <vector>

namespace Raytracer2
{
    class Scene
    {
    public:
        void addObject(std::shared_ptr<Object> const & object);
        void addLight(std::shared_ptr<Light> const & light);
        void addMedium(std::shared_ptr<Medium> const & medium);

    public:
        typedef std::vector<std::shared_ptr<Object>> Objects;
        typedef std::vector<std::shared_ptr<Light>> Lights;
        typedef std::vector<std::shared_ptr<Medium>> Media;

        Scene::Objects & objects();
        const Scene::Objects & objects() const;
        Scene::Lights & lights();
        const Scene::Lights & lights() const;
        Scene::Media & media();
        const Scene::Media & media() const;

    private:
        Objects m_objects;
        Lights m_lights;
        Media m_media;
    };
    typedef std::shared_ptr<Scene> ScenePtr;
}

#endif // SCENE_H