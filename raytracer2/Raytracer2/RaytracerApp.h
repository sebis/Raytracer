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

#ifndef RAYTRACERAPP_H
#define RAYTRACERAPP_H

#include "Film.h"
#include "Raytracer.h"
#include "GLShader.h"

#include <Pane/GameTime.h>
#include <Pane/OpenGLApp.h>

namespace Raytracer2
{
    class RaytracerApp : public Pane::OpenGLApp
    {
    public:
        RaytracerApp();
        virtual ~RaytracerApp();

        bool init() override;
        int run() override;

        void createRenderer(OptionsPtr const & options, ScenePtr const & scene);

        void onMouseClick(int button, int action, int x, int y);
        void onKey(int key, int scancode, int action, int mods);
        void onExit();

        void saveToFile(const std::string & filename);

        void setPerspective(const glm::vec3 & position, const glm::vec3 & lookAt, glm::float_t fov);
        void setThreads(unsigned nthreads);

    protected:
        void update(const Pane::GameTime & gameTime) override;
        void draw() override;
        void exit(int exitCode) override;

    private:
        Renderer * m_renderer;
        GLShader * m_shader;
        std::unique_ptr<Film> m_film;
        std::unique_ptr<Scene> m_scene;

        glm::vec4 m_viewport;
        glm::mat4 m_transform;

        unsigned m_nthreads;
    };
}

#endif // RAYTRACERAPP_H