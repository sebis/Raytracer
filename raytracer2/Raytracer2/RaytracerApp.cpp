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

#include "RaytracerApp.h"

#include "GLMExtensions.h"
#include "LambertShader.h"
#include "PhotonMap.h"
#include "PointLight.h"
#include "Renderers.h"
#include "Sphere.h"
#include "Transform.h"

#include <Pane/Trace.h>

#include <glm/gtc/matrix_transform.hpp>
#include <GL/glew.h>

#include <algorithm>
#include <atomic>
#include <mutex>
#include <list>
#include <thread>

namespace
{
    clock_t s_timer;
    void start(clock_t & c)
    {
        c = clock();
    }

    double since(const clock_t & c)
    {
        return double(clock()-c)/CLOCKS_PER_SEC;
    }

    #define GLSL(version, shader) "#version " #version "\n" #shader
    
    std::string s_vert = GLSL(150,
        uniform mat4 viewMatrix;
        uniform mat4 projMatrix;
 
        in vec3 in_Position;
        in vec4 in_Color;
        in vec2 in_TexCoord;
 
        out vec4 fs_Color;
        out vec2 fs_TexCoord;

        void main()
        {
            fs_Color = in_Color;
            fs_TexCoord = in_TexCoord;
            gl_Position = projMatrix * vec4(in_Position, 1.0);
        }
    );

    std::string s_frag = GLSL(150,
        in vec4 fs_Color;
        in vec2 fs_TexCoord;

        out vec4 out_Color;

        uniform sampler2D tex;

        void main()
        {
            out_Color = texture(tex, fs_TexCoord) * fs_Color;
        }
    );

    unsigned int s_vao;
    unsigned int s_tex;
}

namespace Raytracer2
{
    class TracerThread
    {
    private:
        static unsigned s_counter;
    public:
        explicit TracerThread(const Renderer & renderer, const glm::mat4 & camInv, const glm::vec4 & vp, const glm::irect & region)
            : m_id(s_counter++)
            , m_renderer(renderer)
            , m_camInv(camInv)
            , m_vp(vp)
            , m_region(region)
            , m_data(nullptr)
            , m_done(false)
        {
            m_stop = false;

            const size_t n = 3 * m_region.width() * m_region.height();
            m_data = new float[n];
        }

        explicit TracerThread(const TracerThread && other)
            : m_id(std::move(other.m_id))
            , m_renderer(std::move(other.m_renderer))
            , m_camInv(std::move(other.m_camInv))
            , m_vp(std::move(other.m_vp))
            , m_region(std::move(other.m_region))
            , m_dirty(std::move(other.m_dirty))
            , m_stop(std::move(other.m_stop))
            , m_done(std::move(other.m_done))
        {
            const size_t n = 3 * m_region.width() * m_region.height();
            m_data = new float[n];
        }

        ~TracerThread()
        {
            stop();

            if(m_data)
                delete [] m_data;
        }

        void start()
        {
            m_thread = std::thread(&TracerThread::trace, this);
        }

        void stop()
        {
            m_stop = true;
            if (m_thread.joinable())
                m_thread.join();
        }

        unsigned id() const
        {
            //return m_thread.get_id();
            return m_id;
        }

        bool done() const
        {
            return m_done;
        }

        void trace()
        {
            clock_t timer;

            ::start(timer);

            for(unsigned y = 0; y < m_region.height(); ++y)
            {
                for (unsigned x = 0; x < m_region.width(); ++x)
                {
                    size_t i = 3 * (y * m_region.width() + x);

                    // always use 4-byte values
                    glm::detail::tvec3<float> c = (glm::detail::tvec3<float>)m_renderer.tracePixel(m_camInv, m_vp, m_region.min.x + x, m_region.min.y + y);
                    memmove(m_data + i, &c, sizeof(glm::detail::tvec3<float>));

                    if (m_stop)
                        goto end;
                }

                m_mutex.lock();

                m_dirty.expand(glm::ivec2(m_region.min.x, m_region.min.y + y));
                m_dirty.expand(glm::ivec2(m_region.max.x, m_region.min.y + y + 1));

                m_mutex.unlock();
            }

end:
            Pane::info("Rendering took %.2fs", ::since(timer));

            m_done = true;
        }

        bool getNewData(float ** data, glm::irect & region)
        {    
            m_mutex.lock();

            const int x = m_dirty.min.x - m_region.min.x;
            const int y = m_dirty.min.y - m_region.min.y;

            if (x < 0 || y < 0)
            {
                m_mutex.unlock();
                return false;
            }

            size_t dst = 3 * (y * m_dirty.width() + x);

            std::swap(region, m_dirty);
            assert(m_dirty.empty());

            m_mutex.unlock();

            *data = m_data + dst;

            return true;
        }

    private:
        std::thread m_thread;
        std::mutex m_mutex;
        std::atomic_bool m_stop;

        const glm::irect m_region;
        glm::irect m_dirty;

        const unsigned m_id;
        const Renderer & m_renderer;
        const glm::mat4 m_camInv;
        const glm::vec4 m_vp;
        float * m_data;

        bool m_done;
    };

    unsigned TracerThread::s_counter = 0;

    namespace
    {
        const bool s_exitWhenDone = false;

        std::list<TracerThread> s_threads;
    }

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

    RaytracerApp::RaytracerApp()
        : m_nthreads(1)
    {}

    RaytracerApp::~RaytracerApp()
    {
        for(auto & t : s_threads)
            t.stop();

        glDeleteVertexArrays(1, &s_vao);
        glDeleteTextures(1, &s_tex);
    }

    bool RaytracerApp::init()
    {
        glewExperimental = GL_TRUE;
        if(glewInit())
        {
            Pane::error("Could not initialize GLEW");
            return false;
        }

        GLenum err = glGetError();
        if(err != GL_NO_ERROR)
        {
            // Sometimes GLEW give this error for no apparent reason
            // Can be ignored..
            if(err != GL_INVALID_ENUM)
            {
                Pane::error("Error during GLEW initialization: %d", err);
                return false;
            }
        }

        m_shader = GLShader::load(s_vert, s_frag);

        // Create vertex data
        GLuint buffers[3]; 

        const float vertices[] = { 0,0,0, 1,0,0, 0,1,0, 1,1,0 };
        const float colors[] = { 1,1,1,1, 1,1,1,1, 1,1,1,1, 1,1,1,1 };
        const float texcoords[] = { 0,0, 1,0, 0,1, 1,1 };

        glGenVertexArrays(1, &s_vao);

        glBindVertexArray(s_vao);
        glGenBuffers(3, buffers);

        glBindBuffer(GL_ARRAY_BUFFER, buffers[0]);
        glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);

        glVertexAttribPointer(GLShader::AttributePosition, 3, GL_FLOAT, GL_FALSE, 0, 0);
        glEnableVertexAttribArray(GLShader::AttributePosition);
 
        glBindBuffer(GL_ARRAY_BUFFER, buffers[1]);
        glBufferData(GL_ARRAY_BUFFER, sizeof(colors), colors, GL_STATIC_DRAW);

        glVertexAttribPointer(GLShader::AttributeColor, 4, GL_FLOAT, GL_FALSE, 0, 0);
        glEnableVertexAttribArray(GLShader::AttributeColor);

        glBindBuffer(GL_ARRAY_BUFFER, buffers[2]);
        glBufferData(GL_ARRAY_BUFFER, sizeof(texcoords), texcoords, GL_STATIC_DRAW);

        glVertexAttribPointer(GLShader::AttributeTexCoord, 2, GL_FLOAT, GL_FALSE, 0, 0);
        glEnableVertexAttribArray(GLShader::AttributeTexCoord);

        glBindVertexArray(0);

        // Create texture

        unsigned int n = 3 * m_window->width() * m_window->height();
        unsigned char * data = new unsigned char[n];
        memset(data, 0x0, sizeof(unsigned char) * n);

        glGenTextures(1, &s_tex);

        glBindTexture(GL_TEXTURE_2D, s_tex);

        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB32F, m_window->width(), m_window->height(), 0, GL_RGB, GL_UNSIGNED_BYTE, data);
        glBindTexture(GL_TEXTURE_2D, 0);

        delete [] data;

        glClearColor(0, 0, 0, 0);

        err = glGetError();
        if(err != GL_NO_ERROR)
        {
            Pane::error("Error during initialization: %d", err);
            return false;
        }

        m_film = std::unique_ptr<Film>(new Film);
        m_film->allocate(m_window->width(), m_window->height());

        return true;
    }

    void RaytracerApp::createRenderer(OptionsPtr const & options, ScenePtr const & scene)
    {
        m_renderer = Renderers::createRenderer(options, scene);
    }

    void RaytracerApp::onMouseClick(int button, int action, int x, int y)
    {
        y = m_window->height() - y;

        if(button == Pane::MouseRight && (action == Pane::Press || action == Pane::Repeat))
        {
            Pane::info("Pressed at %d,%d", x, y);

            if(s_threads.size() >= m_nthreads)
            {
                Pane::warning("Maximum allowed threads already running. Wait for at least one to stop before continuing");
            }
            else
            {
                s_threads.emplace_back(*m_renderer, m_transform, m_viewport, glm::irect(x, y, x+1, y+1));
                s_threads.back().start();
            }
        }
    }

    void RaytracerApp::onKey(int key, int scancode, int action, int mods)
    {
        if(key == Pane::Key_Space && action == Pane::Press)
        {
            saveToFile(m_renderer->opt().s_name + std::string(".pfm"));
        }
    }

    void RaytracerApp::onExit()
    {
        for (auto & t : s_threads)
            t.stop();
    }

    void RaytracerApp::saveToFile(const std::string & filename)
    {
        m_film->save(filename);
    }

    void RaytracerApp::setPerspective(const glm::vec3 & position, const glm::vec3 & lookAt, glm::float_t fov)
    {
        const unsigned width = m_window->width();
        const unsigned height = m_window->height();

        m_viewport = glm::vec4(0.f, 0.f, width, height);

        const glm::mat4 & proj = glm::perspective(fov, glm::float_t(width) / glm::float_t(height), 0.1, 10000.0);
        const glm::mat4 & view = glm::lookAt(position, lookAt, glm::vec3(0, 1, 0));

        m_transform = glm::inverse(proj * view);
    }

    void RaytracerApp::setThreads(unsigned nthreads)
    {
        m_nthreads = nthreads;
    }

    int RaytracerApp::run()
    {
        m_renderer->initialize();
        Pane::info("Finished initialization");

#ifndef _DEBUG
        const unsigned width = m_window->width();
        const unsigned height = m_window->height();

        for(int i = 0; i < m_nthreads; i++)
        {
            const unsigned y0 = i * (height / m_nthreads);
            const unsigned y1 = (i+1) * (height / m_nthreads);

            s_threads.emplace_back(*m_renderer, m_transform, m_viewport, glm::irect(0, y0, width, y1));

            Pane::info("Creating thread with region: (%d,%d)x(%d,%d)", 0, y0, width, y1);
        }

        for(auto & t : s_threads)
        {
            Pane::info("Starting thread: %d", t.id());
            t.start();
        }
#endif

        return OpenGLApp::run();
    }

    void RaytracerApp::update(const Pane::GameTime & gameTime)
    {
        for (auto & t : s_threads)
        {
            float * data;
            glm::irect region;

            if (t.getNewData(&data, region) && !region.empty())
            {
                //Pane::info("Traced region: (%d,%d)x(%d,%d), w: %d, h: %d (id: %d)", region.min.x, region.min.y, region.max.x, region.max.y, region.width(), region.height(), t.id());
                m_film->setPixels(region, data);

                glBindTexture(GL_TEXTURE_2D, s_tex);
                glTexSubImage2D(GL_TEXTURE_2D, 0, region.min.x, region.min.y, region.width(), region.height(), GL_RGB, GL_FLOAT, data);
                glBindTexture(GL_TEXTURE_2D, 0);
            }
        }

        // clean up finished threads
        s_threads.remove_if([] (const TracerThread & t) { return t.done(); });

        if(s_exitWhenDone && s_threads.empty())
            exit(0);
    }

    void RaytracerApp::draw()
    {
        glClear(GL_COLOR_BUFFER_BIT);

        m_shader->bind();

        const glm::mat4 projection = glm::ortho<glm::float_t>(0, 1, 0, 1, 0, 1);
        m_shader->setUniform("projMatrix", projection);
        m_shader->setUniform("tex", 0);

        glActiveTexture(GL_TEXTURE0 + 0);
        glBindTexture(GL_TEXTURE_2D, s_tex);

        glBindVertexArray(s_vao);
        glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
        glBindVertexArray(0);

        glBindTexture(GL_TEXTURE_2D, 0);
        m_shader->unbind();

        GLenum err = glGetError();
        if(err != GL_NO_ERROR)
            Pane::error("Error during rendering: %d", err);

        // Sleep for 16ms
        std::this_thread::sleep_for(std::chrono::milliseconds(16));
    }

    void RaytracerApp::exit(int exitCode)
    {
        for(auto & t : s_threads)
            t.stop();

        OpenGLApp::exit(exitCode);
    }
}