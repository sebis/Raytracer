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

#include "OpenGLApp.h"
#include "Trace.h"

#include <GLFW/glfw3.h>

#include <cassert>

namespace Pane
{
    OpenGLApp::OpenGLApp()
        : m_window(nullptr)
    {
        if(!glfwInit())
        {
            Pane::error("Could not initialize GLFW");
            glfwTerminate();
        }
        else
        {
            Pane::info("Loaded GLFW version %d.%d.%d", GLFW_VERSION_MAJOR, GLFW_VERSION_MINOR, GLFW_VERSION_REVISION);
        }
    }

    OpenGLApp::~OpenGLApp()
    {
        glfwTerminate();
    }

    bool OpenGLApp::createWindow(const WindowConf & conf)
    {
        assert(!m_window);

        m_window = std::make_shared<OpenGLWindow>(conf);

        return m_window != nullptr;
    }

    bool OpenGLApp::init()
    {
        // No need to initialize, OpenGLWindow does everything needed
        return true;
    }

    int OpenGLApp::run()
    {
        m_timer.reset();

        while(m_window->running())
        {
            glfwMakeContextCurrent(m_window->handle());

            m_window->processEvents();

            m_timer.tick();

            update(m_timer);
            draw();

            glfwSwapBuffers(m_window->handle());
        }

        return m_window->exitCode();
    }

    void OpenGLApp::exit(int exitCode)
    {
        m_window->exit(exitCode);
    }

    void OpenGLApp::update(const Pane::GameTime & gametime)
    {
    }

    void OpenGLApp::draw()
    {
    }
}
