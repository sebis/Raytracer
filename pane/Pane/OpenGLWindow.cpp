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

#include "OpenGLWindow.h"

#include "Trace.h"

#include <Windows.h>
#include <GLFW/glfw3.h>

#include <map>

namespace
{
    std::map<Handle, Pane::MouseCallback> mouse_callbacks;
    std::map<Handle, Pane::KeyCallback> key_callbacks;
    std::map<Handle, Pane::CloseCallback> close_callbacks;

    void key_callback(Handle handle, int key, int scancode, int action, int mods)
    {
        if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS)
            glfwSetWindowShouldClose(handle, GL_TRUE);

        auto cb = key_callbacks.find(handle);
        while(cb != key_callbacks.end())
        {
            cb->second(key, scancode, action, mods);
            ++cb;
        }
    }

    void mouse_callback(Handle handle, int button, int action, int mods)
    {
        double x, y;
        glfwGetCursorPos(handle, &x, &y);

        auto cb = mouse_callbacks.find(handle);
        while(cb != mouse_callbacks.end())
        {
            cb->second(button, action, int(x), int(y));
            ++cb;
        }
    }

    void close_callback(Handle handle)
    {
        auto cb = close_callbacks.find(handle);
        while(cb != close_callbacks.end())
        {
            cb->second();
            ++cb;
        }
    }
}

namespace Pane
{
    const int MouseLeft = GLFW_MOUSE_BUTTON_LEFT;
    const int MouseRight = GLFW_MOUSE_BUTTON_RIGHT;

    const int Release = GLFW_RELEASE;
    const int Press = GLFW_PRESS;
    const int Repeat = GLFW_REPEAT;

    const int Key_Space = GLFW_KEY_SPACE;

    OpenGLWindow::OpenGLWindow(const WindowConf & conf)
        : Window(conf)
    {
        glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
        glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 2);
        glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
        glfwWindowHint(GLFW_RED_BITS, 32);
        glfwWindowHint(GLFW_GREEN_BITS, 32);
        glfwWindowHint(GLFW_BLUE_BITS, 32);
        glfwWindowHint(GLFW_ALPHA_BITS, 0);
#if defined(_DEBUG)
        glfwWindowHint(GLFW_OPENGL_DEBUG_CONTEXT, GL_TRUE);
#endif

        m_handle = glfwCreateWindow(conf.width, conf.height, conf.title.c_str(), nullptr, nullptr);
        if(m_handle)
        {
            glfwMakeContextCurrent(m_handle);

            glfwSetKeyCallback(m_handle, key_callback);
            glfwSetMouseButtonCallback(m_handle, mouse_callback);
            glfwSetWindowCloseCallback(m_handle, close_callback);

            if(conf.mouseCallback)
                ::mouse_callbacks[m_handle] = conf.mouseCallback;
            if(conf.keyCallback)
                ::key_callbacks[m_handle] = conf.keyCallback;
            if(conf.closeCallback)
                ::close_callbacks[m_handle] = conf.closeCallback;

            m_running = true;

            Pane::info("Opened GLFW window '%s'", conf.title.c_str());
        }
        else
        {
            Pane::error("Could not open window");
        }
        
        GLenum err = glGetError();
        if(err != GL_NO_ERROR)
        {
            Pane::error("Error during window creationg: %d", err);
            m_running = false;
        }
    }

    OpenGLWindow::~OpenGLWindow()
    {
        glfwDestroyWindow(m_handle);
    }

    void OpenGLWindow::processEvents()
    {
        glfwPollEvents();

        if (glfwWindowShouldClose(m_handle))
            exit(0);
    }

    void OpenGLWindow::exit(int exitCode)
    {
        m_exitCode = exitCode;
        m_running = false;
    }

    Handle OpenGLWindow::handle() const
    {
        return m_handle;
    }
}