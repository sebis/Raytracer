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

#ifndef WINDOW_H
#define WINDOW_H

#include "Pane.h"

#include <functional>
#include <string>
#include <memory>

namespace Pane
{
    EXPORT_API extern const int Release;
    EXPORT_API extern const int Press;
    EXPORT_API extern const int Repeat;

    EXPORT_API extern const int MouseLeft;
    EXPORT_API extern const int MouseRight;

    EXPORT_API extern const int Key_Space;

    typedef std::function<void (int button, int action, int x, int y)> MouseCallback;
    typedef std::function<void (int key, int scancode, int action, int mods)> KeyCallback;
    typedef std::function<void ()> CloseCallback;

	struct WindowConf
	{
        WindowConf()
            : title("Window")
            , width(100)
            , height(100)
            , fullscreen(false)
            , mouseCallback(nullptr)
        {}

		std::string title;
		unsigned width;
		unsigned height;
		bool fullscreen;
        MouseCallback mouseCallback;
        KeyCallback keyCallback;
        CloseCallback closeCallback;
	};

	struct WindowEventHook
	{
		virtual void windowMoved() = 0;
        virtual void windowResized(unsigned width, unsigned height) = 0;
	};

	class Window
	{
	public:
		Window(const WindowConf & conf) : m_eventHook(nullptr), m_windowConf(conf), m_running(false), m_exitCode(-1) {}
		virtual ~Window() {};

		virtual void processEvents() = 0;
		virtual void exit(int exitCode) = 0;

		inline const WindowConf & configuration() const { return m_windowConf; }

		virtual unsigned width() const { return m_windowConf.width; }
		virtual unsigned height() const { return m_windowConf.height; }

		inline bool running() const { return m_running; }
		inline int exitCode() const { return m_exitCode; }
		inline void setEventHook(std::shared_ptr<WindowEventHook> eventHook) { m_eventHook = eventHook; }

	protected:
		std::shared_ptr<WindowEventHook> m_eventHook;

		WindowConf m_windowConf;

		bool m_running;
		int m_exitCode;
	};
}

#endif // WINDOW_H