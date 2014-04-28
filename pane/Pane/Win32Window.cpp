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

#include "Win32Window.h"

#include "Trace.h"

#include <Windows.h>

#include <iostream>
#include <memory>

namespace Pane
{
	namespace
	{
        Win32Window * g_window;

		LRESULT CALLBACK WndProc(HWND hWnd, UINT Msg, WPARAM wParam, LPARAM lParam)
		{
            LPRECT rect = nullptr;

			switch(Msg)
			{
			case WM_DESTROY:
				PostQuitMessage(0);
				break;
            case WM_SIZING:
                rect = (LPRECT)lParam;
                if (rect)
                    g_window->resize(rect->right - rect->left, rect->bottom - rect->top);
                break;
			default:
				return DefWindowProc(hWnd, Msg, wParam, lParam);
			}

			return 0;
		}
	}

	Win32Window::Win32Window(const WindowConf & conf, HINSTANCE hInstance, int nCmdShow)
		: Window(conf)
	{
		WNDCLASSEX wc;
		ZeroMemory(&wc, sizeof(WNDCLASSEX));

		wc.cbSize = sizeof(WNDCLASSEX);
		wc.style = CS_HREDRAW | CS_VREDRAW;
		wc.lpfnWndProc = WndProc;
		wc.hInstance = hInstance;
		wc.hCursor = LoadCursor(NULL, IDC_ARROW);
		wc.hbrBackground = (HBRUSH)COLOR_WINDOW;
		wc.lpszClassName = "BasicWindow";

		if(!RegisterClassEx(&wc)) {
			MessageBox(NULL, "Failed to register window class!", "Error", 0);
			std::cout << "Error: " << GetLastError() << std::endl;
			return;
		}

		RECT vr = { 0, 0, conf.width, conf.height };
		if(!AdjustWindowRect(&vr, WS_OVERLAPPEDWINDOW, FALSE)) {
			MessageBox(NULL, "Failed to adjust window rect!", "Error", 0);
			std::cout << "Error: " << GetLastError() << std::endl;
		}
        resize(conf.width, conf.height);

		m_hWnd = CreateWindow("BasicWindow", conf.title.c_str(), WS_OVERLAPPEDWINDOW, CW_USEDEFAULT, CW_USEDEFAULT, vr.right - vr.left, vr.bottom - vr.top, NULL, NULL, hInstance, NULL);
		if(!m_hWnd) {
			MessageBox(NULL, "Failed to create window", "Error", 0);
			std::cout << "Error: " << GetLastError() << std::endl;
		}

		ShowWindow(m_hWnd, nCmdShow);
		UpdateWindow(m_hWnd);

        g_window = this;
		m_running = TRUE;
	}

	Win32Window::~Win32Window()
	{
		DestroyWindow(m_hWnd);
		//UnregisterClass(L"BasicWindow", hInstance);
	}

	void Win32Window::processEvents()
	{
		MSG msg = {0};

		while(PeekMessage(&msg, NULL, 0, 0, PM_REMOVE))
		{
			TranslateMessage(&msg);
			DispatchMessage(&msg);

            if(msg.message == WM_QUIT)
            {
                m_exitCode = (int)msg.wParam;
				m_running = FALSE;
            }
		}
	}

	void Win32Window::exit(int exitCode)
	{
		m_exitCode = exitCode;
		m_running = FALSE;
	}

    void Win32Window::resize(int width, int height)
    {
        m_width = width;
        m_height = height;

        if (m_eventHook)
            m_eventHook->windowResized(width, height);
    }
}