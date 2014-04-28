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

#ifndef OPENGLAPP_H
#define OPENGLAPP_H

#include "Application.h"
#include "GameTime.h"
#include "OpenGLWindow.h"

namespace Pane
{
    class OpenGLApp : public Application
    {
    public:
        EXPORT_API OpenGLApp();
		EXPORT_API virtual ~OpenGLApp();

		EXPORT_API bool createWindow(const WindowConf & conf);

		EXPORT_API virtual bool init() override;
		EXPORT_API virtual int run() override;

		EXPORT_API virtual void exit(int exitCode) override;
      
    protected:
        EXPORT_API virtual void update(const GameTime & gameTime);
        EXPORT_API virtual void draw();

        std::shared_ptr<OpenGLWindow> m_window;

    private:
        GameTime m_timer;
    };
}

#endif // OPENGLAPP_H