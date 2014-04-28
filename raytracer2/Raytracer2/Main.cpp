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

#include "Parser.h"

#include <Pane/Trace.h>
#include <Pane/Window.h>

#include <GL/glew.h>

using namespace Raytracer2;

int main(int argc, char * argv[])
{
    if (argc < 2)
    {
        Pane::error("Usage: %s [-jN] <filename>", argv[0]);
        return 1;
    }

    const std::string filename = argv[argc-1];

    // TODO: check that parse was successful
    ConfigurationParser parser;
    parser.parse(filename);

    // Get parsed options
    OptionsPtr options = parser.getOptions();

    // Set log file and dump scene file
    Pane::set_log_file((options->s_name + std::string(".log")).c_str());
    parser.dumpFile(filename);

    // Create application instance
    Raytracer2::RaytracerApp app;

    // Set number of threads
    if(argc > 2)
    {
        unsigned nthreads;
        if(sscanf(argv[1], "-j%u", &nthreads))
            app.setThreads(nthreads);
    }

    // Configure window
    Pane::WindowConf conf;
    conf.width = options->s_width;
    conf.height = options->s_height;
    conf.fullscreen = false;
    conf.title = std::string("Raytracer 2 - ") + options->s_name;
    conf.mouseCallback = std::bind(&RaytracerApp::onMouseClick, &app,
        std::placeholders::_1,
        std::placeholders::_2,
        std::placeholders::_3,
        std::placeholders::_4);
    conf.keyCallback = std::bind(&RaytracerApp::onKey, &app,
        std::placeholders::_1,
        std::placeholders::_2,
        std::placeholders::_3,
        std::placeholders::_4);
    conf.closeCallback = std::bind(&RaytracerApp::onExit, &app);

    if(!app.createWindow(conf))
    {
        Pane::error("Could not create window");
        return 2;
    }

    GLenum err = glGetError();
    if(err != GL_NO_ERROR)
        Pane::error("OpenGL error: %d", err);

    if(!app.init())
    {
        Pane::error("Could not initialize application");
        return 3;
    }

    // Set up camera matrices
    CameraPtr camera = parser.getCamera();
    app.setPerspective(camera->position(), camera->lookAt(), camera->fov());

    // Get parsed scene
    ScenePtr scene = parser.getScene();

    app.createRenderer(options, scene);

    int ret = app.run();
    if(!ret) app.saveToFile("raytracer2.pfm");

    return ret;
}

