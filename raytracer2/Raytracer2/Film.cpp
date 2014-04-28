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

#include "Film.h"

#include <Pane/Trace.h>

#include "FreeImage/FreeImage.h"

namespace Raytracer2
{
    class Film::D
    {
    public:
        D()
            : image(nullptr)
            , width(-1)
            , height(-1)
        {
        }

        ~D()
        {
            if(image)
                FreeImage_Unload(image);
        }

        FIBITMAP* image;
        int width;
        int height;
    };

    Film::Film()
        : m_d(new D)
    {
    }

    Film::~Film()
    {
        delete m_d;
    }

    void Film::allocate(unsigned width, unsigned height)
    {
        m_d->image = FreeImage_AllocateT(FIT_RGBF, width, height);
        m_d->width = width;
        m_d->height = height;
    }

    void Film::setPixels(const glm::irect & region, float * data)
    {
        FIRGBF* pixels = (FIRGBF*)FreeImage_GetBits(m_d->image);
        for(int x = 0; x < region.width(); x++)
        {
            for(int y = 0; y < region.height(); y++)
            {
                int line = m_d->height - 1 - (region.min.y + y);
                int i = region.min.x + x;
                pixels[m_d->width*line + i].red = (float)data[3*(m_d->width*y + x)];
                pixels[m_d->width*line + i].green = (float)data[3*(m_d->width*y + x) + 1];
                pixels[m_d->width*line + i].blue = (float)data[3*(m_d->width*y + x) + 2];
            }
        }
    }

    void Film::save(const std::string & filename)
    {
        FreeImage_Save(FIF_PFM, m_d->image, filename.c_str(), PFM_DEFAULT);
        Pane::info("Saved file to %s", filename.c_str());
    }
}