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

#include "Trace.h"

#include <cassert>
#include <cstdarg>
#include <cstdio>

#include <iostream>
#include <mutex>

#ifdef _WIN32
#include <comdef.h>
#include <Windows.h>
#endif

#ifdef _MSC_VER
#define snprintf _snprintf
#endif

namespace Pane
{
	bool DXError(HRESULT hr, const char * msg)
	{
		if(FAILED(hr)) {
            const _com_error err(hr);

            const std::string str(err.ErrorMessage());

            Pane::debug("%s (%u: %s)", msg, err.WCode(), str.c_str());
			return true;
		}

		return false;
	}

	namespace
	{
		const char * labels[] = {
			"[DEBUG] ",
			"[INFO] ",
			"[WARNING] ",
			"[ERROR] "
		};

        std::string s_logfile;
        std::mutex s_mutex;

        void g_output(FILE * out, const char * label, const char * format, va_list & args)
        {
            char buffer[4096];
			char * bufptr = buffer;
			int size = sizeof(buffer), wrote = 0;
			
            wrote = snprintf(bufptr, size, "%s", label);
			if(wrote > 0) size -= wrote, bufptr += wrote;

			vsnprintf(bufptr, size, format, args);

			fprintf(out, "%s\n", buffer);

#ifdef _WIN32
            if(out == stdout || out == stderr)
            {
                ::OutputDebugStringA(buffer);
			    ::OutputDebugStringA("\n");
            }
#endif
        }

		void g_output(Severity s, const char * format, va_list & args)
		{
			FILE * out = (s > WARNING) ? stderr : stdout;
            g_output(out, labels[s], format, args);

            if(!s_logfile.empty())
            {
                s_mutex.lock();

                out = fopen(s_logfile.c_str(), "a");
                if(out)
                {
                    g_output(out, labels[s], format, args);
                    fclose(out);
                }

                s_mutex.unlock();
            }
		}
	}

    void set_log_file(const char * logfile)
    {
        s_logfile = logfile;
        FILE * f = fopen(logfile, "w");
        if(f)
            fclose(f);
        else
            Pane::error("Can not write to file %s", logfile);
    }

	void trace(Severity s, const char * format, ...)
	{
		va_list args;
		va_start (args, format);
		g_output(s, format, args);
		va_end (args);
	}

	void error(const char * format, ...)
	{
		va_list args;
		va_start(args, format);
		g_output(FAILURE, format, args);
		va_end(args);
	}
	
	void warning(const char * format, ...)
	{
		va_list args;
		va_start(args, format);
		g_output(WARNING, format, args);
		va_end(args);
	}

	void info(const char * format, ...)
	{
		va_list args;
		va_start(args, format);
		g_output(INFO, format, args);
		va_end(args);
	}

	void debug(const char * format, ...)
	{
#if defined DEBUG || _DEBUG
		va_list args;
		va_start(args, format);
		g_output(DEBUG, format, args);
		va_end(args);
#else
		(void)format;
#endif
	}
}