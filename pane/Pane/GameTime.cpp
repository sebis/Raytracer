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

#include "GameTime.h"

#include <Windows.h>

#include <algorithm>

namespace
{
	__int64 g_tickFrequency = 0;
	double g_secondsPerCount = 0.0;
}

namespace Pane
{
	GameTime::GameTime()
		: m_elapsed(0.0)
		, m_totalTime(0.0)
		, m_running(false)
	{
		QueryPerformanceFrequency(reinterpret_cast<LARGE_INTEGER*>(&g_tickFrequency));

		g_secondsPerCount = 1.0 / g_tickFrequency;
	}

	GameTime::~GameTime()
	{
	}

	void GameTime::reset()
	{
		__int64 currTime;
		QueryPerformanceCounter(reinterpret_cast<LARGE_INTEGER*>(&currTime));

		m_prevTime = currTime;
		m_startTime = currTime;

		m_running = true;
	}

	void GameTime::tick()
	{
		if(!m_running) {
			m_elapsed = 0.0;
			return;
		}

		__int64 currTime;
		QueryPerformanceCounter(reinterpret_cast<LARGE_INTEGER*>(&currTime));

		m_elapsed = std::max(0.0, (currTime - m_prevTime) * g_secondsPerCount);
		m_totalTime = (currTime - m_startTime) * g_secondsPerCount;

		m_prevTime = currTime;
	}
}

