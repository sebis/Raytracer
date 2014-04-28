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

#ifndef GLSHADER_H
#define GLSHADER_H

#include <Pane/Trace.h>

#include <string>
#include <map>
#include <GL/glew.h>
#include <glm/glm.hpp>

namespace Raytracer2
{
	class GLShader
	{
	private:
		GLShader();

		unsigned id_table[10];
		unsigned num_ids;

		unsigned p_id;

		bool load(const std::string & source, GLenum shaderType);
        bool loadFromFile(const std::string & filename, GLenum shaderType);
		bool compile(unsigned id);
		void link();

	public:
		virtual ~GLShader();

		enum Attribute
		{
			AttributePosition = 0,
			AttributeColor = 1,
			AttributeTexCoord = 2,
		};

		void bind() const;
		void unbind() const;

        void bindAttribute(GLuint index, const GLchar* name);

		void setUniform(const GLchar *name, const glm::vec3& v);
		void setUniform(const GLchar *name, const glm::vec4& v);
		void setUniform(const GLchar *name, const glm::mat4& m);
		void setUniform(const GLchar *name, const glm::float_t& f);
        void setUniform(const GLchar *name, const glm::uint_t& i);
        void setUniform(const GLchar *name, int i);

		void setUniform(const GLchar *name, bool b);

        static GLShader* loadTechnique(const std::string & technique, const std::string & path = std::string());
		static GLShader* load(const std::string & vert, const std::string & frag, const std::string & geom = std::string());
	};
}

#endif // GLSHADER_H
