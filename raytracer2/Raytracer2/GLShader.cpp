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

#include "GLShader.h"

#include <glm/gtc/type_ptr.hpp>
#include <fstream>

#include <map>

namespace Raytracer2
{
    namespace
    {
        typedef std::map<std::string, GLShader*> ShaderContainer;
        typedef ShaderContainer::iterator ShaderIterator;
        typedef ShaderContainer::const_iterator ShaderConstIterator;

        ShaderContainer s_shaders;

        void bindDefaultAttributes(GLShader *shader)
        {
            shader->bindAttribute(GLShader::AttributePosition, "in_Position");
            shader->bindAttribute(GLShader::AttributeColor, "in_Color");
            shader->bindAttribute(GLShader::AttributeTexCoord, "in_TexCoord");
        }
    }

	GLShader::GLShader()
	: num_ids(0)
	{
		// create shader program
		p_id = glCreateProgram();
	}

	GLShader::~GLShader()
	{
		for (unsigned i = 0; i < num_ids; i++)
		{
			glDetachShader(p_id, id_table[i]);
			glDeleteShader(id_table[i]);
		}

		glDeleteProgram(p_id);
	}

	void GLShader::link()
	{
		// link the program
		glLinkProgram(p_id);

		// query information about the linked program
		/*int num_active;
		int max_length;

		glGetProgramiv(p_id, GL_ACTIVE_ATTRIBUTES, &num_active);
		glGetProgramiv(p_id, GL_ACTIVE_ATTRIBUTE_MAX_LENGTH, &max_length);

		char * attr_name = new char[max_length];

		// loop through all active attributes
		for (int i = 0; i < num_active; i++)
		{
			int attrib_size;
			GLenum attrib_type;

			glGetActiveAttrib(p_id, i, max_length, 0, &attrib_size, &attrib_type, attr_name);
			Trace::info("Found GLSL attibute %s\n", attr_name);
		}

		delete [] attr_name;*/
	}

	void GLShader::bindAttribute(GLuint index, const GLchar* name)
	{
		glBindAttribLocation(p_id, index, name);
	}

    bool GLShader::loadFromFile(const std::string & filename, GLenum shaderType)
    {
        std::ifstream ifs(filename.c_str());
		if (!ifs.is_open()) {
            Pane::warning("Could not open: %s\n", filename.c_str());
			return false;
		}

		const std::string source((std::istreambuf_iterator<char>(ifs)), std::istreambuf_iterator<char>());

        return this->load(source, shaderType);
    }

	bool GLShader::load(const std::string & source, GLenum shaderType)
	{
		if (source.empty())
			return false;

		id_table[num_ids] = glCreateShader(shaderType);
		int id = id_table[num_ids++];

		const GLchar* sstring = source.c_str();
		const GLint slength = source.length();
		glShaderSource(id, 1, &sstring, &slength);

		if (!compile(id))
			return false;

		glAttachShader(p_id, id);
		return true;
	}

	bool GLShader::compile(unsigned int id)
	{
		GLint compiled;

		// compile shader
		glCompileShader(id);

		// check for compile status
		glGetShaderiv(id, GL_COMPILE_STATUS, &compiled);
		if (!compiled)
		{
			GLint length;
			char* log;

			// retreive log length
			glGetShaderiv(id, GL_INFO_LOG_LENGTH, &length);
		
			if (length > 0)
			{
				// retreive log contents and output it to cout
				log = new char[length];
				glGetShaderInfoLog(id, length, 0, log);
				Pane::error("Shader compile failed: %s\n", log);
				delete[] log;
			}

			return false;
		}
	
		Pane::info("Shader compile succeeded!\n");
		return true;
	}

	void GLShader::setUniform(const GLchar *name, const glm::vec3& v)
	{
		glUniform3dv(glGetUniformLocation(p_id, name), 1, glm::value_ptr(v));
	}

	void GLShader::setUniform(const GLchar *name, const glm::vec4& v)
	{
		glUniform4dv(glGetUniformLocation(p_id, name), 1, glm::value_ptr(v));
	}

	void GLShader::setUniform(const GLchar *name, const glm::mat4& m)
	{
		glUniformMatrix4dv(glGetUniformLocation(p_id, name), 1, GL_FALSE, glm::value_ptr(m));
	}

	void GLShader::setUniform(const GLchar *name, const glm::float_t& f)
	{
		glUniform1d(glGetUniformLocation(p_id, name), f);
	}

    void GLShader::setUniform(const GLchar *name, const glm::uint_t& i)
	{
		glUniform1i(glGetUniformLocation(p_id, name), i);
	}

    void GLShader::setUniform(const GLchar *name, int i)
	{
		glUniform1i(glGetUniformLocation(p_id, name), i);
	}

	void GLShader::setUniform(const GLchar *name, bool b)
	{
		glUniform1i(glGetUniformLocation(p_id, name), b);
	}

	void GLShader::bind() const
	{
		glUseProgram(p_id);
	}

	void GLShader::unbind() const
	{
		glUseProgram(0);
	}

    GLShader* GLShader::loadTechnique(const std::string & technique, const std::string & path)
    {
        ShaderIterator it = s_shaders.find(technique);
		if (it != s_shaders.end())
			return it->second;

        GLShader* shader = new GLShader;

		if (!shader->loadFromFile(path + technique + ".vert", GL_VERTEX_SHADER)) {
            Pane::error("Could not find %s.vert\n", (path + technique).c_str());
			return nullptr;
		}
		if (!shader->loadFromFile(path + technique + ".frag", GL_FRAGMENT_SHADER)) {
			Pane::error("Could not find %s.frag\n", (path + technique).c_str());
			return nullptr;
		}
		if (!shader->loadFromFile(path + technique + ".geom", GL_GEOMETRY_SHADER)) {
			Pane::warning("Could not find %s.geom\n", (path + technique).c_str());
		}

        bindDefaultAttributes(shader);

		shader->link();

        s_shaders[technique] = shader;

        return shader;
    }

    GLShader* GLShader::load(const std::string & vert, const std::string & frag, const std::string & geom)
    {
		GLShader* shader = new GLShader;

        if(!vert.empty() && shader->load(vert, GL_VERTEX_SHADER))
            Pane::error("Could not load vertex shader");
        if(!frag.empty() && shader->load(frag, GL_FRAGMENT_SHADER))
            Pane::error("Could not load fragment shader");
        if(!geom.empty() && shader->load(geom, GL_GEOMETRY_SHADER))
            Pane::error("Could not load geometry shader");

        bindDefaultAttributes(shader);

		shader->link();
			
		return shader;
    }
}