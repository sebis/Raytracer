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

#include "Parser.h"

#include "Camera.h"
#include "Constants.h"
#include "DensityMedium.h"
#include "DirectionalLight.h"
#include "EmissiveShader.h"
#include "HGPhaseFunction.h"
#include "HomogeneousMedium.h"
#include "IsotropicPhaseFunction.h"
#include "Mesh.h"
#include "MeshGeometry.h"
#include "MeshImporter.h"
#include "Object.h"
#include "Options.h"
#include "PlanarLight.h"
#include "Plane.h"
#include "PointLight.h"
#include "Raytracer.h"
#include "ReflectShader.h"
#include "RefractShader.h"
#include "Sphere.h"
#include "Triangle.h"
#include "LambertShader.h"
#include "PhongShader.h"
#include "Transform.h"

#include <Pane/Trace.h>

#include <fstream>
#include <map>
#include <memory>
#include <list>
#include <regex>
#include <stack>
#include <string>

namespace
{
    using namespace Raytracer2;

    // Remove char ch from string value
    std::string remove(const std::string & value, char ch)
    {
        std::string tmp = value;
        tmp.erase(std::remove(tmp.begin(), tmp.end(), ch), tmp.end());
        return tmp;
    }

    // Remove leading spaces from str
    std::string trim(const std::string & str)
    {
        std::size_t found = str.find_first_not_of(' ');
        if(found != std::string::npos)
            return std::string(str.begin()+found, str.end());
        return str;
    }
    
    // Compute the hierarchy level of str
    int level(const std::string & str)
    {
        int level = 0;
        for(; level < str.length(); level++)
        {
            if(str[level] != ' ')
                return level;
        }
        return level;
    }

    // Allowed identifiers
    const std::string s_allowedIdentifiers[] = {
        "Object",
        "Geometry",
        "Light",
        "Shader",
        "Medium",
        "Density",
        "PhaseFunction",
        "Transform",
        "Scene",
        "ObjectGroup",
        "Options",
        "Camera",
        "Render"
    };

    // <Identifier> "<name>"
    bool is_identifier(const std::string & token, std::string & value)
    {
        bool success = false;
        for(auto const & str : s_allowedIdentifiers)
            if(token == str)
            {
                success = true;
                break;
            }
        if(!success) return false;
        success = std::regex_match(value, std::regex("\"\\w+\""));
        if(success)
            value = remove(value, '"');
        return success;
    }

    // <Identifier>: [<name>]
    bool is_reference(std::string & token, const std::string & value)
    {
        bool success = false;
        for(auto const & str : s_allowedIdentifiers)
            if(token == str + ':')
            {
                success = true;
                break;
            }
        if(!success) return false;
        success = std::regex_match(value, std::regex("\\w*"));
        if(success)
            token = remove(token, ':');
        return success;
    }

    // <property> <value>
    bool is_property(const std::string & token)
    {
        return std::regex_match(token, std::regex("[a-z0-9]+"));
    }

    std::string createAnonymousName(const std::string & token)
    {
        static int counter = 0;
        return std::string("ref-") + token + std::to_string(++counter);
    }

    // Tokenizes the line to command token and command value
    void tokenize(const std::string & line, std::string & token, std::string & value)
    {
        const std::string & str = trim(line);
        std::size_t found = str.find_first_of(' ');
        if(found != std::string::npos)
        {
            token = std::string(str.begin(), str.begin()+found);
            value = std::string(str.begin()+found+1, str.end());
        }
        else
        {
            token = str;
        }
    }

    glm::vec3 parseVector3(const std::string & value)
    {
        glm::vec3 v;
        sscanf(value.c_str(), "%lf %lf %lf", &v.x, &v.y, &v.z);
        return v;
    }

    glm::quat parseQuat(const std::string & value)
    {
        real val[4];
        sscanf(value.c_str(), "%lf %lf %lf %lf", &val[0], &val[1], &val[2], &val[3]);
        return glm::rotate(glm::quat(), val[0], glm::vec3(val[1], val[2], val[3]));
    }

    real parseSingle(const std::string & value)
    {
        real r;
        sscanf(value.c_str(), "%lf", &r);
        return r;
    }

    int parseInt(const std::string & value)
    {
        int i;
        sscanf(value.c_str(), "%d", &i);
        return i;
    }
}

namespace Raytracer2
{
    struct Render
    {
        std::shared_ptr<Scene> scene;
        std::shared_ptr<Options> options;
        std::shared_ptr<Camera> camera;
    };

    // Identifier      Type ["name"]
    // EndIdentifier   EndType
    // Reference       Type: name
    // Property        prop value [, value]

    class ConfigurationParser::D
    {
    private:
        struct Identifier
        {
            std::string type;
            std::string name;
        };

        struct Reference
        {
            std::string type;
            std::string name;

            std::shared_ptr<void> value;
        };

        struct AssetFrame
        {
            Identifier ident;

            int level;
            std::map<std::string, std::string> properties;
            std::list<Reference> references;
        };

    public:
        void dumpFile(const std::string & filename)
        {
            std::ifstream file(filename);
            if(!file.is_open())
            {
                Pane::error("Could not load configuration file (%s)", filename.c_str());
            }

            std::string line;

            while (file.good())
            {
                std::getline(file, line);
                Pane::info("%s", line.c_str());
            }
        }

        void parseFile(const std::string & filename)
        {
            std::ifstream file(filename);

            if(!file.is_open())
            {
                Pane::error("Could not load configuration file (%s)", filename.c_str());
            }

            std::string line;

            while (file.good())
            {
                std::getline(file, line);
                //Pane::info("Read line: %s", line.c_str());

                // Skip empty lines and comments
                if(!line.empty() && line[0] != '#')
                    parse(line);
            }

            // Make sure we terminate open assets
            if (!assets.empty())
                constructAssetFrame();
            assert(assets.empty());
        }

        void dumpMemory()
        {
            for(auto const & mem : memory)
            {
                const Reference & ref = mem.second;
                Pane::info("%s \"%s\"", ref.type.c_str(), ref.name.c_str());
            }
        }

        void clearMemory()
        {
            memory.clear();
        }

        std::shared_ptr<void> asset(const std::string & name)
        {
            assert(memory.find(name) != memory.end());
            return memory.at(name).value;
        }

        template <typename T>
        std::shared_ptr<T> asset(const std::string & name)
        {
            return std::static_pointer_cast<T>(asset(name));
        }

    private:
        bool parse(const std::string & line)
        {
            const int level = ::level(line);
            AssetFrame *top = nullptr;
            
            if(!assets.empty())
            {
                top = &assets.top();

                // Construct all frames that are on the same level as our current asset
                while (top && top->level >= level)
                {
                    constructAssetFrame();
                    top = !assets.empty() ? &assets.top() : nullptr;
                }
            }

            std::string token;
            std::string value;
            ::tokenize(line, token, value);

            //Pane::info("tokenize (%s) (%s)", token.c_str(), value.c_str());

            if(::is_identifier(token, value))
            {
                //Pane::info("Found identifier: %s", line.c_str());

                AssetFrame frame;

                frame.ident.type = token;
                frame.ident.name = value;
                frame.level = level;

                assets.push(frame);
            }
            else if(::is_reference(token, value))
            {
                //Pane::info("Found reference: %s", line.c_str());

                // anonymous reference
                bool anonymous = value.empty();
                if(anonymous)
                    value = ::createAnonymousName(token);

                // Create reference
                Reference ref;

                ref.type = token;
                ref.name = value;

                top->references.push_back(ref);

                if(anonymous)
                {
                    // Create a new anonymous frame and add a reference from this object
                    AssetFrame frame;

                    frame.ident.type = token;
                    frame.ident.name = value;
                    frame.level = level;

                    assets.push(frame);
                }
            }
            else if(::is_property(token))
            {
                //Pane::info("Found property: %s", line.c_str());
                top->properties.insert(std::make_pair(token, value));
            }
            else
            {
                Pane::warning("Could not parse line: %s", line.c_str());
                return false;
            }

            return true;
        }

        void constructAssetFrame()
        {
            const AssetFrame & top = assets.top();

            const Reference & ref = parseAsset(top);
            memory.emplace(top.ident.name, ref);

            Pane::info("Created element %s", top.ident.name.c_str());

            assets.pop();
        }

        Reference parseAsset(const AssetFrame & frame)
        {
            Reference ref;
            ref.type = frame.ident.type;
            ref.name = frame.ident.name;

            if(frame.ident.type == "Object")
            {
                ref.value = parseObject(frame);
            }
            else if(frame.ident.type == "Geometry")
            {
                ref.value = parseGeometry(frame);
            }
            else if(frame.ident.type == "Light")
            {
                ref.value = parseLight(frame);
            }
            else if(frame.ident.type == "Shader")
            {
                ref.value = parseShader(frame);
            }
            else if(frame.ident.type == "PhaseFunction")
            {
                ref.value = parsePhaseFunction(frame);
            }
            else if(frame.ident.type == "Medium")
            {
                ref.value = parseMedium(frame);
            }
            else if(frame.ident.type == "Density")
            {
                ref.value = parseDensity(frame);
            }
            else if(frame.ident.type == "Transform")
            {
                ref.value = parseTransform(frame);
            }
            else if(frame.ident.type == "Scene")
            {
                ref.value = parseScene(frame);
            }
            else if(frame.ident.type == "ObjectGroup")
            {
                ref.value = parseObjectGroup(frame);
            }
            else if(frame.ident.type == "Options")
            {
                ref.value = parseOptions(frame);
            }
            else if(frame.ident.type == "Camera")
            {
                ref.value = parseCamera(frame);
            }
            else if(frame.ident.type == "Render")
            {
                ref.value = parseRender(frame);
            }

            return ref;
        }

        std::shared_ptr<void> parseOptions(const AssetFrame & frame)
        {
            std::shared_ptr<Options> opt = 
                std::make_shared<Options>(Options::default());

            // General options
            for(auto const & prop : frame.properties)
            {
                if(prop.first == "renderer")
                {
                    if(prop.second == "raytracer")
                        opt->s_renderer = Options::RendererRayTracer;
                    else if(prop.second == "pathtracer")
                        opt->s_renderer = Options::RendererPathTracer;
                    else if(prop.second == "photonmapper")
                        opt->s_renderer = Options::RendererPhotonMapper;
                }
                else if(prop.first == "name")
                {
                    opt->s_name = prop.second;
                }
                else if(prop.first == "width")
                {
                    opt->s_width = parseInt(prop.second);
                }
                else if(prop.first == "height")
                {
                    opt->s_height = parseInt(prop.second);
                }
                else if(prop.first == "fudge")
                {
                    opt->s_fudge = parseSingle(prop.second);
                }
                else if(prop.first == "raysperpixel")
                {
                    opt->s_raysPerPixel = parseInt(prop.second);
                }
                else if(prop.first == "lightsamples")
                {
                    opt->s_lightSamples = parseInt(prop.second);
                }
                else if(prop.first == "directlighting")
                {
                    opt->s_directLighting = parseInt(prop.second);
                }
                else if(prop.first == "singlescattering")
                {
                    opt->s_singleScattering = parseInt(prop.second);
                }
                else if(prop.first == "finalgather")
                {
                    opt->s_finalGather = parseInt(prop.second);
                }
                else if(prop.first == "finalgatherrays")
                {
                    opt->s_finalGatherRays = parseInt(prop.second);
                }
                else if(prop.first == "photons")
                {
                    opt->s_photons = parseInt(prop.second);
                }
                else if(prop.first == "queryphotons")
                {
                    opt->s_queryPhotons = parseInt(prop.second);
                }
                else if(prop.first == "photondistance")
                {
                    opt->s_photonDistance = parseSingle(prop.second);
                }
                else if(prop.first == "beamestimate")
                {
                    opt->s_beamEstimate = parseSingle(prop.second);
                }
            }

            return std::static_pointer_cast<void>(opt);
        }

        std::shared_ptr<void> parseCamera(const AssetFrame & frame)
        {
            auto c = std::make_shared<Camera>();

            for(auto const & prop : frame.properties)
            {
                if(prop.first == "position")
                {
                    c->setPosition(parseVector3(prop.second));
                }
                else if(prop.first == "lookat")
                {
                    c->setLookAt(parseVector3(prop.second));
                }
                else if(prop.first == "fov")
                {
                    c->setFov(parseSingle(prop.second));
                }
            }

            return std::static_pointer_cast<void>(c);
        }

        std::shared_ptr<void> parseRender(const AssetFrame & frame)
        {
            auto r = std::make_shared<Render>();

            for(auto const & ref : frame.references)
            {
                if(ref.type == "Scene")
                    r->scene = asset<Scene>(ref.name);
                else if(ref.type == "Options")
                    r->options = asset<Options>(ref.name);
                else if(ref.type == "Camera")
                    r->camera = asset<Camera>(ref.name);
            }

            return std::static_pointer_cast<void>(r);
        }

        std::shared_ptr<void> parseScene(const AssetFrame & frame)
        {
            auto s = std::make_shared<Scene>();

            auto selector = [&s, this] (const Reference & ref)
                {
                    if(ref.type == "Object")
                        s->addObject(asset<Object>(ref.name));
                    else if(ref.type == "Light")
                        s->addLight(asset<Light>(ref.name));
                    else if(ref.type == "Medium")
                        s->addMedium(asset<Medium>(ref.name));
                };
            
            for(auto const & ref : frame.references)
            {
                if (ref.type == "ObjectGroup")
                {
                    auto const & group = *asset<std::vector<Reference>>(ref.name);
                    for(auto const & ref : group)
                        selector(ref);
                }
                else
                {
                    selector(ref);
                }
            }

            return std::static_pointer_cast<void>(s);
        }

        std::shared_ptr<void> parseObjectGroup(const AssetFrame & frame)
        {
            return std::make_shared<std::vector<Reference>>(frame.references.begin(), frame.references.end());
        }

        std::shared_ptr<void> parseObject(const AssetFrame & frame)
        {
            std::shared_ptr<Transform> transform = nullptr;
            std::shared_ptr<Geometry> geometry = nullptr;
            std::shared_ptr<Shader> shader = nullptr;

            for(auto const & ref : frame.references)
            {
                if(ref.type == "Transform")
                    transform = asset<Transform>(ref.name);
                else if(ref.type == "Geometry")
                    geometry = asset<Geometry>(ref.name);
                else if(ref.type == "Shader")
                    shader = asset<Shader>(ref.name);
            }

            if(!transform)
            {
                Pane::warning("Transform not specified. Using identity", frame.ident.name);
                transform = std::make_shared<Transform>();
            }
            if(!geometry)
            {
                Pane::error("Geometry not specified. Cannot construct asset %s", frame.ident.name);
                return nullptr;
            }
            if(!shader)
            {
                Pane::error("Shader not specified. Cannot construct asset %s", frame.ident.name);
                return nullptr;
            }

            return std::shared_ptr<void>(new Object(*transform, *geometry, *shader));
        }

        std::shared_ptr<void> parseTransform(const AssetFrame & frame)
        {
            auto t = std::make_shared<Transform>();
            
            for (auto const & prop : frame.properties)
            {
                if(prop.first == "move")
                    t->setPosition(parseVector3(prop.second));
                else if (prop.first == "rotate")
                    t->setRotation(parseQuat(prop.second));
                else if (prop.first == "scale")
                    t->setScale(parseVector3(prop.second));
            }

            return std::static_pointer_cast<void>(t);
        }

        std::shared_ptr<void> parseGeometry(const AssetFrame & frame)
        {
            const std::string & type = frame.properties.at("type");

            if(type.empty())
            {
                Pane::error("No type for geometry");
            }
            else if (type == "sphere")
            {
                return parseSphere(frame);
            }
            else if (type == "plane")
            {
                return parsePlane(frame);
            }
            else if (type == "mesh")
            {
                return parseMesh(frame);
            }
            else if (type == "triangle")
            {
                return parseTriangle(frame);
            }

            return nullptr;
        }

        std::shared_ptr<void> parseSphere(const AssetFrame & frame)
        {
            const glm::vec3 center = parseVector3(frame.properties.at("center"));
            const real radius = parseSingle(frame.properties.at("radius"));

            return std::shared_ptr<void>(new Sphere(center, radius));
        }

        std::shared_ptr<void> parsePlane(const AssetFrame & frame)
        {
            const glm::vec3 n = parseVector3(frame.properties.at("n"));
            const real d = parseSingle(frame.properties.at("d"));

            return std::shared_ptr<void>(new Plane(n, d));
        }

        std::shared_ptr<void> parseMesh(const AssetFrame & frame)
        {
            const std::string file = frame.properties.at("file");
            const int flags = parseInt(frame.properties.at("flags"));

            MeshPtr mesh = MeshImporter::load(file, flags);

            return std::shared_ptr<void>(new MeshGeometry(mesh));
        }

        std::shared_ptr<void> parseTriangle(const AssetFrame & frame)
        {
            const glm::vec3 & v1 = parseVector3(frame.properties.at("v1"));
            const glm::vec3 & v2 = parseVector3(frame.properties.at("v2"));
            const glm::vec3 & v3 = parseVector3(frame.properties.at("v3"));

            const std::array<glm::vec3, 3> verts = { v1, v2, v3 };

            return std::shared_ptr<void>(new Triangle(verts));
        }

        std::shared_ptr<void> parseLight(const AssetFrame & frame)
        {
            const std::string & type = frame.properties.at("type");

            if(type.empty())
            {
                Pane::error("No type for light");
            }
            else if (type == "point")
            {
                return parsePointLight(frame);
            }
            else if (type == "area")
            {
                return parseAreaLight(frame);
            }
            else if (type == "directional")
            {
                return parseDirectionalLight(frame);
            }

            return nullptr;
        }

        std::shared_ptr<void> parsePointLight(const AssetFrame & frame)
        {
            const glm::vec3 center = parseVector3(frame.properties.at("center"));
            const glm::vec3 intensity = parseVector3(frame.properties.at("intensity"));

            return std::shared_ptr<void>(new PointLight(center, intensity));
        }

        std::shared_ptr<void> parseAreaLight(const AssetFrame & frame)
        {
            const glm::vec3 center = parseVector3(frame.properties.at("center"));
            const glm::vec3 extent0 = parseVector3(frame.properties.at("extent0"));
            const glm::vec3 extent1 = parseVector3(frame.properties.at("extent1"));
            const glm::vec3 intensity = parseVector3(frame.properties.at("intensity"));

            return std::shared_ptr<void>(new PlanarLight(center, extent0, extent1, intensity));
        }

        std::shared_ptr<void> parseDirectionalLight(const AssetFrame & frame)
        {
            const glm::vec3 direction = parseVector3(frame.properties.at("direction"));
            const glm::vec3 intensity = parseVector3(frame.properties.at("intensity"));

            return std::shared_ptr<void>(new DirectionalLight(direction, intensity));
        }

        std::shared_ptr<void> parseShader(const AssetFrame & frame)
        {
            const std::string & shader = frame.properties.at("shader");
            if(shader.empty())
                Pane::error("No shader for material");
            else if(shader == "phong")
                return parsePhong(frame);
            else if(shader == "lambert")
                return parseLambert(frame);
            else if(shader == "refract")
                return parseRefract(frame);
            else if(shader == "reflect")
                return parseReflect(frame);
            else if(shader == "emissive")
                return parseEmissive(frame);

            return nullptr;
        }

        std::shared_ptr<void> parsePhong(const AssetFrame & frame)
        {
            const glm::vec3 diffuse = parseVector3(frame.properties.at("diffuse"));
            const glm::vec3 specular = parseVector3(frame.properties.at("specular"));
            const real shininess = parseSingle(frame.properties.at("shininess"));

            return std::shared_ptr<void>(new PhongShader(diffuse, specular, shininess));
        }

        std::shared_ptr<void> parseLambert(const AssetFrame & frame)
        {
            const glm::vec3 diffuse = parseVector3(frame.properties.at("diffuse"));

            return std::shared_ptr<void>(new LambertShader(diffuse));
        }

        std::shared_ptr<void> parseRefract(const AssetFrame & frame)
        {
            const real index = parseSingle(frame.properties.at("index"));
            const glm::vec3 reflectance = parseVector3(frame.properties.at("reflectance"));
            const glm::vec3 transmittance = parseVector3(frame.properties.at("transmittance"));

            return std::shared_ptr<void>(new RefractShader(index, reflectance, transmittance));
        }

        std::shared_ptr<void> parseReflect(const AssetFrame & frame)
        {
            const glm::vec3 reflectance = parseVector3(frame.properties.at("reflectance"));

            return std::shared_ptr<void>(new ReflectShader(reflectance));
        }

        std::shared_ptr<void> parseEmissive(const AssetFrame & frame)
        {
            const glm::vec3 intensity = parseVector3(frame.properties.at("intensity"));

            return std::shared_ptr<void>(new EmissiveShader(intensity));
        }

        std::shared_ptr<void> parsePhaseFunction(const AssetFrame & frame)
        {
            const std::string & function = frame.properties.at("function");
            if(function == "isotropic")
                return parseIsotropic(frame);
            else if(function == "hg")
                return parseHG(frame);

            Pane::error("Not a valid phase function: %s", function);
            return nullptr;
        }

        std::shared_ptr<void> parseIsotropic(const AssetFrame & frame)
        {
            return std::shared_ptr<void>(new IsotropicPhaseFunction());
        }

        std::shared_ptr<void> parseHG(const AssetFrame & frame)
        {
            const real g = parseSingle(frame.properties.at("g"));

            return std::shared_ptr<void>(new HGPhaseFunction(g));
        }

        std::shared_ptr<void> parseMedium(const AssetFrame & frame)
        {
            std::shared_ptr<Geometry> geometry = nullptr;
            std::shared_ptr<PhaseFunction> phaseFunction = nullptr;
            std::shared_ptr<Density> density = nullptr;

            for(auto const & ref : frame.references)
            {
                if(ref.type == "Geometry")
                    geometry = asset<Geometry>(ref.name);
                else if(ref.type == "PhaseFunction")
                    phaseFunction = asset<PhaseFunction>(ref.name);
                else if(ref.type == "Density")
                    density = asset<Density>(ref.name);
            }

            if(!geometry)
            {
                Pane::error("Geometry not specified. Cannot construct asset %s", frame.ident.name);
                return nullptr;
            }
            if(!phaseFunction)
            {
                Pane::error("Phase function not specified. Cannot construct asset %s", frame.ident.name);
                return nullptr;
            }

            const real sigma_a = parseSingle(frame.properties.at("a"));
            const real sigma_s = parseSingle(frame.properties.at("s"));
            const glm::vec3 emission = parseVector3(frame.properties.at("emission"));

            if(density)
                return std::shared_ptr<void>(new DensityMedium(*geometry, *phaseFunction, *density, sigma_a, sigma_s, emission));
            else
                return std::shared_ptr<void>(new HomogeneousMedium(*geometry, *phaseFunction, sigma_a, sigma_s, emission));
        }

        std::shared_ptr<void> parseDensity(const AssetFrame & frame)
        {
            const std::string & type = frame.properties.at("type");
            if(type == "noise")
            {
                const real size = parseSingle(frame.properties.at("size"));
                return std::shared_ptr<void>(new NoiseDensity(size));
            }

            Pane::error("Not a valid density: %s", type);
            return nullptr;
        }

        std::stack<AssetFrame> assets;
        std::map<std::string, Reference> memory;
    };

    ////////////////////////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////////////////////////

    ConfigurationParser::ConfigurationParser()
        : m_d(new D)
    {}

    ConfigurationParser::~ConfigurationParser()
    {
        delete m_d;
    }

    void ConfigurationParser::parse(const std::string & filename)
    {
        m_d->parseFile(filename);
    }

    void ConfigurationParser::dumpFile(const std::string & filename)
    {
        m_d->dumpFile(filename);
    }

    void ConfigurationParser::clear()
    {
        m_d->clearMemory();
    }

    OptionsPtr ConfigurationParser::getOptions()
    {
        auto const & r = m_d->asset<Render>("render");
        return r->options;
    }
    
    ScenePtr ConfigurationParser::getScene()
    {
        auto const & r = m_d->asset<Render>("render");
        return r->scene;
    }

    CameraPtr ConfigurationParser::getCamera()
    {
        auto const & r = m_d->asset<Render>("render");
        return r->camera;
    }
}
