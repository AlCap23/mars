/*
 *  Copyright 2016, DFKI GmbH Robotics Innovation Center
 *
 *  This file is part of the MARS simulation framework.
 *
 *  MARS is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU Lesser General Public License
 *  as published by the Free Software Foundation, either version 3
 *  of the License, or (at your option) any later version.
 *
 *  MARS is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Lesser General Public License for more details.
 *
 *   You should have received a copy of the GNU Lesser General Public License
 *   along with MARS.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

/*
 *  OsgMaterial.cpp
 *  General OsgMaterial to inherit from.
 *
 *  Created by Langosz on 2016
 */

#include <mars/utils/misc.h>
#include "OsgMaterial.h"
#include "OsgMaterialManager.h"
#include "MaterialNode.h"

#include "shader/shader-generator.h"
#include "shader/shader-function.h"
#include "shader/bumpmapping.h"
#include "shader/pixellight.h"

#include <osg/TexMat>
#include <osg/CullFace>

namespace osg_material_manager {

  using namespace std;
  using namespace mars::utils;
  using namespace configmaps;

  OsgMaterial::OsgMaterial(std::string resPath)
    : material(0),
      hasShaderSources(false),
      useShader(true),
      maxNumLights(1),
      resPath(resPath),
      invShadowTextureSize(1./1024),
      useWorldTexCoords(false) {
    noiseMapUniform = new osg::Uniform("NoiseMap", NOISE_MAP_UNIT);
    texScaleUniform = new osg::Uniform("texScale", 1.0f);
    sinUniform = new osg::Uniform("sin_", 0.0f);
    cosUniform = new osg::Uniform("cos_", 1.0f);
    shadowScaleUniform = new osg::Uniform("shadowScale", 0.5f);
    bumpNorFacUniform = new osg::Uniform("bumpNorFac", 1.0f);
    shadowSamplesUniform = new osg::Uniform("shadowSamples", 1);
    invShadowSamplesUniform = new osg::Uniform("invShadowSamples",
                                               1.f/1);
    invShadowTextureSizeUniform = new osg::Uniform("invShadowTextureSize",
                                                   (float)(invShadowTextureSize));

    noiseMap = new osg::Texture2D();
    noiseMap->setDataVariance(osg::Object::DYNAMIC);
    noiseMap->setWrap(osg::Texture::WRAP_S, osg::Texture::REPEAT);
    noiseMap->setWrap(osg::Texture::WRAP_T, osg::Texture::REPEAT);
    noiseMap->setWrap(osg::Texture::WRAP_R, osg::Texture::REPEAT);
    noiseMap->setFilter(osg::Texture::MIN_FILTER, osg::Texture::NEAREST);
    noiseMap->setFilter(osg::Texture::MAG_FILTER, osg::Texture::NEAREST);
    unitMap["diffuseMap"] = 0;
    unitMap["normalMap"] = 1;
    unitMap["displacementMap"] = 3;
    unitMap["environmentMap"] = 0;
    unitMap["envMapR"] = 5;
    unitMap["envMapG"] = 6;
  }

  OsgMaterial::~OsgMaterial() {
  }

  osg::Vec4 OsgMaterial::getColor(string key) {
    osg::Vec4 c(0, 0, 0, 1);
    if(map.hasKey(key)) {
      ConfigMap &m = map[key];
      c[0] = m.get("r", 0.0);
      c[1] = m.get("g", 0.0);
      c[2] = m.get("b", 0.0);
      c[3] = m.get("a", 1.0);
    }
    return c;
  }
  // the material struct can also contain a static texture (texture file)
  void OsgMaterial::setMaterial(const ConfigMap &map_) {
    //return;
    map = map_;
    name << map["name"];
    getLight = map.get("getLight", true);

    // create the osg::Material
    material = new osg::Material();
    material->setColorMode(osg::Material::OFF);

    material->setAmbient(osg::Material::FRONT_AND_BACK, getColor("ambientColor"));
    material->setSpecular(osg::Material::FRONT_AND_BACK, getColor("specularColor"));
    material->setDiffuse(osg::Material::FRONT_AND_BACK, getColor("diffuseColor"));
    material->setEmission(osg::Material::FRONT_AND_BACK, getColor("emissionColor"));
    material->setShininess(osg::Material::FRONT_AND_BACK, map.get("shininess", 0.0));
    material->setTransparency(osg::Material::FRONT_AND_BACK, map.get("transparency", 0.0));

    // get the StateSet of the Object
    osg::StateSet *state = getOrCreateStateSet();

    // set the material
    state->setAttributeAndModes(material.get(), osg::StateAttribute::ON);

    if(!getLight) {
      osg::ref_ptr<osg::CullFace> cull = new osg::CullFace();
      cull->setMode(osg::CullFace::BACK);
      state->setAttributeAndModes(cull.get(), osg::StateAttribute::OFF);
      state->setMode(GL_LIGHTING,
                     osg::StateAttribute::OFF);
      state->setMode(GL_FOG, osg::StateAttribute::OFF);
    }

    float transparency = (float)map.get("transparency", 0.0);
    string texturename = map.get("diffuseTexture", std::string());
    double tex_scale = map.get("tex_scale", 1.0);
    texScaleUniform->set((float)tex_scale);

    //disable all textures
    std::map<std::string, TextureInfo>::iterator it = textures.begin();
    for(; it!=textures.end(); ++it) {
      it->second.enabled = false;
      state->setTextureAttributeAndModes(it->second.unit,
                                         it->second.texture,
                                         osg::StateAttribute::OFF);
    }
    if (!texturename.empty()) {
      ConfigMap config;
      config["name"] = "diffuseMap";
      config["file"] = texturename;
      config["texScale"] = tex_scale;
      addTexture(config, map.hasKey("instancing"));
    }

    bool generateTangents = false;
    texturename = map.get("normalTexture", std::string());
    if (!texturename.empty()) {
      generateTangents = true;
      ConfigMap config;
      config["name"] = "normalMap";
      config["file"] = texturename;
      config["texScale"] = tex_scale;
      addTexture(config, map.hasKey("instancing"));
    }
    bumpNorFacUniform->set((float)map.get("bumpNorFac", 1.0));

    if(map.hasKey("textures")) {
      ConfigVector::iterator it = map["textures"].begin();
      for(; it!=map["textures"].end(); ++it) {
        addTexture(*it, map.hasKey("instancing"));
      }
    }
    useWorldTexCoords = map.get("useWorldTexCoords", false);
    updateShader(true);

    {
      std::vector<osg::ref_ptr<MaterialNode> >::iterator it = materialNodeVector.begin();
      for(; it!=materialNodeVector.end(); ++it) {
        if(generateTangents) {
          (*it)->setNeedTangents(true);
          (*it)->generateTangents();
        }
        (*it)->setTransparency(transparency);
      }
    }
  }

  void OsgMaterial::addTexture(ConfigMap &config, bool nearest) {
    osg::StateSet *state = getOrCreateStateSet();
    std::map<std::string, TextureInfo>::iterator it = textures.find((std::string)config["name"]);
    if(it != textures.end()) {
      TextureInfo &info = it->second;
      // todo: handle changes in texture unit etc.
      info.texture = OsgMaterialManager::loadTexture((std::string)config["file"]);
      if(!info.enabled) {
        state->setTextureAttributeAndModes(info.unit, info.texture,
                                           osg::StateAttribute::ON);
        state->addUniform(info.textureUniform.get());
        info.enabled = true;
      }
    }
    else {
      TextureInfo info;
      info.texture = OsgMaterialManager::loadTexture((std::string)config["file"]);
      if(nearest) {
        info.texture->setFilter(osg::Texture::MIN_FILTER,
                                osg::Texture::NEAREST);
        info.texture->setFilter(osg::Texture::MAG_FILTER,
                                osg::Texture::NEAREST);
      }
      else {
        info.texture->setFilter(osg::Texture::MIN_FILTER,
                                osg::Texture::LINEAR_MIPMAP_LINEAR);
        info.texture->setFilter(osg::Texture::MAG_FILTER,
                                osg::Texture::LINEAR);
      }
      info.texture->setWrap( osg::Texture::WRAP_S, osg::Texture::REPEAT );
      info.texture->setWrap( osg::Texture::WRAP_T, osg::Texture::REPEAT );
      info.texture->setMaxAnisotropy(8);
      info.name << config["name"];
      info.unit = 0;
      if(unitMap.hasKey(info.name)) {
        info.unit = unitMap[info.name];
      }
      if(config.hasKey("unit")) {
        info.unit = config["unit"];
      }
      fprintf(stderr, "add Texture: %s %d\n", info.name.c_str(), info.unit);
      info.textureUniform = new osg::Uniform(info.name.c_str(), info.unit);
      state->setTextureAttributeAndModes(info.unit, info.texture,
                                         osg::StateAttribute::ON);
      state->addUniform(info.textureUniform.get());
      if(config.hasKey("texScale") && (double)config["texScale"] != 1.0) {
        osg::ref_ptr<osg::TexMat> scaleTexture = new osg::TexMat();
        float tex_scale = (double)config["texScale"];
        scaleTexture->setMatrix(osg::Matrix::scale(tex_scale, tex_scale,
                                                   tex_scale));
        state->setTextureAttributeAndModes(info.unit, scaleTexture.get(),
                                           osg::StateAttribute::ON);
      }
      info.enabled = true;
      textures[info.name] = info;
    }
  }

  void OsgMaterial::disableTexture(std::string name) {
    std::map<std::string, TextureInfo>::iterator it = textures.find(name);
    if(it != textures.end()) {
      osg::StateSet *state = getOrCreateStateSet();
      it->second.enabled = false;
      state->setTextureAttributeAndModes(it->second.unit,
                                         it->second.texture,
                                         osg::StateAttribute::OFF);
      state->removeUniform(it->second.textureUniform);
    }
  }

  void OsgMaterial::enableTexture(std::string name) {
    std::map<std::string, TextureInfo>::iterator it = textures.find(name);
    if(it != textures.end()) {
      osg::StateSet *state = getOrCreateStateSet();
      it->second.enabled = true;
      state->setTextureAttributeAndModes(it->second.unit,
                                         it->second.texture,
                                         osg::StateAttribute::ON);
      state->addUniform(it->second.textureUniform);
    }
  }

  bool OsgMaterial::checkTexture(std::string name) {
    std::map<std::string, TextureInfo>::iterator it = textures.find(name);
    if(it != textures.end()) {
      return it->second.enabled;
    }
    return false;
  }

  void OsgMaterial::setColor(string color, string key, string value) {
    double v = atof(value.c_str());
    if(key[key.size()-1] == 'a') map[color]["a"] = v;
    else if(key[key.size()-1] == 'r') map[color]["r"] = v;
    else if(key[key.size()-1] == 'g') map[color]["g"] = v;
    else if(key[key.size()-1] == 'b') map[color]["b"] = v;
    setMaterial(map);
  }

  void OsgMaterial::edit(const std::string &key, const std::string &value) {
    if(matchPattern("*/ambientColor/*", key) ||
       matchPattern("*/ambientFront/*", key)) {
      setColor("ambientColor", key, value);
    }
    else if(matchPattern("*/diffuseColor/*", key) ||
            matchPattern("*/diffuseFront/*", key)) {
      setColor("diffuseColor", key, value);
    }
    if(matchPattern("*/specularColor/*", key) ||
       matchPattern("*/specularFront/*", key)) {
      setColor("specularColor", key, value);
    }
    if(matchPattern("*/emissionColor/*", key) ||
       matchPattern("*/emissionFront/*", key)) {
      setColor("emissionColor", key, value);
    }

    if(matchPattern("*/diffuseTexture", key) ||
       matchPattern("*/texturename", key)) {
      if(value == "") {
        map["diffuseTexture"] = string();
        fprintf(stderr, "edit material: %s\n", map["diffuseTexture"].c_str());
      }
      else if(pathExists(value)) {
        map["diffuseTexture"] = value;
        fprintf(stderr, "edit material: %s\n", map["diffuseTexture"].c_str());
      }
      setMaterial(map);
    }
    if(matchPattern("*/normalTexture", key) ||
       matchPattern("*/bumpmap", key)) {
      if(value == "") {
        map["normalTexture"] = string();
        fprintf(stderr, "edit material: %s\n", map["normalTexture"].c_str());
      }
      else if(pathExists(value)) {
        map["normalTexture"] = value;
        fprintf(stderr, "edit material: %s\n", map["normalTexture"].c_str());
      }
      setMaterial(map);
    }
    if(matchPattern("*/displacementTexture", key) ||
       matchPattern("*/displacementmap", key)) {
      if(value == "") {
        map["displacementTexture"] = string();
      }
      else if(pathExists(value)) {
        map["displacementTexture"] = value;
      }
      setMaterial(map);
    }
    if(matchPattern("*/bumpNorFac", key)) {
      map["bumpNorFac"] = atof(value.c_str());
      setMaterial(map);
    }
    if(matchPattern("*/shininess", key)) {
      map["shininess"] = atof(value.c_str());
      setMaterial(map);
    }
    if(matchPattern("*/transparency", key)) {
      map["transparency"] = atof(value.c_str());
      setMaterial(map);
    }
    if(matchPattern("*/tex_scale", key)) {
      map["tex_scale"] = atof(value.c_str());
      setMaterial(map);
    }
  }

  void OsgMaterial::setTexture(osg::Texture2D *texture) {
    if(textures.find("diffuseMap") != textures.end()) {
      TextureInfo &info = textures["diffuseMap"];
      info.texture = texture;
      if(!info.enabled) {
        osg::StateSet *state = getOrCreateStateSet();
        state->setTextureAttributeAndModes(info.unit, info.texture,
                                           osg::StateAttribute::ON);
      }
    }
  }

  void OsgMaterial::setBumpMap(const std::string &filename) {
    fprintf(stderr, "OsgMaterial: setBumpMap is deprecated use addTexture instead");
  }

  void OsgMaterial::setNormalMap(const std::string &filename) {
    fprintf(stderr, "OsgMaterial: setNormalMap is deprecated use addTexture instead");
  }

  void OsgMaterial::setUseShader(bool val) {
    fprintf(stderr, "use shader: %d %d\n", useShader, val);
    if(useShader != val) {
      useShader = val;
      updateShader(true);
    }
  }

  void OsgMaterial::setShadowScale(float v) {
    shadowScaleUniform->set(1.f/(v*v));
  }

  void OsgMaterial::updateShader(bool reload) {
    osg::StateSet* stateSet = getOrCreateStateSet();

    //return;
    if(!useShader || !getLight) {
      if(lastProgram.valid()) {
        stateSet->removeAttribute(lastProgram.get());
        lastProgram = NULL;
      }
      disableTexture("normalMap");
      stateSet->setTextureAttributeAndModes(NOISE_MAP_UNIT, noiseMap,
                                            osg::StateAttribute::OFF);
      return;
    }
    enableTexture("normalMap");

    if(!reload && hasShaderSources) {
      // no need to regenerate, shader source did not changed
      return;
    }
    hasShaderSources = true;
    stateSet->setTextureAttributeAndModes(NOISE_MAP_UNIT, noiseMap,
                                          osg::StateAttribute::ON);
    ShaderGenerator shaderGenerator;
    vector<string> args;

    bool hasTexture = checkTexture("environmentMap") || checkTexture("diffuseMap") || checkTexture("normalMap");

    ShaderFunc *vertexShader = new ShaderFunc;
    {
      if(map["instancing"]) {
        vertexShader->addUniform( (GLSLUniform)
                                  { "sampler2D", "NoiseMap" } );
        vertexShader->enableExtension("GL_ARB_draw_instanced");
        vertexShader->addUniform( (GLSLUniform)
                                  { "float", "sin_" } );
        vertexShader->addUniform( (GLSLUniform)
                                  { "float", "cos_" } );
        vertexShader->addMainVar( (GLSLVariable)
                                  { "vec4", "offset", "vec4(4.0*rnd(float(gl_InstanceIDARB)*0.1, float(gl_InstanceIDARB)*0.2), 4.0*rnd(float(gl_InstanceIDARB)*0.2, float(gl_InstanceIDARB)*0.3), 0, 0)" });
        vertexShader->addMainVar( (GLSLVariable)
                                  { "vec4", "fPos", "gl_Vertex + offset" });
        vertexShader->addMainVar( (GLSLVariable)
                                  { "vec3", "sc", "vec3(normalize(vec2(fPos.x*0.1+0.5*rnd(float(gl_InstanceIDARB)*0.16, float(gl_InstanceIDARB)*0.8), fPos.y*0.1+0.5*rnd(float(gl_InstanceIDARB)*0.8, float(gl_InstanceIDARB)*0.16))), rnd(float(gl_InstanceIDARB)*0.4, float(gl_InstanceIDARB)*0.4))-0.5" });
        vertexShader->addMainVar( (GLSLVariable)
                                  { "vec4", "vPos", "fPos + vec4(sc.z*(sin_*gl_Vertex.z*sc.x + cos_*gl_Vertex.z*sc.y), sc.z*(sin_*gl_Vertex.z*sc.y + cos_*gl_Vertex.z*sc.x), 0, 0)" });
        vertexShader->addExport( (GLSLExport)
                                 {"gl_TexCoord[2].xy", "gl_TexCoord[2].xy + vec2(-offset.y, offset.x)"} );
      }
      else {
        vertexShader->addMainVar( (GLSLVariable)
                                  { "vec4", "vPos", "gl_Vertex " });
      }
      vertexShader->addMainVar( (GLSLVariable)
                                { "vec4", "scol", "gl_FrontMaterial.specular" });
      vertexShader->addExport( (GLSLExport)
                               {"gl_Position", "gl_ModelViewProjectionMatrix * vPos"} );
      vertexShader->addExport( (GLSLExport) {"gl_ClipVertex", "v1"} );
      if(hasTexture) {
        vertexShader->addExport( (GLSLExport)
                                 { "gl_TexCoord[0].xy", "gl_MultiTexCoord0.xy" });
      }
      shaderGenerator.addShaderFunction(vertexShader, SHADER_TYPE_VERTEX);
    }

    ShaderFunc *fragmentShader = new ShaderFunc;
    {
      if(hasTexture) {
        fragmentShader->addUniform( (GLSLUniform)
                                    { "float", "texScale" } );
        {
          if(useWorldTexCoords) {
            fragmentShader->addMainVar( (GLSLVariable)
                                        { "vec2", "texCoord", "positionVarying.xy*texScale" });
          }
          else {
            fragmentShader->addMainVar( (GLSLVariable)
                                        { "vec2", "texCoord", "gl_TexCoord[0].xy*texScale" });
          }
        }
      }
      std::map<std::string, TextureInfo>::iterator it = textures.begin();
      for(; it!=textures.end(); ++it) {
        fragmentShader->addUniform( (GLSLUniform)
                                    { "sampler2D", it->second.name } );
      }
      {
        if(map["insetancing"]) {
          fragmentShader->addMainVar( (GLSLVariable)
                                      { "vec4", "col", "vec4(1.0, 1.0, 1.0, texture2D(normalMap, texCoord).a)" });
        }
        else {
          fragmentShader->addMainVar( (GLSLVariable)
                                      { "vec4", "col", "vec4(1.0)" });
        }
      }
      fragmentShader->addExport( (GLSLExport) {"gl_FragColor", "col"} );
      shaderGenerator.addShaderFunction(fragmentShader, SHADER_TYPE_FRAGMENT);
    }


    osg::Program *glslProgram;
    args.clear();
    if(!map.hasKey("shader")) {
      map["shader"]["PixelLightVertex"] = 1;
      map["shader"]["PixelLightFragment"] = 1;
      if(checkTexture("normalMap")) {
        map["shader"]["NormalMapVertex"] = 1;
        map["shader"]["NormalMapFragment"] = 1;
      }
    }
    if(map.hasKey("shader")) {
      bool havePCol = false;
      if(map["shader"].hasKey("PixelLightVertex")) {
        PixelLightVert *plightVert = new PixelLightVert(args, maxNumLights,
                                                        resPath);
        shaderGenerator.addShaderFunction(plightVert, SHADER_TYPE_VERTEX);
      }
      if(map["shader"].hasKey("NormalMapVertex")) {
        BumpMapVert *bumpVert = new BumpMapVert(args, resPath);
        shaderGenerator.addShaderFunction(bumpVert, SHADER_TYPE_VERTEX);
      }
      if(map["shader"].hasKey("NormalMapFragment")) {
        BumpMapFrag *bumpFrag = new BumpMapFrag(args, resPath);
        shaderGenerator.addShaderFunction(bumpFrag, SHADER_TYPE_FRAGMENT);
      }

      if(map["shader"].hasKey("EnvMapVertex")) {
        vertexShader->addUniform( (GLSLUniform) { "sampler2D", "envMapG" } );
        vertexShader->addUniform( (GLSLUniform) { "sampler2D", "envMapR" } );
        vertexShader->addUniform( (GLSLUniform) { "sampler2D", "environmentMap" } );

        vertexShader->addUniform( (GLSLUniform)
                                  { "float", "texScale" } );
        vertexShader->addMainVar( (GLSLVariable) { "vec4", "scale",
              "texture2D(environmentMap, texCoord)" }, 1);
        vertexShader->addMainVar( (GLSLVariable) { "float", "gcol",
              "scale.g*(texture2D(envMapG, texCoord.xy*150.0).a)*0" }, 2);
        vertexShader->addMainVar( (GLSLVariable) { "float", "rcol",
              "scale.r*(texture2D(envMapR, texCoord.xy*450.).a)" }, 3);
        vertexShader->addMainVar( (GLSLVariable)
                                  { "vec2", "texCoord", "gl_MultiTexCoord0.xy*texScale" }, true);
        vertexShader->addMainVar( (GLSLVariable)
                                  { "vec4", "scol", "vec4(gl_FrontMaterial.specular.rgb*rcol+gl_FrontMaterial.specular.rgb*gcol, 1)" });

      }
      if(map["shader"].hasKey("EnvMapFragment")) {
        havePCol = true;
        fragmentShader->addMainVar( (GLSLVariable) { "vec4", "scale",
              "texture2D(environmentMap, texCoord)" }, 2);
        fragmentShader->addMainVar( (GLSLVariable) { "vec4", "nt_",
              "texture2D( normalMap, texCoord*450. )" }, 3);
        fragmentShader->addMainVar( (GLSLVariable) { "vec4", "rcol",
              "texture2D(envMapR, texCoord.xy*450.)" });
        fragmentShader->addMainVar( (GLSLVariable) { "vec4", "gcol",
              "texture2D(envMapG, texCoord.xy*150.0)" });
        //fragmentShader->addMainVar( (GLSLVariable) { "vec4", "pcol",
        //      "vec4(scale.r*rcol.r,scale.r*rcol.g,scale.r*rcol.b, 1.0)" });
        fragmentShader->addMainVar( (GLSLVariable) { "vec4", "pcol",
              "vec4(scale.r*rcol.rgb+scale.g*gcol.rgb, 1.0)" });
        fragmentShader->addMainVar( (GLSLVariable) { "vec4", "nt",
              "vec4(scale.r*nt_.xy, (1-scale.r)*nt_.z, 1)" });

      }
      if(map["shader"].hasKey("PixelLightFragment")) {
        bool haveDiffuseMap = checkTexture("diffuseMap");
        PixelLightFrag *plightFrag = new PixelLightFrag(args, maxNumLights,
                                                        resPath,
                                                        haveDiffuseMap,
                                                        havePCol);
        // invert the normal if gl_FrontFacing=true to handle back faces
        // correctly.
        // TODO: check not needed if backfaces not processed.
        shaderGenerator.addShaderFunction(plightFrag, SHADER_TYPE_FRAGMENT);
      }
    }

    glslProgram = shaderGenerator.generate();
    if(checkTexture("normalMap")) {
      glslProgram->addBindAttribLocation( "vertexTangent", TANGENT_UNIT );
      stateSet->addUniform(bumpNorFacUniform.get());
    }
    else {
      stateSet->removeUniform(bumpNorFacUniform.get());
    }
    stateSet->addUniform(noiseMapUniform.get());

    if(hasTexture) {
      stateSet->addUniform(texScaleUniform.get());
      stateSet->addUniform(sinUniform.get());
      stateSet->addUniform(cosUniform.get());
    }
    else {
      stateSet->removeUniform(texScaleUniform.get());
    }

    if(lastProgram.valid()) {
      stateSet->removeAttribute(lastProgram.get());
    }
    stateSet->setAttributeAndModes(glslProgram,
                                   osg::StateAttribute::ON);

    stateSet->removeUniform(shadowSamplesUniform.get());
    stateSet->removeUniform(invShadowSamplesUniform.get());
    stateSet->removeUniform(invShadowTextureSizeUniform.get());
    stateSet->removeUniform(shadowScaleUniform.get());

    stateSet->addUniform(shadowSamplesUniform.get());
    stateSet->addUniform(invShadowSamplesUniform.get());
    stateSet->addUniform(invShadowTextureSizeUniform.get());
    stateSet->addUniform(shadowScaleUniform.get());

    lastProgram = glslProgram;
  }

  void OsgMaterial::setNoiseImage(osg::Image *i) {
    noiseMap->setImage(i);
  }

  void OsgMaterial::update() {
    t += 0.04;
    if(t > 6.28) t -= 6.28;
    sinUniform->set((float)(sin(t)*0.5));
    cosUniform->set((float)(cos(t)*0.75));
  }

  void OsgMaterial::setShadowSamples(int v) {
    shadowSamplesUniform->set(v);
    invShadowSamplesUniform->set(1.f/(v*v));
  }

  void OsgMaterial::removeMaterialNode(MaterialNode* d) {
    std::vector<osg::ref_ptr<MaterialNode> >::iterator it;
    for(it=materialNodeVector.begin(); it!=materialNodeVector.end(); ++it) {
      if(it->get() == d) {
        materialNodeVector.erase(it);
        return;
      }
    }
  }

  void OsgMaterial::setShadowTextureSize(int size) {
    invShadowTextureSize = 1./size;
    invShadowTextureSizeUniform->set((float)invShadowTextureSize);
  }

  void OsgMaterial::addMaterialNode(MaterialNode *d) {
    materialNodeVector.push_back(d);
    if(checkTexture("normalMap") || checkTexture("displacementMap")) {
      d->setNeedTangents(true);
    }
    d->setTransparency((float)map.get("transparency", 0.0));
  }

  void OsgMaterial::setMaxNumLights(int n) {
    bool needUpdate = (maxNumLights != n);
    maxNumLights = n;
    if(needUpdate) updateShader(true);
  }

} // end of namespace osg_material_manager
