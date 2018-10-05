/*
 *  Copyright 2018 DFKI GmbH Robotics Innovation Center
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

#include "FMUData.h"
#include "sim/LoadCenter.h"

#include <mars/utils/misc.h>

#define GET_VALUE(str, val, type)                  \
    if ((it = config->find(str)) != config->end()) \
    {                                              \
        val = it->second;                          \
    }

namespace mars
{
namespace interfaces
{
using namespace configmaps;
FMUData::FMUData(const std::string &name)
{
    this->name = name;
    this->max_step_size = -1;
    this->step_size = -1;
    this->relativeTolerance = -1;
};
bool FMUData::fromConfigMap(configmaps::ConfigMap *config, std::string filenamePrefix,
                            LoadCenter *loadCenter)
{
    CPP_UNUSED(filenamePrefix);
    CPP_UNUSED(loadCenter);
    ConfigMap::iterator it;
    unsigned int mapIndex;
    GET_VALUE("mapIndex", mapIndex, UInt);

    name = config->get("name", name);
    path = config->get("path", path);

    GET_VALUE("index", index, ULong);
    GET_VALUE("maxStepSize", max_step_size, Double);
    GET_VALUE("stepSize", step_size, Double);
    GET_VALUE("relativeTolerance", relativeTolerance, Double);

    if (config->hasKey("inputs"))
    {
        for (auto Variable : (*config)["inputs"])
        {
            input_names.push_back(std::string(Variable));
        }
    }

    if (config->hasKey("outputs"))
    {
        for (auto Variable : (*config)["outputs"])
        {
            output_names.push_back(std::string(Variable));
        }
    }

    if (config->hasKey("observed"))
    {
        for (auto Variable : (*config)["observed"])
        {
            observed_names.push_back(std::string(Variable));
        }
    }
    // Set the config map
    this->config = *config;

    // Generate the directory
    tmp_path = utils::getCurrentWorkingDir() + "/tmp/mars_fmu/" + name;
    // Add randomness to temp path
    tmp_path += std::to_string(random() % 999999);
    if (not utils::createDirectory(tmp_path))
    {
        fprintf(stderr, "Failed to create temporary directory %s\n", tmp_path.c_str());
    }
};
} // namespace interfaces
} // namespace mars