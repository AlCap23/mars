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

#ifndef MARS_INTERFACES_FMU_DATA_H
#define MARS_INTERFACES_FMU_DATA_H

// FMI libs
#include <fmilib.h>
#include <JM/jm_portability.h>

#include <string>
#include "MARSDefs.h" //for sReal
#include <configmaps/ConfigData.h>

namespace mars
{
namespace interfaces
{
// Forward declaration
class LoadCenter;

/**
     * FMUData is a struct to exchange fmu information between the GUI and the simulation
     */
class FMUData
{
  public:
    explicit FMUData(const std::string &name = "");
    bool fromConfigMap(configmaps::ConfigMap *config, std::string filenamePrefix,
                       LoadCenter *loadCenter = 0);

    // Filesystem for the fmu
    std::string path;
    std::string tmp_path;

    // Configmap
    configmaps::ConfigMap config;

    // Name ( equal to instance name )
    std::string name;
    // Mars ID
    unsigned long index;
    // GUID
    fmi2_string_t GUID;
    // Management variables
    fmi_version_enu_t version;

    // Step size, Tolerance
    interfaces::sReal max_step_size, step_size;
    fmi2_real_t relativeTolerance;

    // FMU Variables
    // Inputs into the FMU
    std::vector<std::string> input_names;
    std::vector<fmi2_value_reference_t> inputs;
    // Outputs of the FMU
    std::vector<fmi2_value_reference_t> outputs;
    std::vector<std::string> output_names;
    // FMU Observed
    std::vector<fmi2_value_reference_t> observed;
    std::vector<std::string> observed_names;
};

} // namespace interfaces
} // namespace mars

#endif