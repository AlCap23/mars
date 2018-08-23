/*
 *  Copyright 2013, DFKI GmbH Robotics Innovation Center
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

/**
 * \file MarsFmu.cpp
 * \author Julius (julius.martensen@dfki.de)
 * \brief A
 *
 * Version 0.1
 */


#include "MarsFmu.h"
#include <mars/data_broker/DataBrokerInterface.h>
#include <mars/data_broker/DataPackage.h>

#include <fmilib.h>
#include <JM/jm_portability.h>

namespace mars {
  namespace plugins {
    namespace mars_fmu {

      // Make an importlogger
      void importlogger(jm_callbacks* callbacks, jm_string module, jm_log_level_enu_t log_level, jm_string message){
        printf("module = %s, log level = %s: %s\n", module, jm_log_level_to_string(log_level), message);
      }

      // Internal logger
      // TODO

      using namespace mars::utils;
      using namespace mars::interfaces;

      MarsFmu::MarsFmu(lib_manager::LibManager *theManager)
        : MarsPluginTemplate(theManager, "MarsFmu") {
      }

      void MarsFmu::init() {

        // Setup the callbacks
        callbacks.malloc = malloc;
        callbacks.calloc = calloc;
        callbacks.realloc = realloc;
        callbacks.free = free;
        callbacks.logger = importlogger;
        callbacks.log_level = jm_log_level_debug;
        callbacks.context = 0;

        // Get context and version
        context = fmi_import_allocate_context(&callbacks);
        // Need to give c strings
        version = fmi_import_get_fmi_version(context, fmu_path.c_str(), tmp_path.c_str());

        if(version != fmi_version_2_0_enu) {
          printf("The code only supports version 2.0\n");
        }

        fmu = fmi2_import_parse_xml(context, tmp_path.c_str(), 0);

        if(!fmu) {
      		printf("Error parsing XML, exiting\n");
      	}

        if(fmi2_import_get_fmu_kind(fmu) == fmi2_fmu_kind_me) {
      		printf("Only CS 2.0 is supported by this code\n");
      	}

        callbacks.logger = fmi2_log_forwarding;
      	callbacks.allocateMemory = calloc;
      	callbacks.freeMemory = free;
      	callbacks.componentEnvironment = fmu;

        status = fmi2_import_create_dllfmu(fmu, fmi2_fmu_kind_cs, &callbacks);
      	if (status == jm_status_error) {
      		printf("Could not create the DLL loading mechanism(C-API) (error: %s).\n", fmi2_import_get_last_error(fmu));
      	}

        printf("Now the simulation should start");


      }

      void MarsFmu::reset() {
      }

      MarsFmu::~MarsFmu() {
      }


      void MarsFmu::update(sReal time_ms) {

        // control->motors->setMotorValue(id, value);
      }

      void MarsFmu::receiveData(const data_broker::DataInfo& info,
                                    const data_broker::DataPackage& package,
                                    int id) {
        // package.get("force1/x", force);
      }

      void MarsFmu::cfgUpdateProperty(cfg_manager::cfgPropertyStruct _property) {

        if(_property.paramId == example.paramId) {
          example.dValue = _property.dValue;
        }
      }



    } // end of namespace mars_fmu
  } // end of namespace plugins
} // end of namespace mars

DESTROY_LIB(mars::plugins::mars_fmu::MarsFmu);
CREATE_LIB(mars::plugins::mars_fmu::MarsFmu);
