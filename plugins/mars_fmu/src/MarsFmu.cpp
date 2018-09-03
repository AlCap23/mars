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

#include <mars/interfaces/sim/MotorManagerInterface.h>
#include <mars/interfaces/sim/SensorManagerInterface.h>

// External libraries from FMI Lib & JModelica
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
        fmu_path = "/media/jmartensen/Data/linux/mars_dev/simulation/mars/plugins/mars_fmu/Test/PT2.fmu";
        tmp_path = "/media/jmartensen/Data/linux/mars_dev/simulation/mars/plugins/mars_fmu/tmp";

        // Clear tmp folder
        system("exec rm -r /media/jmartensen/Data/linux/mars_dev/simulation/mars/plugins/mars_fmu/tmp/* ");

        fmu_instanceName = "Test Model";
        fmu_relativeTolerance = 0.001;
        current_time = 0.0;
        time_step = 0.001;
        stop_time_defined = fmi2_false;
        // Data broker configuration
        typeName = "FMU/";
        data_broker::DataPackage dbPackage;


        // Setup the callbacks
        callbacks.malloc = malloc;
        callbacks.calloc = calloc;
        callbacks.realloc = realloc;
        callbacks.free = free;
        callbacks.logger = importlogger;
        callbacks.log_level = jm_log_level_warning;
        callbacks.context = 0;

        // Get context and version
        context = fmi_import_allocate_context(&callbacks);
        // Need to give c strings
        version = fmi_import_get_fmi_version(context, fmu_path.c_str(), tmp_path.c_str());

        // Check for version of the FMU
        if(version != fmi_version_2_0_enu) {
          printf("The code only supports version 2.0\n");
        }

        // Read the XML
        fmu = fmi2_import_parse_xml(context, tmp_path.c_str(), 0);



        // Get the MARS -> FMU map
        for(auto mapping : mars_fmu_map){
          // Find the fmu variable
          fmi2_import_variable_t* vr_pointer = fmi2_import_get_variable_by_name(fmu, mapping.second.c_str() );
          // Map the variable onto the motor
          mars_fmu_ID[control->sensors->getSensorID(mapping.first)] = fmi2_import_get_variable_vr(vr_pointer);
          // Push the id to the mapping array
          dataBroker_ID.push_back(fmi2_import_get_variable_vr(vr_pointer));
          // Add the data broker packages
          dbPackage.add(mapping.second, 0.0);
        }

        for(auto mapping : fmu_mars_map){
          // Find the fmu variable
          fmi2_import_variable_t* vr_pointer = fmi2_import_get_variable_by_name(fmu, mapping.first.c_str() );
          fmu_mars_ID[fmi2_import_get_variable_vr(vr_pointer)] = control->motors->getID(mapping.second);
          // Push the id to the mapping array
          dataBroker_ID.push_back(fmi2_import_get_variable_vr(vr_pointer));
          // Add the data broker packages
          dbPackage.add(mapping.first, 0.0);
        }


        if(!fmu) {
      		printf("Error parsing XML, exiting\n");
      	}

        // Check for FMU type (Cosim or ModelExchange)
        if(fmi2_import_get_fmu_kind(fmu) == fmi2_fmu_kind_me) {
      		printf("Only CS 2.0 is supported by this code\n");
      	}

        // Set callBackFunctions
        callBackFunctions.logger = fmi2_log_forwarding;
      	callBackFunctions.allocateMemory = calloc;
      	callBackFunctions.freeMemory = free;
      	callBackFunctions.componentEnvironment = fmu;

        // Check the status
        fmu_status_jm = fmi2_import_create_dllfmu(fmu, fmi2_fmu_kind_cs, &callBackFunctions);
      	if (fmu_status_jm == jm_status_error) {
      		printf("Could not create the DLL loading mechanism(C-API) (error: %s).\n", fmi2_import_get_last_error(fmu));
      	}

        // Get a model id
        fmu_GUID = fmi2_import_get_GUID(fmu);

        // Instanciate the fmu
        fmu_status_jm = fmi2_import_instantiate(fmu, fmu_instanceName, fmi2_cosimulation, NULL, fmi2_false);
        if(fmu_status_jm == jm_status_error){
          printf("fmi2_import_instantiate failed! \n");
          return;
        }

        // Initialize with the step size as an end time!
        fmu_status = fmi2_import_setup_experiment(fmu, fmi2_true, fmu_relativeTolerance, current_time, stop_time_defined, time_step);
        if(fmu_status != fmi2_status_ok){
          printf("Setup experiment failed! \n");
        }

        // Initialize the fmu
        fmu_status = fmi2_import_enter_initialization_mode(fmu);
        if(fmu_status != fmi2_status_ok){
          printf("Enter initialization failed! \n");
        }

        // Exit initialization mode
        fmu_status = fmi2_import_exit_initialization_mode(fmu);
        if(fmu_status != fmi2_status_ok){
          printf("Enter initialization failed! \n");
        }

        // Finish data broker config

        std::string groupName = "external_sim";
        std::string dataName = typeName.append(fmu_instanceName);

        control->dataBroker->pushData(groupName, dataName, dbPackage, NULL, data_broker::DATA_PACKAGE_READ_FLAG);
        control->dataBroker->registerTimedProducer(this, groupName, dataName, "mars_sim/simTimer", 0);


      }

      void MarsFmu::reset() {
        // Reset the fmu and the internal variables
        fmu_status = fmi2_import_reset(fmu);
        if(fmu_status != fmi2_status_ok){
          printf("Reset failed! \n");
        }

        //// Initialize the fmu
        fmu_status = fmi2_import_enter_initialization_mode(fmu);
        if(fmu_status != fmi2_status_ok){
          printf("Enter initialization failed! \n");
        }

        //// Exit initialization mode
        fmu_status = fmi2_import_exit_initialization_mode(fmu);
        if(fmu_status != fmi2_status_ok){
          printf("Enter initialization failed! \n");
        }
        current_time = 0.0;

      }

      MarsFmu::~MarsFmu() {
        // Remove from dataBroker
        fprintf(stderr, "Destructor ! \n");

        //control->dataBroker->unregisterTimedProducer(this, groupName, dataName, "mars_sim/simTimer");

        // Destroy the fmu
        fmi2_import_destroy_dllfmu(fmu);
        fmi2_import_free(fmu);
        fmi_import_free_context(context);
        printf("Destroyed fmu! \n");
      }


      void MarsFmu::update(sReal time_ms) {

// SEEMS TO WORK
        // Get the input values
        for(auto mapping : mars_fmu_ID){

            int numSensorValues = control->sensors->getSensorData(mapping.first, &sensorData);
            fmu_status = fmi2_import_set_real(fmu, &mapping.second, 1, sensorData);

        }

      fmi2_value_reference_t input_v = 23;
      fmi2_real_t input_amp = 35.0;
      // Conversion needed because mars calculates in ms
      while(current_time*1000 < time_ms){


        fmu_status = fmi2_import_set_real(fmu, &input_v, 1, &input_amp);
        // Do a step
        fmu_status = fmi2_import_do_step(fmu, current_time, time_step, fmi2_true);

        // Check if fmu feels good
        if(fmu_status != fmi2_status_ok){
          printf("Error during simulation! \n");
        }
        // Raise time
        current_time += time_step;
      }

        // Set the output values
        for(auto mapping : fmu_mars_ID){
          fmu_status = fmi2_import_get_real(fmu, &mapping.first, 1, &fmu_output);
          control->motors->setMotorValue(mapping.second, fmu_output);
        }


        // Reset internal time
        current_time = 0.0;
      }

      void MarsFmu::produceData(const data_broker::DataInfo &info,
                                         data_broker::DataPackage *package,
                                         int callbackParam) {

          for(int i = 0; i < sizeof(dataBroker_ID)/sizeof(dataBroker_ID[0]); i++){
            fmu_status = fmi2_import_get_real(fmu, &dataBroker_ID[i], 1, &fmu_output);

            // Check if fmu feels good
            if(fmu_status != fmi2_status_ok){
              printf("Error during simulation! \n");
            }

            package->set(i, fmu_output);

          }

                                  }

     void MarsFmu::receiveData(const data_broker::DataInfo &info, const data_broker::DataPackage &package, int callbackParam){
       // package.get("force1/x", force);
     }

      void MarsFmu::cfgUpdateProperty(cfg_manager::cfgPropertyStruct _property) {
      }



    } // end of namespace mars_fmu
  } // end of namespace plugins
} // end of namespace mars

DESTROY_LIB(mars::plugins::mars_fmu::MarsFmu);
CREATE_LIB(mars::plugins::mars_fmu::MarsFmu);
