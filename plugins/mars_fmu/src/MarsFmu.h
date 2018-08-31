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
 * \file MarsFmu.h
 * \author Julius (julius.martensen@dfki.de)
 * \brief A
 *
 * Version 0.1
 */

#ifndef MARS_PLUGINS_MARS_FMU_H
#define MARS_PLUGINS_MARS_FMU_H

#ifdef _PRINT_HEADER_
  #warning "MarsFmu.h"
#endif

// set define if you want to extend the gui
//#define PLUGIN_WITH_MARS_GUI
#include <mars/interfaces/sim/MarsPluginTemplate.h>
#include <mars/interfaces/MARSDefs.h>

#include <mars/cfg_manager/CFGManagerInterface.h>

// Use to publish the results and get the forces
#include <mars/data_broker/ProducerInterface.h>
#include <mars/data_broker/ReceiverInterface.h>

#include <mars/interfaces/sim/MotorManagerInterface.h>
#include <mars/interfaces/sim/SensorManagerInterface.h>

#include <string>

#include "fmilib.h"

namespace mars {

  namespace plugins {
    namespace mars_fmu {

      // inherit from MarsPluginTemplateGUI for extending the gui
      class MarsFmu: public mars::interfaces::MarsPluginTemplate,
      mars::data_broker::ReceiverInterface,
      mars::data_broker::ProducerInterface,

        // for gui
        // public mars::main_gui::MenuInterface,
        public mars::cfg_manager::CFGClient {

      public:
        MarsFmu(lib_manager::LibManager *theManager);
        ~MarsFmu();

        // LibInterface methods
        int getLibVersion() const
        { return 1; }
        const std::string getLibName() const
        { return std::string("mars_fmu"); }
        CREATE_MODULE_INFO();

        // MarsPlugin methods
        void init();
        void reset();
        void update(mars::interfaces::sReal time_ms);

        // DataBrokerReceiver methods
        virtual void receiveData(const data_broker::DataInfo &info,const data_broker::DataPackage &package, int callbackParam);
        virtual void produceData(const data_broker::DataInfo &info,
                                 data_broker::DataPackage *package,
                                 int callbackParam);
        //// CFGClient methods
        virtual void cfgUpdateProperty(cfg_manager::cfgPropertyStruct _property);

        // MenuInterface methods
        //void menuAction(int action, bool checked = false);

        // MarsFmu methods




      private:
        cfg_manager::cfgPropertyStruct example;

        //TODO Delte this!
        // Inputs and Outputs for the FMU
        //std::string fmu_inputs[2] = {"input_voltage", "external_torque"};
        //std::string fmu_outputs[1] = {"output_speed"};

        // Mars Inputs and outputs
        //std::string input_nodes[1] = {"motor_a"};
        //std::string output_nodes[1] = {"joint_torque"};

        // Map inputs and outputs from fmu to mars and vice versa
        std::map<std::string, std::string> mars_fmu_map; // = { {"joint_torque", "external_torque"}};
        std::map<std::string, std::string> fmu_mars_map;// = { {"output_speed", "motor_a"} };
        std::string fmu_observed[1] = {"output_speed"};

        // Make some ID maps
        std::map<unsigned long, fmi2_value_reference_t> mars_fmu_ID;
        std::map<fmi2_value_reference_t, unsigned long> fmu_mars_ID;

        // File system
        std::string typeName;
        std::string fmu_path; //= "/media/jmartensen/Data/linux/mars_dev/simulation/mars/plugins/mars_fmu/Test/PT2.fmu";
        std::string tmp_path; // = "/media/jmartensen/Data/linux/mars_dev/simulation/mars/plugins/mars_fmu/tmp";

        // Callbacks
        jm_callbacks callbacks;

        // FMI - Global variables used for managing
        fmi_import_context_t* context;
        fmi_version_enu_t version;

        fmi2_import_t* fmu;
        fmi2_callback_functions_t callBackFunctions;

        // FMI - Global variables used for simulation
        fmi2_string_t fmu_instanceName; // = "Test Model";
        fmi2_string_t fmu_GUID;
        //fmi2_string_t fmu_location = "";
        //fmi2_boolean_t fmu_visible = fmi2_false;
        fmi2_real_t fmu_relativeTolerance; // = 1e-2;

        fmi2_status_t fmu_status;
        jm_status_enu_t fmu_status_jm;

        // Need to work on that, here are the outputs
        //fmi2_value_reference_t fmu_Input_Ref[2] = {0, 0};
        //fmi2_value_reference_t fmu_Output_Ref[1] = {0};

        // Store inputs and outputs
        fmi2_real_t fmu_output = 0.0;
        fmi2_real_t *sensorData;
        //fmi2_real_t fmu_input_values[2] = {22.0, 10.0};
        //fmi2_real_t fmu_output_values[1] = {0.0};

        fmi2_real_t current_time; // = 0.0;
        fmi2_real_t time_step; // = 1e-3;
        fmi2_boolean_t stop_time_defined; // = fmi2_false;

        // Mars IDs
        //unsigned long mars_input_ids[1] = {0};
        //unsigned long mars_output_ids[1] = {0};


      }; // end of class definition MarsFmu

    } // end of namespace mars_fmu
  } // end of namespace plugins
} // end of namespace mars

#endif // MARS_PLUGINS_MARS_FMU_H
