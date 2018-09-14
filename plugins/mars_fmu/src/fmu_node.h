#ifndef FMU_NODE_H
#define FMU_NODE_H


#include<stdio.h>


#include <fmilib.h>
#include <JM/jm_portability.h>

#include <mars/interfaces/MARSDefs.h>
#include <mars/utils/misc.h>
#include <configmaps/ConfigData.h>


#include <mars/interfaces/sim/ControlCenter.h>
#include <mars/interfaces/sim/NodeManagerInterface.h>
#include <mars/interfaces/sim/MotorManagerInterface.h>
#include <mars/interfaces/sim/SensorManagerInterface.h>


class fmuNode{
  // Configmap
  configmaps::ConfigMap fmu_configMap;
  // Save the control
  mars::interfaces::ControlCenter* control;

  // Filesystem for the fmu
  std::string fmu_path;
  std::string tmp_path;

  // Callbacks
  jm_callbacks callbacks;

  // FMI - Global variables used for managing
  fmi_import_context_t* context;
  fmi_version_enu_t version;

  fmi2_import_t* fmu;
  fmi2_callback_functions_t callBackFunctions;

  // FMI - Global variables used for simulation
  std::string fmu_instanceName;
  fmi2_string_t fmu_GUID;
  fmi2_real_t fmu_relativeTolerance;

  fmi2_status_t fmu_status;
  jm_status_enu_t fmu_status_jm;

  // From mars to fmu
  std::vector<unsigned long> mars_outputs;
  std::vector<fmi2_value_reference_t> fmu_inputs;
  // From fmu to mars
  std::vector<unsigned long> mars_inputs;
  std::vector<fmi2_value_reference_t> fmu_outputs;
  // From fmu to data broker
  std::vector<fmi2_value_reference_t> fmu_observed;

  fmi2_real_t current_time; // = 0.0;
  fmi2_real_t time_step; // = 1e-3;

public:
  // Constructor and destructor
  fmuNode(configmaps::ConfigMap fmu_config, mars::interfaces::ControlCenter* ControlCenter);
  ~fmuNode();

  // Standard functions for simulation
  void init();
  void reset();
  void stepSimulation(mars::interfaces::sReal update_time);

  // Initialize the mapping
  void readConfig();
  void CreateMapping();
  int MapToMars(std::string VariableName);
  void MapToFMU(std::string VariableName, int IO);

  // Set the first values
  void SetInitialValues();
  void SetFMUInputs();
  void SetFMUOutputs();
};

#endif
