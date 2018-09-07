#ifndef FMU_NODE_H
#define FMU_NODE_H


#include <string>
#include <map>
#include <vector>

#include <fmilib.h>
#include <JM/jm_portability.h>

#include <mars/interfaces/MARSDefs.h>


class fmuNode{

  // Filesystem for the fmu
  std::string fmu_path;
  char* tmp_path;

  // Define the variables for managing

  // Callbacks
  jm_callbacks callbacks;

  // FMI - Global variables used for managing
  fmi_import_context_t* context;
  fmi_version_enu_t version;

  fmi2_import_t* fmu;
  fmi2_callback_functions_t callBackFunctions;

  // FMI - Global variables used for simulation
  fmi2_string_t fmu_instanceName;
  fmi2_string_t fmu_GUID;
  fmi2_real_t fmu_relativeTolerance;

  fmi2_status_t fmu_status;
  jm_status_enu_t fmu_status_jm;

  // Store inputs and outputs
  std::map<std::string, fmi2_value_reference_t> reference_map;
  std::map<fmi2_value_reference_t, fmi2_real_t> fmu_input;
  std::map<fmi2_value_reference_t, fmi2_real_t> fmu_output;

  fmi2_real_t current_time; // = 0.0;
  fmi2_real_t time_step; // = 1e-3;
  fmi2_boolean_t stop_time_defined; // = fmi2_false;

public:
  // Constructor and destructor
  fmuNode(std::string filePath,std::string tmpPath, std::string instanceName, std::vector<std::string> fmu_variables);
  ~fmuNode();

  // Standard functions
  void init(std::vector<std::string> fmu_variables);
  void reset();
  void stepSimulation(double update_time);
};

#endif
