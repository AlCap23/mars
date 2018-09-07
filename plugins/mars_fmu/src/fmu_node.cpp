#include "fmu_node.h"

namespace FMU_NODE{

fmuNode::fmuNode(std::string filePath, std::string instanceName, std::vector<std::string> fmu_variables){
  // Set the fmu path
  fmu_path = filePath;
  fmu_instanceName = instanceName;

  // TODO Make a temp directory
  tmp_path = "/media/jmartensen/Data/linux/mars_dev/simulation/mars/plugins/mars_fmu/tmp";

  // Clear tmp folder
  system("exec rm -r /media/jmartensen/Data/linux/mars_dev/simulation/mars/plugins/mars_fmu/tmp/* ");

  // Init the I/O maps without value refs
  for(std::vector<int>::size_type i = 0; i!= fmu_variables.size(); i++){
    // Init with -1
    reference_map[fmu_variables[i]] = -1;
  }

  // Init the FMU
  this->init();

}


fmuNode::~fmuNode(){
  // Destroy the fmu
  fmi2_import_destroy_dllfmu(fmu);
  fmi2_import_free(fmu);
  fmi_import_free_context(context);

  // TODO Free tmp directory

  printf("Destroyed fmu! \n");
}


void fmuNode::fmu_logger(jm_callbacks* callbacks, jm_string module, jm_log_level_enu_t log_level, jm_string message){
    printf("module = %s, log level = %s: %s\n", module, jm_log_level_to_string(log_level), message);
}


void fmuNode::init(){

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
  callbacks.logger = fmuNode::fmu_logger;
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

  if(!fmu) {
    printf("Error parsing XML, exiting\n");
  }

  // Get the value references
  for(auto mapping : reference_map){
    // Find the fmu variable
    fmi2_import_variable_t* vr_pointer = fmi2_import_get_variable_by_name(fmu, mapping.first.c_str() );
    // Map the variable onto the motor
    mapping.second = fmi2_import_get_variable_vr(vr_pointer);
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
    printf("Exit initialization failed! \n");
  } else {
    printf("FMU initialization finished successfully! \n");
  }
}

void fmuNode::reset(){
  // Reset the fmu and the internal variables
  fmu_status = fmi2_import_reset(fmu);
  if(fmu_status != fmi2_status_ok){
    printf("Reset failed! \n");
  }

  //// Initialize the fmu
  fmu_status = fmi2_import_enter_initialization_mode(fmu);
  if(fmu_status != fmi2_status_ok){
    printf("Enter initialization failed on reset! \n");
  }

  //// Exit initialization mode
  fmu_status = fmi2_import_exit_initialization_mode(fmu);
  if(fmu_status != fmi2_status_ok){
    printf("Exit initialization failed on reset! \n");
  }

  current_time = 0.0;

}

void fmuNode::stepSimulation(){

  // Set the input values
  for(auto input_map : fmu_input){
    fmu_status = fmi2_import_set_real(fmu, &mapping.first, &mapping.second);

    if(fmu_status != fmi2_status_ok){
      printf("Setting input failed! \n");
    }

  }

  // Do a step
  fmu_status = fmi2_import_do_step(fmu, current_time, time_step, fmi2_true);

  if(fmu_status != fmi2_status_ok){
    printf("Simulation step failed! \n");
  }

}

} // end of namespace FMU_NODE
