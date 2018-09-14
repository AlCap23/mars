#include "fmu_node.h"

#include <string>
#include <map>
#include <vector>

#include <configmaps/ConfigData.h>
#include <mars/utils/misc.h>

#include <mars/interfaces/sim/ControlCenter.h>
#include <mars/interfaces/sim/NodeManagerInterface.h>
#include <mars/interfaces/sim/MotorManagerInterface.h>
#include <mars/interfaces/sim/SensorManagerInterface.h>


void fmu_logger(jm_callbacks* callbacks, jm_string module, jm_log_level_enu_t log_level, jm_string message){
    printf("module = %s, log level = %s: %s\n", module, jm_log_level_to_string(log_level), message);
}

fmuNode::fmuNode(configmaps::ConfigMap fmu_config, mars::interfaces::ControlCenter* ControlCenter){
  // Set the config Map
  fmu_configMap = fmu_config;
  // Set the control center
  control = ControlCenter;

  // Read config map
  this->readConfig();
  // Init the FMU
  this->init();

}


fmuNode::~fmuNode(){
  // Destroy the fmu
  fmi2_import_destroy_dllfmu(fmu);
  fmi2_import_free(fmu);
  fmi_import_free_context(context);

  // TODO Free tmp directory
  std::string delete_command = std::string("exec rm -r ")+tmp_path+std::string("/*");
  system(delete_command.c_str());

  if(rmdir(tmp_path.c_str()) == -1){
    fprintf(stderr, "Error while removing Directory %s\n", tmp_path.c_str());
    fprintf(stderr, "   %s\n", strerror(errno));
  } else{
    fprintf(stderr, "Removed temporary directory \n");
  }

  printf("Destroyed fmu! \n");
}


void fmuNode::init(){

  current_time = 0.0;


  // Setup the callbacks
  callbacks.malloc = malloc;
  callbacks.calloc = calloc;
  callbacks.realloc = realloc;
  callbacks.free = free;
  callbacks.logger = fmu_logger;
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
  fmu_status_jm = fmi2_import_instantiate(fmu, fmu_instanceName.c_str(), fmi2_cosimulation, NULL, fmi2_false);
  if(fmu_status_jm == jm_status_error){
    printf("fmi2_import_instantiate failed! \n");
    return;
  }

  // Creates the mappings and sets the initial values
  this->CreateMapping();

  // Initialize with the step size as an end time!
  fmu_status = fmi2_import_setup_experiment(fmu, fmi2_true, fmu_relativeTolerance, current_time, fmi2_false, 0.0);
  if(fmu_status != fmi2_status_ok){
    printf("Setup experiment failed! \n");
  }

  // Initialize the fmu
  fmu_status = fmi2_import_enter_initialization_mode(fmu);
  if(fmu_status != fmi2_status_ok){
    printf("Enter initialization failed! \n");
  }

  this->SetInitialValues();

  // Exit initialization mode
  fmu_status = fmi2_import_exit_initialization_mode(fmu);
  if(fmu_status != fmi2_status_ok){
    printf("Exit initialization failed! \n");
  } else {
    printf("FMU initialization of %s finished successfully! \n", fmu_instanceName.c_str());
  }

}

void fmuNode::reset(){

  // Reset the fmu and the internal variables
  fmu_status = fmi2_import_reset(fmu);
  if(fmu_status != fmi2_status_ok){
    printf("Reset failed! \n");
  }

  // Initialize the fmu
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
  this->SetInitialValues();
  this->SetFMUInputs();
  printf("Reset of FMU %s  successful!\n", fmu_instanceName.c_str());
}

void fmuNode::stepSimulation(mars::interfaces::sReal update_time){

  fprintf(stderr, "Update time is %4.2f for %4.2f \n", update_time, current_time);

  // Get and set all sensor values
  this->SetFMUInputs();

  while(current_time<update_time){

    fmu_status = fmi2_import_do_step(fmu, current_time, time_step, fmi2_true);

    if(fmu_status != fmi2_status_ok){
      printf("Simulation step failed! \n");
    }

    current_time += time_step;
    }
  current_time = 0.0;


  fprintf(stderr, "Finished stepping FMU\n");

  // Synchronize with mars
  this->SetFMUOutputs();

  fprintf(stderr, "Finished setting Motor values \n");

  }

  void fmuNode::readConfig(){
    // Create temporary directory
    std::string curret_working_dir = mars::utils::getCurrentWorkingDir();
    curret_working_dir += "/tmp/mars_fmu";

    // Read the config map and set the corresponding values

    // Set the path
    if(fmu_configMap.hasKey("fmu_path")){
      fmu_path = std::string(fmu_configMap["fmu_path"]);
      fprintf(stderr, " Loading model from %s\n", fmu_path.c_str());
    } else{
      fprintf(stderr, "No model path given! \n");
    }

    // Get the instancce Name
    if(fmu_configMap.hasKey("instanceName")){
      fmu_instanceName = std::string(fmu_configMap["instanceName"]);
      tmp_path = mars::utils::pathJoin(curret_working_dir,fmu_configMap["instanceName"]);
    } else{
      fmu_instanceName = "Instance_1";
      tmp_path = mars::utils::pathJoin(curret_working_dir,fmu_instanceName);
    }
    // Add randomness to temp path
    tmp_path += std::to_string(random() % 999999);

    // Set the step size if given
    if(fmu_configMap.hasKey("step_size")){
      time_step = double(fmu_configMap["step_size"]);
      fprintf(stderr, "Step size : %g\n", time_step);
    } else {
      time_step = 0.001;
    }
    // Set the relative tolerance if given
    if(fmu_configMap.hasKey("relative_tolerance")){
      fmu_relativeTolerance = double(fmu_configMap["relative_tolerance"]);
    } else {
      fmu_relativeTolerance = 0.001;
    }



    // Generate the directory
    if(not mars::utils::createDirectory(tmp_path)){
      fprintf(stderr, "Failed to create temporary directory %s\n", tmp_path.c_str());
    }

}


void fmuNode::CreateMapping(){

  // READ IO MAP
  if(fmu_configMap.hasKey("io_mapping")){
    configmaps::ConfigMap ioMaps = fmu_configMap["io_mapping"];
    std::string first_key;
    std::string second_key;
    // Iterate over the map
    for(auto ioMapping : ioMaps){
      first_key = std::string(ioMapping.first);
      second_key = std::string(ioMapping.second);

      // Check if either first key is in mars
      int IO = this->MapToMars(first_key);
      // If first key is found in mars, than find second key in FMU
      if(IO > 0){
        this->MapToFMU(second_key, IO);
      }
      // If first key is not found in mars, check if second key can be found
      else{
        IO = this->MapToMars(second_key);
        if(IO > 0){
          // Then the first key should be in FMU
          this->MapToFMU(first_key, IO);
        }
      }

    } // End of iterator through IO Maps
  } // End of IO mapping

  if(fmu_configMap.hasKey("observed")){
    for(auto observedVar : fmu_configMap["observed"]){
      std::string VariableName = std::string(observedVar);
      this->MapToFMU(VariableName, 3);
    }
  } // End of observed


  // Print newline
  fprintf(stderr, "\n");
}


int fmuNode::MapToMars(std::string VariableName){
  if(control->motors->getID(VariableName)>0){
    // Clearly Input to mars
    fprintf(stderr, " Mars Input : %s\n", VariableName.c_str());
    mars_inputs.push_back(control->motors->getID(VariableName));
    // If input to mars, then the fmu is a corresponding output
    return 2;
  }
  else if(control->sensors->getSensorID(VariableName)){
    // Clearly output from mars
    fprintf(stderr, " Mars Output : %s\n", VariableName.c_str());
    mars_outputs.push_back(control->sensors->getSensorID(VariableName));
    // if output from mars, the fmu is a corresponding input
    return 1;
  }
  else{
    // Else
    return -1;
  }
}

void fmuNode::MapToFMU(std::string VariableName, int IO){
  // TODO register as timed producer!
  fmi2_import_variable_t* vr_pointer = fmi2_import_get_variable_by_name(fmu, VariableName.c_str());
  if(vr_pointer){
    if(IO == 1){
      // Here the inputs to the fmu get stored
      fprintf(stderr, " FMU Input : %s\n", VariableName.c_str());
      fmu_inputs.push_back(fmi2_import_get_variable_vr(vr_pointer));
    }
    else if(IO == 2){
      fprintf(stderr, " FMU Output : %s\n", VariableName.c_str());
      fmu_outputs.push_back(fmi2_import_get_variable_vr(vr_pointer));
    }
    else if(IO == 3){
      fprintf(stderr, " FMU Observed : %s\n", VariableName.c_str());
      fmu_observed.push_back(fmi2_import_get_variable_vr(vr_pointer));
    }
  } else{
    fprintf(stderr, "No variable named %s found in FMU\n", VariableName.c_str());
  }
}

void fmuNode::SetInitialValues(){
  // This functions sets all initial values for the fmu in- and outputs

  fprintf(stderr, "Setting initial values for the FMU plugin. \n");

  // Set all initial sensor values
  this->SetFMUInputs();

  // Set all the initial values given in the config
  if(fmu_configMap.hasKey("initial_values")){
    configmaps::ConfigMap initialValues = fmu_configMap["initial_values"];
    for(auto initialVal : initialValues){
      fmi2_import_variable_t* vr_pointer = fmi2_import_get_variable_by_name(fmu, (initialVal.first).c_str());
      if(vr_pointer){
          fmi2_value_reference_t vr = fmi2_import_get_variable_vr(vr_pointer);
          fmi2_real_t init_val = double(initialVal.second);
          fprintf(stderr, " FMU Initial Value : %s = %g \n", (initialVal.first).c_str(), init_val);
          fmu_status = fmi2_import_set_real(fmu, &vr, 1, &init_val);

          if(fmu_status != fmi2_status_ok){
            fprintf(stderr, "Setting FMU initial value of instance %s failed! \n", fmu_instanceName.c_str());
          }
        }
    }
  }

  // Set all the initial outputs to zero
  // Init the iterators
  std::vector<fmi2_value_reference_t>::iterator i = fmu_outputs.begin();
  std::vector<unsigned long>::iterator j = mars_inputs.begin();

  fmi2_real_t current_input = 0.0;

  for(; i != fmu_outputs.end() && j != mars_inputs.end(); ++i, ++j){
    // This is highly confusing for me --> pointer to begin of iterator data?
    fprintf(stderr, " Set FMU output %d to %g \n", *i, current_input);
    fmu_status = fmi2_import_set_real(fmu, &(*i), 1, &current_input);
    if(fmu_status != fmi2_status_ok){
      fprintf(stderr, "Setting FMU output value of instance %s failed! \n", fmu_instanceName.c_str());
    }
    control->motors->setMotorValue(*j, current_input);
  }
  //
}


void fmuNode::SetFMUInputs(){
  // Set all internal data from mars to the fmu

  // Define the internal status of the fmu
  mars::interfaces::sReal *sensorData = NULL;


  // Set the input values
  std::vector<fmi2_value_reference_t>::iterator i = fmu_inputs.begin();
  std::vector<unsigned long>::iterator j = mars_outputs.begin();

  for(; i != fmu_inputs.end() && j != mars_outputs.end(); ++i, ++j){

    int numSensorValues = control->sensors->getSensorData(*j, &sensorData);

    if(sensorData){
      fprintf(stderr, " Sensor Data %d , %g\n", *j, sensorData[0]);
    // This is highly confusing for me --> pointer to begin of iterator data?
      fmu_status = fmi2_import_set_real(fmu, &*i, 1, sensorData);
      if(fmu_status != fmi2_status_ok){
        fprintf(stderr, "Setting FMU input value of instance %s failed! \n", fmu_instanceName.c_str());
      }
    }
  }
  free(sensorData);
}

void fmuNode::SetFMUOutputs(){
  // Reset the iterators
  std::vector<fmi2_value_reference_t>::iterator i = fmu_outputs.begin();
  std::vector<unsigned long>::iterator j = mars_inputs.begin();

  fmi2_real_t current_input = 0.0;

  for(; i != fmu_outputs.end() && j != mars_inputs.end(); ++i, ++j){

    // This is highly confusing for me --> pointer to begin of iterator data?
    fprintf(stderr, "%d \n", *i);
    fmu_status = fmi2_import_get_real(fmu, &(*i), 1, &current_input);
    fprintf(stderr, "Motor Data %d , %g\n", *j, double(current_input));
    if(fmu_status != fmi2_status_ok){
      fprintf(stderr, "Getting FMU output value of instance %s failed! \n", fmu_instanceName.c_str());
    }
    control->motors->setMotorValue(*j, current_input);
  }
}
