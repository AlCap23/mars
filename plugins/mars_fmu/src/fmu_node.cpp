#include "fmu_node.h"

#include <string>
#include <map>
#include <vector>

#include <configmaps/ConfigData.h>
#include <mars/utils/misc.h>

#include <mars/interfaces/sim/ControlCenter.h>

void fmu_logger(jm_callbacks *callbacks, jm_string module, jm_log_level_enu_t log_level, jm_string message)
{
  printf("module = %s, log level = %s: %s\n", module, jm_log_level_to_string(log_level), message);
}

void *createFMUThread(void *theObject)
{
  // Run the thread
  ((fmuNode *)theObject)->run();
  // Stop the thread
  //((fmuNode *)theObject)->setThreadStopped();
  pthread_exit(NULL);
  return 0;
}

// CONSTRUCTOR AND DESTRUCTOR

fmuNode::fmuNode(std::string Name, configmaps::ConfigMap fmu_config, mars::interfaces::ControlCenter *ControlCenter)
{
  // Set the instanceName
  fmu_instanceName = Name;
  // Set the config Map
  fmu_configMap = fmu_config;
  // Set the control center
  control = ControlCenter;

  // Read config map
  this->readConfig();
  // Init the FMU
  this->init();

  fprintf(stderr, "We are here right now \n");

  // Init the threading
  step_finished = true;
  pthread_mutex_init(&fmu_thread_Mutex, NULL);
  pthread_create(&fmu_thread, NULL, createFMUThread, (void *)this);
}

fmuNode::~fmuNode()
{
  fprintf(stderr, "Shutting down fmu! \n");
  // Stop the thread
  this->stopSimulation();
  this->setThreadStopped();

  // Wait for thread to stop
  while (thread_running)
  {
#ifdef WIN32
    Sleep(0.1);
#else
    usleep(10);
#endif
  }
  // Destroy the fmu
  fmi2_import_destroy_dllfmu(fmu);
  fmi2_import_free(fmu);
  fmi_import_free_context(context);

  // TODO Free tmp directory
  std::string delete_command = std::string("exec rm -r ") + tmp_path + std::string("/*");
  system(delete_command.c_str());

  if (rmdir(tmp_path.c_str()) == -1)
  {
    fprintf(stderr, "Error while removing Directory %s\n", tmp_path.c_str());
    fprintf(stderr, "   %s\n", strerror(errno));
  }
  else
  {
    fprintf(stderr, "Removed temporary directory \n");
  }

  // Unregister the data broker
  control->dataBroker->unregisterTimedProducer(this, "*", "*", "mars_sim/simTimer");
  control->dataBroker->unregisterTimedReceiver(this, "*", "*", "mars_sim/simTimer");

  printf("Destroyed fmu! \n");
}

// RESET FUNCTION

void fmuNode::reset()
{
  // Set the thread idle
  this->stopSimulation();

  // Reset the fmu and the internal variables
  fmu_status = fmi2_import_reset(fmu);
  if (fmu_status != fmi2_status_ok)
  {
    printf("Reset failed! \n");
  }

  // Initialize the fmu
  fmu_status = fmi2_import_enter_initialization_mode(fmu);
  if (fmu_status != fmi2_status_ok)
  {
    printf("Enter initialization failed on reset! \n");
  }

  //// Exit initialization mode
  fmu_status = fmi2_import_exit_initialization_mode(fmu);
  if (fmu_status != fmi2_status_ok)
  {
    printf("Exit initialization failed on reset! \n");
  }

  // Reset the local time
  local_time = 0.0;
  target_time = 0.0;

  this->SetInitialValues();

  printf("Reset of FMU %s  successful!\n", fmu_instanceName.c_str());
}

// STATUS GETTER

int *fmuNode::getStatus()
{
  return &simulation_status;
}

fmi2_real_t *fmuNode::getTargetTime()
{
  return &target_time;
}

fmi2_real_t fmuNode::getStepSize()
{
  return time_step;
}

// THREADING

void fmuNode::run()
{
  thread_running = true;
  fprintf(stderr, "Thread set running! \n");

  while (thread_running)
  {
    this->statusUpdate();

    if (do_step)
    {
      this->stepSimulation();
    }
    else
    {
      mars::utils::msleep(1);
    }
  }
}

void fmuNode::setThreadStopped()
{

  // Create a message
  fprintf(stderr, "Thread stopped! \n");
  thread_running = false;
}

// SIMULATION

void fmuNode::startSimulation()
{
  fprintf(stderr, "Starting FMU \n");
  step_finished = false;
  do_step = true;
}

void fmuNode::stopSimulation()
{

  fprintf(stderr, "Stopping FMU \n");
  do_step = false;
}

void fmuNode::statusUpdate()
{
  //fprintf(stderr, "Updating Status \n");
  // Set the simulation status

  // Thread running
  if (fmu_status != fmi2_status_ok)
  {
    // Error
    simulation_status = -1;
  }
  else
  {
    // Do a step
    if (do_step)
    {
      simulation_status = 1;
    }
    else if (step_finished)
    {

      simulation_status = 2;
    }
    else
    {
      simulation_status = 0;
    }
  }
}

void fmuNode::stepSimulation()
{

  target_time = local_time + time_step;
  fprintf(stderr, "Stepping \n");

  // Set the inputs
  this->setInputs();

  fmu_status = fmi2_import_do_step(fmu, local_time, time_step, fmi2_true);

  if (fmu_status != fmi2_status_ok)
  {
    printf("Simulation step failed! \n");
  }

  // Get outputs and observed
  this->getOutputs();
  this->getObserved();

  // Update the local time
  local_time = target_time;

  // Set simulation status
  step_finished = true;
  do_step = false;
}

// INPUT & OUTPUT SETTERS AND GETTERS

void fmuNode::setInputs()
{
  pthread_mutex_lock(&fmu_thread_Mutex);
  std::vector<fmi2_value_reference_t>::iterator i = fmu_inputs.begin();
  std::vector<mars::interfaces::sReal>::iterator j = current_inputs.begin();

  long package_id = 0.0;
  mars::interfaces::sReal cmd_value = 0.0;

  for (; i != fmu_inputs.end(); i++)
  {
    // Set the input
    if (j != current_inputs.end() && !std::isnan(*j))
    {
      fmu_status = fmi2_import_set_real(fmu, &(*i), 1, &(*j));
      if (fmu_status != fmi2_status_ok)
      {
        fprintf(stderr, "Setting FMU input value of instance %s failed! \n", fmu_instanceName.c_str());
      }
      j++;
    }
    else
    {
      fmu_status = fmi2_import_set_real(fmu, &(*i), 1, &cmd_value);
    }
  }

  pthread_mutex_unlock(&fmu_thread_Mutex);
}

void fmuNode::getOutputs()
{
  pthread_mutex_lock(&fmu_thread_Mutex);
  std::vector<fmi2_value_reference_t>::iterator i = fmu_outputs.begin();
  current_outputs.clear();

  fmi2_real_t current_value = 0.0;

  for (; i != fmu_outputs.end(); i++)
  {
    // Get the output
    fmu_status = fmi2_import_get_real(fmu, &(*i), 1, &current_value);
    if (fmu_status != fmi2_status_ok)
    {
      fprintf(stderr, "Getting FMU output value of instance %s failed! \n", fmu_instanceName.c_str());
    }
    else
    {
      current_outputs.push_back(current_value);
    }
  }
  pthread_mutex_unlock(&fmu_thread_Mutex);
}

void fmuNode::getObserved()
{
  pthread_mutex_lock(&fmu_thread_Mutex);
  std::vector<fmi2_value_reference_t>::iterator i = fmu_observed.begin();
  current_observed.clear();

  fmi2_real_t current_value = 0.0;

  for (; i != fmu_observed.end(); i++)
  {
    // Get the output
    fmu_status = fmi2_import_get_real(fmu, &(*i), 1, &current_value);
    if (fmu_status != fmi2_status_ok)
    {
      fprintf(stderr, "Getting FMU observed value of instance %s failed! \n", fmu_instanceName.c_str());
    }
    else
    {
      current_observed.push_back(current_value);
    }
  }
  pthread_mutex_unlock(&fmu_thread_Mutex);
}

// INIT FUNCTIONS

void fmuNode::init()
{
  // Set local time
  local_time = 0.0;
  target_time = local_time + time_step;

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
  if (version != fmi_version_2_0_enu)
  {
    printf("The code only supports version 2.0\n");
  }

  // Read the XML
  fmu = fmi2_import_parse_xml(context, tmp_path.c_str(), 0);

  if (!fmu)
  {
    printf("Error parsing XML, exiting\n");
  }

  // Check for FMU type (Cosim or ModelExchange)
  if (fmi2_import_get_fmu_kind(fmu) == fmi2_fmu_kind_me)
  {
    printf("Only CS 2.0 is supported by this code\n");
  }

  // Set callBackFunctions
  callBackFunctions.logger = fmi2_log_forwarding;
  callBackFunctions.allocateMemory = calloc;
  callBackFunctions.freeMemory = free;
  callBackFunctions.componentEnvironment = fmu;

  // Check the status
  fmu_status_jm = fmi2_import_create_dllfmu(fmu, fmi2_fmu_kind_cs, &callBackFunctions);
  if (fmu_status_jm == jm_status_error)
  {
    printf("Could not create the DLL loading mechanism(C-API) (error: %s).\n", fmi2_import_get_last_error(fmu));
  }

  // Get a model id
  fmu_GUID = fmi2_import_get_GUID(fmu);

  // Instanciate the fmu
  fmu_status_jm = fmi2_import_instantiate(fmu, fmu_instanceName.c_str(), fmi2_cosimulation, NULL, fmi2_false);
  if (fmu_status_jm == jm_status_error)
  {
    printf("fmi2_import_instantiate failed! \n");
    return;
  }

  // Creates the mappings and sets the initial values
  this->CreateMapping();

  // Initialize with the step size as an end time!
  fmu_status = fmi2_import_setup_experiment(fmu, fmi2_true, fmu_relativeTolerance, local_time, fmi2_false, 0.0);
  if (fmu_status != fmi2_status_ok)
  {
    printf("Setup experiment failed! \n");
  }

  // Initialize the fmu
  fmu_status = fmi2_import_enter_initialization_mode(fmu);
  if (fmu_status != fmi2_status_ok)
  {
    printf("Enter initialization failed! \n");
  }

  this->SetInitialValues();

  // Exit initialization mode
  fmu_status = fmi2_import_exit_initialization_mode(fmu);
  if (fmu_status != fmi2_status_ok)
  {
    printf("Exit initialization failed! \n");
  }
  else
  {
    printf("FMU initialization of %s finished successfully! \n", fmu_instanceName.c_str());
  }

  // Register at Databroker
  // Callled from master

  // Set the status
  do_step = false;
  step_finished = true;

  // Set the simulation stopped
  this->stopSimulation();
  // Set the status
  this->statusUpdate();
}

void fmuNode::readConfig()
{
  // Create temporary directory
  tmp_path = mars::utils::pathJoin(mars::utils::getCurrentWorkingDir(), "tmp/" + fmu_instanceName);
  // Add randomness to temp path
  tmp_path += "_" + std::to_string(random() % 999999);

  // Read the config map and set the corresponding values
  // Set the path
  if (fmu_configMap.hasKey("fmu_path"))
  {
    fmu_path = std::string(fmu_configMap["fmu_path"]);
    fprintf(stderr, " Loading model from %s\n", fmu_path.c_str());
  }
  else
  {
    fprintf(stderr, "No model path given! \n");
  }

  // Set the step size if given
  if (fmu_configMap.hasKey("step_size"))
  {
    time_step = double(fmu_configMap["step_size"]);
    fprintf(stderr, "Step size : %g\n", time_step);
  }
  else
  {
    time_step = 0.001;
  }
  // Set the relative tolerance if given
  if (fmu_configMap.hasKey("relative_tolerance"))
  {
    fmu_relativeTolerance = double(fmu_configMap["relative_tolerance"]);
  }
  else
  {
    fmu_relativeTolerance = 0.001;
  }

  // Generate the directory
  if (not mars::utils::createDirectory(tmp_path))
  {
    fprintf(stderr, "Failed to create temporary directory %s\n", tmp_path.c_str());
  }

  // Names for data broker
  producerGroup = "mars_fmu";
  receiverGroup = "mars_fmu";

  producerData = fmu_instanceName;
  receiverData = "cmd_" + fmu_instanceName;
}

void fmuNode::CreateMapping()
{
  // Read the inputs
  if (fmu_configMap.hasKey("inputs"))
  {
    for (auto Variable : fmu_configMap["inputs"])
    {
      std::string VariableName = std::string(Variable);
      this->MapToFMU(VariableName, 1);
    }
  } // End of inputs

  // Read the outputs
  if (fmu_configMap.hasKey("outputs"))
  {
    for (auto Variable : fmu_configMap["outputs"])
    {
      std::string VariableName = std::string(Variable);
      this->MapToFMU(VariableName, 2);
    }
  } // End of outputs

  // Read the observed variables
  if (fmu_configMap.hasKey("observed"))
  {
    for (auto observedVar : fmu_configMap["observed"])
    {
      std::string VariableName = std::string(observedVar);
      this->MapToFMU(VariableName, 3);
    }
  } // End of observed

  // Print newline
  fprintf(stderr, "\n");
}

void fmuNode::MapToFMU(std::string VariableName, int IO)
{
  // TODO register as timed producer!
  fmi2_import_variable_t *vr_pointer = fmi2_import_get_variable_by_name(fmu, VariableName.c_str());
  if (vr_pointer)
  {
    if (IO == 1)
    {
      // Here the inputs to the fmu get stored
      fprintf(stderr, " FMU Input : %s\n", VariableName.c_str());
      fmu_inputs.push_back(fmi2_import_get_variable_vr(vr_pointer));
      // Add this to the databroker
      receiverPackage.add("inputs/" + VariableName, 0.0);
      fmu_input_names.push_back("inputs/" + VariableName);
    }
    else if (IO == 2)
    {
      fprintf(stderr, " FMU Output : %s\n", VariableName.c_str());
      fmu_outputs.push_back(fmi2_import_get_variable_vr(vr_pointer));
      // Add this to the databroker
      producerPackage.add("outputs/" + VariableName, 0.0);
      fmu_output_names.push_back("outputs/" + VariableName);
    }
    else if (IO == 3)
    {
      fprintf(stderr, " FMU Observed : %s\n", VariableName.c_str());
      fmu_observed.push_back(fmi2_import_get_variable_vr(vr_pointer));
      fmu_observed_names.push_back("observed/" + VariableName);

      // Add this to the databroker
      producerPackage.add("observed/" + VariableName, 0.0);
    }
  }
  else
  {
    fprintf(stderr, "No variable named %s found in FMU\n", VariableName.c_str());
  }
}

void fmuNode::SetInitialValues()
{
  // This functions sets all initial values for the fmu in- and outputs

  fprintf(stderr, "Setting initial values for the FMU plugin. \n");

  // Set all initial sensor values
  //this->SetFMUInputs();

  // Set all the initial values given in the config
  if (fmu_configMap.hasKey("initial_values"))
  {
    configmaps::ConfigMap initialValues = fmu_configMap["initial_values"];
    for (auto initialVal : initialValues)
    {
      fmi2_import_variable_t *vr_pointer = fmi2_import_get_variable_by_name(fmu, (initialVal.first).c_str());
      if (vr_pointer)
      {
        fmi2_value_reference_t vr = fmi2_import_get_variable_vr(vr_pointer);
        fmi2_real_t init_val = double(initialVal.second);
        fprintf(stderr, " FMU Initial Value : %s = %g \n", (initialVal.first).c_str(), init_val);
        fmu_status = fmi2_import_set_real(fmu, &vr, 1, &init_val);

        if (fmu_status != fmi2_status_ok)
        {
          fprintf(stderr, "Setting FMU initial value of instance %s failed! \n", fmu_instanceName.c_str());
        }
      }
    }
  }

  this->setInputs();
  this->getOutputs();
  this->getObserved();
}

// DATA BROKER FUNCTIONS

void fmuNode::RegisterDataBroker(int interval)
{
  communication_interval = interval;
  fprintf(stderr, "%s  : interval is %d \n", fmu_instanceName.c_str(), interval);

  producerID = control->dataBroker->pushData(producerGroup, producerData, producerPackage, NULL, mars::data_broker::DATA_PACKAGE_READ_FLAG);

  control->dataBroker->registerTimedProducer(this, producerGroup, producerData,
                                             "mars_fmu/simTimer",
                                             interval);

  receiverID = control->dataBroker->pushData(receiverGroup, receiverData, receiverPackage, NULL, mars::data_broker::DATA_PACKAGE_READ_WRITE_FLAG);

  // TODO mars_fmu/simTimer
  control->dataBroker->registerTimedReceiver(this, receiverGroup, receiverData, "mars_fmu/simTimer", interval);
}

void fmuNode::produceData(const mars::data_broker::DataInfo &info,
                          mars::data_broker::DataPackage *package,
                          int callbackParam)
{
  pthread_mutex_lock(&fmu_thread_Mutex);
  // Reset the iterators
  long package_id = 0.0;
  //std::vector<fmi2_value_reference_t>::iterator i = fmu_observed.begin();
  std::vector<fmi2_real_t>::iterator j = current_observed.begin();
  std::vector<std::string>::iterator k = fmu_observed_names.begin();

  for (; j != current_observed.end() && k != fmu_observed_names.end(); j++, k++)
  {
    package_id = package->getIndexByName(*k);
    package->set(package_id, *j);
  }

  j = current_outputs.begin();
  k = fmu_output_names.begin();

  for (; j != current_outputs.end() && k != fmu_output_names.end(); j++, k++)
  {
    package_id = package->getIndexByName(*k);
    package->set(package_id, *j);
  }
  pthread_mutex_unlock(&fmu_thread_Mutex);
}

void fmuNode::receiveData(const mars::data_broker::DataInfo &info,
                          const mars::data_broker::DataPackage &package,
                          int callbackParam)
{
  pthread_mutex_lock(&fmu_thread_Mutex);
  // Reset the iterator
  std::vector<std::string>::iterator i = fmu_input_names.begin();
  current_inputs.clear();

  long package_id = 0.0;
  mars::interfaces::sReal cmd_value = 0.0;

  for (; i != fmu_input_names.end(); i++)
  {
    package_id = package.getIndexByName(*i);
    package.get(package_id, &cmd_value);

    if (package_id > -1)
    {
      current_inputs.push_back(cmd_value);
    }
    else
    {
      current_inputs.push_back(NAN);
    }
  }
  pthread_mutex_unlock(&fmu_thread_Mutex);
  this->startSimulation();
}
