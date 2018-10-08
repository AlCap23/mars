#ifndef FMU_NODE_H
#define FMU_NODE_H

#include <stdio.h>
#include <pthread.h>

#include <fmilib.h>
#include <JM/jm_portability.h>

#include <mars/interfaces/MARSDefs.h>
#include <mars/utils/misc.h>
#include <configmaps/ConfigData.h>

#include <mars/data_broker/ProducerInterface.h>
#include <mars/data_broker/ReceiverInterface.h>
#include <mars/data_broker/DataPackage.h>
#include <mars/data_broker/DataBrokerInterface.h>

#include <mars/interfaces/sim/ControlCenter.h>
#include <mars/interfaces/sim/NodeManagerInterface.h>
#include <mars/interfaces/sim/MotorManagerInterface.h>
#include <mars/interfaces/sim/SensorManagerInterface.h>

class fmuNode : public mars::data_broker::ProducerInterface,
                public mars::data_broker::ReceiverInterface
{

  // Configmap
  configmaps::ConfigMap fmu_configMap;
  // Save the control
  mars::interfaces::ControlCenter *control;
  // Data broker
  mars::data_broker::DataPackage producerPackage;
  mars::data_broker::DataPackage receiverPackage;
  std::string producerGroup, receiverGroup;
  std::string producerData, receiverData;
  unsigned long producerID, receiverID;

  // Threading
  pthread_t fmu_thread;
  pthread_mutex_t fmu_thread_Mutex;
  bool thread_running, stop_thread, do_step;

  // Filesystem for the fmu
  std::string fmu_path;
  std::string tmp_path;

  // Callbacks
  jm_callbacks callbacks;

  // FMI - Global variables used for managing
  fmi_import_context_t *context;
  fmi_version_enu_t version;

  fmi2_import_t *fmu;
  fmi2_callback_functions_t callBackFunctions;

  // FMI - Global variables used for simulation
  std::string fmu_instanceName;
  fmi2_string_t fmu_GUID;
  fmi2_real_t fmu_relativeTolerance;

  fmi2_status_t fmu_status;
  jm_status_enu_t fmu_status_jm;

  // From mars to fmu
  std::vector<mars::interfaces::sReal> current_inputs;
  std::vector<fmi2_value_reference_t> fmu_inputs;
  std::vector<std::string> fmu_input_names;
  // From fmu to mars
  std::vector<fmi2_real_t> current_outputs;
  std::vector<fmi2_value_reference_t> fmu_outputs;
  std::vector<std::string> fmu_output_names;
  // From fmu to data broker
  std::vector<fmi2_real_t> current_observed;
  std::vector<fmi2_value_reference_t> fmu_observed;
  std::vector<std::string> fmu_observed_names;

  //mars::interfaces::sReal current_update_time;
  fmi2_real_t current_time; // = 0.0;
  fmi2_real_t time_step;    // = 1e-3;

public:
  // Constructor and destructor
  fmuNode(configmaps::ConfigMap fmu_config, mars::interfaces::ControlCenter *ControlCenter);
  ~fmuNode();

  // Standard functions for simulation
  void init();
  void reset();

  // Threading
  void update(mars::interfaces::sReal time_ms);
  void run();
  void setThreadStopped();

  // Simulation
  void setInputs();
  void getOutputs();
  void getObserved();
  void stepSimulation();

  virtual void produceData(const mars::data_broker::DataInfo &info,
                           mars::data_broker::DataPackage *package,
                           int callbackParam);

  virtual void receiveData(const mars::data_broker::DataInfo &info,
                           const mars::data_broker::DataPackage &package,
                           int callbackParam);

  // Initialize the mapping
  void readConfig();
  void CreateMapping();
  void MapToFMU(std::string VariableName, int IO);
  void RegisterDataBroker();

  // Set the first values
  void SetInitialValues();
};
#endif
