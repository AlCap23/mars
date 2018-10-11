#ifndef FMU_MASTER_H
#define FMU_MASTER_H

// Standard libs
#include <string>

// Internal mars libs
#include <mars/utils/Thread.h>
#include <mars/utils/misc.h>
#include <mars/interfaces/MARSDefs.h>
#include <mars/cfg_manager/CFGManagerInterface.h>
#include <configmaps/ConfigData.h>
#include <mars/interfaces/sim/ControlCenter.h>
#include <mars/data_broker/ProducerInterface.h>
#include <mars/data_broker/ReceiverInterface.h>
#include <mars/data_broker/DataPackage.h>
#include <mars/data_broker/DataBrokerInterface.h>

// External libraries from FMI Lib & JModelica
#include "fmu_node.h"
#include <fmilib.h>
#include <JM/jm_portability.h>

class fmuMaster : public mars::data_broker::ProducerInterface,
                  public mars::data_broker::ReceiverInterface,
                  public mars::utils::Thread
{
public:
  fmuMaster(mars::interfaces::ControlCenter *controlCenter);
  ~fmuMaster();

  void update(mars::interfaces::sReal update_time);
  void stop();
  void reset();
  void setStepSize();
  void setUpdateInterval();
  bool getWait();
  bool checkStatus(int *status);
  void registerDataBroker();

  virtual void produceData(const mars::data_broker::DataInfo &info,
                           mars::data_broker::DataPackage *package,
                           int callbackParam);

  virtual void receiveData(const mars::data_broker::DataInfo &info,
                           const mars::data_broker::DataPackage &package,
                           int callbackParam);

protected:
  void run();

private:
  bool mars_wait;
  // Threading
  bool thread_running;
  // control center
  mars::interfaces::ControlCenter *control;
  // Configuration file
  configmaps::ConfigMap marsfmu_Config;
  // Data broker
  mars::data_broker::DataPackage producerPackage;
  mars::data_broker::DataPackage receiverPackage;
  std::string producerGroup, receiverGroup;
  std::string producerData, receiverData;
  unsigned long producerID, receiverID;
  mars::interfaces::sReal current_mars_time;

  // Step size of the fmu timer
  fmi2_real_t current_time;
  fmi2_real_t step_size;
  fmi2_real_t next_target;
  // Storage for all fmu models
  std::vector<fmuNode *> fmu_models;
  // Storage for the target times
  std::vector<fmi2_real_t *> fmu_targetTime;
};
#endif