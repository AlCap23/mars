#ifndef FMU_MASTER_H
#define FMU_MASTER_H

// Standard libs
#include <string>

// Internal mars libs
#include <mars/interfaces/MARSDefs.h>
#include <mars/cfg_manager/CFGManagerInterface.h>

// External libraries from FMI Lib & JModelica
#include "fmu_node.h"
#include <fmilib.h>
#include <JM/jm_portability.h>

class fmuMaster
{
public:
  fmuMaster(mars::interfaces::ControlCenter *controlCenter);
  ~fmuMaster();

  void update(mars::interfaces::sReal update_time);
  void stop();
  void reset();
  void setStepSize();
  void setUpdateInterval();
  bool checkStatus(int *status);

private:
  // Step size of the fmu timer
  fmi2_real_t current_time;
  fmi2_real_t step_size;
  // control center
  mars::interfaces::ControlCenter *control;
  // Configuration file
  configmaps::ConfigMap marsfmu_Config;
  // Storage for all fmu models
  std::vector<fmuNode *> fmu_models;
  // Storage for the status of fmu
  std::vector<int *> fmu_status;
  // Storage for the target times
  std::vector<fmi2_real_t *> fmu_targetTime;
};
#endif