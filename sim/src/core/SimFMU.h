/*
 *  Copyright 2018 DFKI GmbH Robotics Innovation Center
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

#ifndef SIMFMU_H
#define SIMFMU_H

#ifdef _PRINT_HEADER_
#warning "SimFMU.h"
#endif

// Standard libs
#include <stdio.h>
#include <pthread.h>

// FMI libs
#include <fmilib.h>
#include <JM/jm_portability.h>

// Mars internal libs
#include <mars/interfaces/MARSDefs.h>
#include <mars/utils/misc.h>
#include <mars/interfaces/FMUData.h>
#include <configmaps/ConfigData.h>
#include <mars/interfaces/sim/ControlCenter.h>
#include <mars/data_broker/ProducerInterface.h>
#include <mars/data_broker/ReceiverInterface.h>
#include <mars/data_broker/DataPackage.h>
#include <mars/data_broker/DataBrokerInterface.h>

namespace mars
{
namespace interfaces
{
class ControlCenter;
}
namespace sim
{

class SimFMU : public mars::data_broker::ProducerInterface,
               public mars::data_broker::ReceiverInterface
{

  public:
    // Constructor and destructor
    SimFMU(interfaces::ControlCenter *control, const interfaces::FMUData &sFMU_);
    ~SimFMU();

    // function methods
    void activate(void);
    void deactivate(void);

    // getters
    const std::string getName() const;
    unsigned long getIndex(void) const;
    const std::vector<std::string> getInputs() const;
    const std::vector<std::string> getOutputs() const;
    const std::vector<std::string> getObserved() const;
    interfaces::sReal getStepSize();
    interfaces::sReal getMaximumStepSize();
    const std::string getFMUPath() const;
    const fmi2_string_t getFMUGUID() const;
    const fmi2_status_t getFMUStatus() const;

    // setters
    void setUpdateTime(interfaces::sReal step_size);
    void setFMUStatus(fmi2_status_t status);

    // methods inherited from data broker interfaces
    void getDataBrokerNames(std::string *groupName, std::string *dataName) const;

    virtual void produceData(const data_broker::DataInfo &info,
                             data_broker::DataPackage *package,
                             int callbackParam);

    virtual void receiveData(const data_broker::DataInfo &info,
                             const data_broker::DataPackage &package,
                             int callbackParam);

  private:
    // control Center
    interfaces::ControlCenter *control;
    interfaces::FMUData sFMU;
    // Configmap
    //configmaps::ConfigMap fmu_configMap;

    // Data broker
    data_broker::DataPackage producerPackage, receiverPackage;
    unsigned long producerID, receiverID;
    std::string groupName, dataName;
    //std::string producerData, receiverData;

    // Threading
    pthread_t fmu_thread;
    pthread_mutex_t fmu_thread_Mutex;
    bool thread_running, stop_thread, do_step;

    // Callbacks
    jm_callbacks callbacks;

    // FMI - Global variables used for managing
    fmi_import_context_t *context;
    fmi_version_enu_t version;

    fmi2_import_t *fmu;
    fmi2_callback_functions_t callBackFunctions;

    // FMI - Global variables used for simulation
    //fmi2_real_t fmu_relativeTolerance;

    fmi2_status_t fmu_status;
    jm_status_enu_t fmu_status_jm;

    // FMU Inputs
    //std::vector<unsigned long> mars_outputs;
    std::vector<fmi2_value_reference_t> fmu_inputs;
    //std::vector<std::string> fmu_input_names;
    std::vector<double> fmu_input_values;

    // FMU Outputs
    //std::vector<unsigned long> mars_inputs;
    std::vector<fmi2_value_reference_t> fmu_outputs;
    //std::vector<std::string> fmu_output_names;

    // FMU Observed
    std::vector<fmi2_value_reference_t> fmu_observed;
    //std::vector<std::string> fmu_observed_names;

    // Internal time management
    //interfaces::sReal max_step_size;
    //interfaces::sReal current_update_time;
    fmi2_real_t current_time;
    fmi2_real_t time_step;

    // Standard functions for simulation
    void init();
    void reset();

    // Threading
    //void update(mars::interfaces::sReal time_ms);
    void run();
    void setThreadStopped();

    // Simulation
    void stepSimulation();

    // Initialize the mapping
    //void readConfig();
    //void reateMapping();
    bool getVariableRef(std::string VariableName, int IO);
    void registerDataBroker();

    // Set the first values
    void SetInitialValues();
};
} // namespace sim
} // namespace mars

#endif