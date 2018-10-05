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

#include "SimFMU.h"

#include <string>
#include <map>
#include <vector>

#include <configmaps/ConfigData.h>

namespace mars
{
namespace sim
{
using namespace utils;
using namespace interfaces;

// Declare the logger
void fmu_logger(jm_callbacks *callbacks, jm_string module, jm_log_level_enu_t log_level, jm_string message)
{
    printf("module = %s, log level = %s: %s\n", module, jm_log_level_to_string(log_level), message);
}

void *createFMUThread(void *theObject)
{
    // Run the thread
    ((SimFMU *)theObject)->activate();
    // Stop the thread
    ((SimFMU *)theObject)->deactivate();
    // Create a message
    fprintf(stderr, "Thread stopped! \n");
    pthread_exit(NULL);
    return 0;
}

SimFMU::SimFMU(ControlCenter *control, const FMUData &sFMU_) : control(control)
{
    sFMU.index = sFMU_.index;
    sFMU.name = sFMU_.name;
    sFMU.path = sFMU_.path;
    sFMU.tmp_path = sFMU_.tmp_path;

    sFMU.max_step_size = sFMU_.max_step_size;
    sFMU.step_size = sFMU_.step_size;
    sFMU.relativeTolerance = sFMU_.relativeTolerance;

    sFMU.input_names = sFMU_.input_names;
    sFMU.output_names = sFMU_.output_names;
    sFMU.observed_names = sFMU_.observed_names;

    // Init the data broker
    this->registerDataBroker();

    // Get the variables
    for (auto variable : sFMU.input_names)
    {
        if (this->getVariableRef(variable, 1))
        {
            receiverPackage.add("inputs/" + variable, 0.0);
        }
    }
    for (auto variable : sFMU.output_names)
    {
        if (this->getVariableRef(variable, 2))
        {
            producerPackage.add("outputs/" + variable, 0.0);
        }
    }
    for (auto variable : sFMU.observed_names)
    {
        if (this->getVariableRef(variable, 3))
        {

            producerPackage.add("observed/" + variable, 0.0);
        }
    }

    // Init the threading
    thread_running = false;
    stop_thread = false;
    do_step = false;
    pthread_mutex_init(&fmu_thread_Mutex, NULL);
    pthread_create(&fmu_thread, NULL, createFMUThread, (void *)this);
}

SimFMU::~SimFMU()
{
    // Stop the thread and wait for it to stop
    stop_thread = true;

    while (thread_running)
    {
#ifdef WIN32
        Sleep(10);
#else
        usleep(10000);
#endif
    }

    // Destroy the fmu
    fmi2_import_destroy_dllfmu(fmu);
    fmi2_import_free(fmu);
    fmi_import_free_context(context);

    // Free the temporary directory
    std::string delete_command = std::string("exec rm -r ") + sFMU.tmp_path + std::string("/*");
    system(delete_command.c_str());
    if (rmdir(sFMU.tmp_path.c_str()) == -1)
    {
        fprintf(stderr, "Error while removing Directory %s\n", sFMU.tmp_path.c_str());
        fprintf(stderr, "   %s\n", strerror(errno));
    }

    // Unregister the data broker
    control->dataBroker->unregisterTimedProducer(this, "*", "*", "mars_sim/simTimer");
    control->dataBroker->unregisterTimedReceiver(this, "*", "*", "mars_sim/simTimer");
}

bool SimFMU::getVariableRef(std::string VariableName, int IO)
{
    // TODO register as timed producer!
    fmi2_import_variable_t *vr_pointer = fmi2_import_get_variable_by_name(fmu, VariableName.c_str());
    if (vr_pointer)
    {
        if (IO == 1)
        {
            // Here the inputs to the fmu get stored
            fprintf(stderr, " FMU Input : %s\n", VariableName.c_str());
            sFMU.inputs.push_back(fmi2_import_get_variable_vr(vr_pointer));
        }
        else if (IO == 2)
        {
            fprintf(stderr, " FMU Output : %s\n", VariableName.c_str());
            sFMU.outputs.push_back(fmi2_import_get_variable_vr(vr_pointer));
        }
        else if (IO == 3)
        {
            fprintf(stderr, " FMU Observed : %s\n", VariableName.c_str());
            sFMU.observed.push_back(fmi2_import_get_variable_vr(vr_pointer));
        }
        return 1;
    }
    else
    {
        fprintf(stderr, "No variable named %s found in FMU\n", VariableName.c_str());
        return 0;
    }
}

void SimFMU::registerDataBroker()
{
    getDataBrokerNames(&groupName, &dataName);

    producerID = control->dataBroker->pushData(groupName, dataName, producerPackage, NULL, mars::data_broker::DATA_PACKAGE_READ_FLAG);

    control->dataBroker->registerTimedProducer(this, groupName, dataName,
                                               "mars_sim/simTimer",
                                               time_step);

    receiverID = control->dataBroker->pushData(groupName, "cmd/" + dataName, receiverPackage, NULL, mars::data_broker::DATA_PACKAGE_READ_WRITE_FLAG);

    control->dataBroker->registerTimedReceiver(this, groupName, "cmd/" + dataName, "mars_sim/simTimer", time_step);
}

} // namespace sim
} // namespace mars