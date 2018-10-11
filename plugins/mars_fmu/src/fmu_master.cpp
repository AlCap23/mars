#include "fmu_master.h"

using namespace mars::utils;
using namespace mars::interfaces;

fmi2_real_t gcd(fmi2_real_t a, fmi2_real_t b)
{
    if (a < b)
        return gcd(b, a);

    if (fabs(b) < 1e-9)
        return a;
    else
        return (gcd(b, a - floor(a / b) * b));
}

fmuMaster::fmuMaster(ControlCenter *controlCenter)
{
    control = controlCenter;
    fprintf(stderr, "\nStart FMU Master algorithm \n");

    // Get the configmap
    std::string confPath = control->cfg->getOrCreateProperty("Config", "config_path", ".").sValue;
    if (pathExists(confPath + "/mars_fmu.yml"))
    {

        // Create a timer
        fprintf(stderr, "   Create fmu timer \n");
        control->dataBroker->createTimer("mars_fmu/simTimer");

        confPath = pathJoin(confPath, "mars_fmu.yml");
        fprintf(stderr, "   FMU Config found : %s \n", confPath.c_str());

        // Read the Config Map
        configmaps::ConfigMap map = configmaps::ConfigMap::fromYamlFile(confPath);
        if (map.hasKey("marsFMU"))
        {

            marsfmu_Config = map["marsFMU"];
            for (auto fmus : marsfmu_Config)
            {
                printf("        Initialize FMU  %s \n", fmus.first.c_str());
                configmaps::ConfigMap current_fmu = marsfmu_Config[fmus.first];
                // Create a new fmu and store it
                fmu_models.push_back(new fmuNode(fmus.first, current_fmu, control));
                // Get status and store it
                //fmu_status.push_back((fmu_models.back())->getStatus());
                // Get the target time and store it
                //fmu_targetTime.push_back((fmu_models.back())->getTargetTime());
            }
        }

        //this->setStepSize();
        //this->setUpdateInterval();

        // Set the time
        current_time = 0.0;
    }
    else
    {
        fprintf(stderr, "No configuration for mars_fmu found! \n");
        return;
    }
}

fmuMaster::~fmuMaster()
{
    for (auto fmus : fmu_models)
    {
        fmus->~fmuNode();
    }
}

void fmuMaster::reset()
{
    for (auto fmu : fmu_models)
    {
        fmu->reset();
    }
}

// TODO dies das

void fmuMaster::update(mars::interfaces::sReal update_time)
{
    // Start iterators
    std::vector<fmuNode *>::iterator i = fmu_models.begin();
    std::vector<int *>::iterator j = fmu_status.begin();
    std::vector<fmi2_real_t *>::iterator k = fmu_targetTime.begin();
    bool fmu_status_ = false;
    fmi2_real_t fmu_targetTime_ = NAN;

    // Iterate over the fmus
    //for (; i != fmu_models.end() && j != fmu_status.end() && k != fmu_targetTime.end(); i++, j++, k++)
    //{
    //    fmu_status_ = checkStatus(*j);
    //}
}

void fmuMaster::setStepSize()
{
    // Get the step time
    std::vector<fmi2_real_t> time_steps = {};

    for (auto fmu : fmu_models)
    {
        time_steps.push_back(fmu->getStepSize());
    }

    // Find the step size
    std::vector<fmi2_real_t>::iterator i = time_steps.begin();
    step_size = *i;
    i++;

    for (; i != time_steps.end(); i++)
    {
        step_size = gcd(step_size, *i);
    }
    if (step_size < 1e-9)
    {
        fprintf(stderr, "   Minimal step size is 1 nanosecond, setting step size accordingly. \n");
        step_size = 1e-9;
    }

    fprintf(stderr, "   Simulation step size is %g \n", step_size);
}

void fmuMaster::setUpdateInterval()
{
    for (auto fmu : fmu_models)
    {
        double interval = fmu->getStepSize() / step_size;
        fmu->RegisterDataBroker((int)interval);
    }
}

bool fmuMaster::checkStatus(int *status)
{
    // Check the status
    if (*status == -1)
    {
        fprintf(stderr, "   FMU error \n");
        return false;
    }
    else if (*status == 1)
    {
        fprintf(stderr, "   FMU stepping \n");
        return true;
    }
    else if (*status == 2)
    {
        fprintf(stderr, "   FMU finished\n");
        return true;
    }
    else
    {
        fprintf(stderr, "   Unknown state");
        return true;
    }
}