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
                // Get the target time and store it
                fmu_targetTime.push_back((fmu_models.back())->getTargetTime());
            }
        }

        this->setStepSize();
        this->setUpdateInterval();

        this->registerDataBroker();

        // Start mars thread
        thread_running = true;
        start();
    }
    else
    {
        fprintf(stderr, "No configuration for mars_fmu found! \n");
        return;
    }
}

fmuMaster::~fmuMaster()
{
    thread_running = false;

    while (isRunning())
    {
        msleep(1);
    }

    for (auto fmus : fmu_models)
    {
        fmus->~fmuNode();
    }

    control->dataBroker->unregisterTimedReceiver(this, "*", "*",
                                                 "mars_fmu/simTimer");
}

void fmuMaster::reset()
{
    for (auto fmu : fmu_models)
    {
        fmu->reset();
    }
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
        fprintf(stderr, "Register fmu at data broker \n");
        double interval = fmu->getStepSize() / step_size;
        fmu->RegisterDataBroker((int)interval);
    }
}

bool fmuMaster::getWait()
{
    return mars_wait;
}

void fmuMaster::run()
{
    current_time = 0.0;
    current_mars_time = 0.0;
    mars_wait = false;
    while (thread_running)
    {

        // Iterator
        std::vector<fmi2_real_t *>::iterator i = fmu_targetTime.begin();

        // Set the next target to inf
        next_target = INFINITY;

        // Find the minimal next value
        for (; i != fmu_targetTime.end(); i++)
        {
            if (*(*i) <= next_target)
            {
                next_target = *(*i);
            }
        }

        // Step the timer if next target is in future
        if (current_time + step_size <= next_target || mars_wait)
        {
            //fprintf(stderr, "Next target : %g \n", next_target);
            double steps = (next_target - current_time) / step_size;
            if (steps <= 0)
            {
                steps = 1;
            }

            control->dataBroker->stepTimer("mars_fmu/simTimer", (int)steps);
            current_time = next_target;
            producerPackage.set(0, current_time);
            producerID = control->dataBroker->pushData(producerGroup, producerData,
                                                       producerPackage, NULL,
                                                       mars::data_broker::DATA_PACKAGE_READ_FLAG);
        }
        else
        {
            //fprintf(stderr, "Sleep \n");
            msleep(1);
        }
        mars_wait = current_mars_time > current_time;
    }
}

void fmuMaster::registerDataBroker()
{
    producerGroup = "mars_fmu";
    producerData = "simTimer";
    producerPackage.add("simTime", current_time);

    receiverGroup = "mars_sim";
    receiverData = "simTime";

    control->dataBroker->registerTimedReceiver(this, receiverGroup, receiverData,
                                               "mars_fmu/simTimer",
                                               0);
    control->dataBroker->registerTimedReceiver(this, receiverGroup, receiverData,
                                               "mars_sim/simTimer",
                                               1);
}

void fmuMaster::produceData(const mars::data_broker::DataInfo &info,
                            mars::data_broker::DataPackage *package,
                            int callbackParam)
{
    package->set(0, current_time);
};

void fmuMaster::receiveData(const mars::data_broker::DataInfo &info,
                            const mars::data_broker::DataPackage &package,
                            int callbackParam)
{
    package.get(0, &current_mars_time);
    // Set to seconds
    current_mars_time *= 1e-3;
};
