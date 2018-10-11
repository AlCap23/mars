/*
 *  Copyright 2013, DFKI GmbH Robotics Innovation Center
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

/**
 * \file MarsFmu.cpp
 * \author Julius (julius.martensen@dfki.de)
 * \brief A
 *
 * Version 0.1
 */

#include "MarsFmu.h"
#include <mars/data_broker/DataBrokerInterface.h>
#include <mars/data_broker/DataPackage.h>

namespace mars
{
namespace plugins
{
namespace mars_fmu
{

using namespace mars::utils;
using namespace mars::interfaces;

MarsFmu::MarsFmu(lib_manager::LibManager *theManager)
    : MarsPluginTemplate(theManager, "MarsFmu")
{
}

void MarsFmu::init()
{
  Master = new fmuMaster(control);
}

void MarsFmu::reset()
{
  Master->reset();
}

MarsFmu::~MarsFmu()
{
  Master->~fmuMaster();
}

void MarsFmu::update(sReal time_ms)
{
  while (Master->getWait())
  {
    msleep(1);
  }
  fprintf(stderr, "Finished step! \n");
}

void MarsFmu::produceData(const data_broker::DataInfo &info,
                          data_broker::DataPackage *package,
                          int callbackParam)
{
}

void MarsFmu::receiveData(const data_broker::DataInfo &info, const data_broker::DataPackage &package, int callbackParam)
{
  // package.get("force1/x", force);
}

void MarsFmu::cfgUpdateProperty(cfg_manager::cfgPropertyStruct _property)
{
}

} // end of namespace mars_fmu
} // end of namespace plugins
} // end of namespace mars

DESTROY_LIB(mars::plugins::mars_fmu::MarsFmu);
CREATE_LIB(mars::plugins::mars_fmu::MarsFmu);
