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
 * \file MarsFmu.h
 * \author Julius (julius.martensen@dfki.de)
 * \brief A
 *
 * Version 0.1
 */

#ifndef MARS_PLUGINS_MARS_FMU_H
#define MARS_PLUGINS_MARS_FMU_H

#ifdef _PRINT_HEADER_
#warning "MarsFmu.h"
#endif

// set define if you want to extend the gui
//#define PLUGIN_WITH_MARS_GUI
#include <mars/utils/misc.h>
#include <mars/interfaces/sim/MarsPluginTemplate.h>
#include <mars/interfaces/MARSDefs.h>

#include <mars/cfg_manager/CFGManagerInterface.h>

// Use to publish the results and get the forces
#include <mars/data_broker/ProducerInterface.h>
#include <mars/data_broker/ReceiverInterface.h>

#include <mars/interfaces/sim/MotorManagerInterface.h>
#include <mars/interfaces/sim/SensorManagerInterface.h>

#include <string>

#include "fmu_master.h"
#include "fmu_node.h"
#include "fmilib.h"

namespace mars
{

namespace plugins
{
namespace mars_fmu
{

// inherit from MarsPluginTemplateGUI for extending the gui
class MarsFmu : public mars::interfaces::MarsPluginTemplate,

                // for gui
                // public mars::main_gui::MenuInterface,
                public mars::cfg_manager::CFGClient
{

public:
  MarsFmu(lib_manager::LibManager *theManager);
  ~MarsFmu();

  // LibInterface methods
  int getLibVersion() const
  {
    return 1;
  }
  const std::string getLibName() const
  {
    return std::string("mars_fmu");
  }
  CREATE_MODULE_INFO();

  // MarsPlugin methods
  void init();
  void reset();
  void update(mars::interfaces::sReal time_ms);

  // DataBrokerReceiver methods
  //virtual void receiveData(const data_broker::DataInfo &info, const data_broker::DataPackage &package, int callbackParam);
  //virtual void produceData(const data_broker::DataInfo &info,
  //                         data_broker::DataPackage *package,
  //                         int callbackParam);
  //// CFGClient methods
  virtual void cfgUpdateProperty(cfg_manager::cfgPropertyStruct _property);

  // MenuInterface methods
  //void menuAction(int action, bool checked = false);

  // MarsFmu methods

private:
  fmuMaster *Master;

}; // end of class definition MarsFmu

} // end of namespace mars_fmu
} // end of namespace plugins
} // end of namespace mars

#endif // MARS_PLUGINS_MARS_FMU_H
