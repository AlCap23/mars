/*
 *  Copyright 2011, 2012, DFKI GmbH Robotics Innovation Center
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

/*
 *  IDListConfig.h
 *  QTVersion
 *
 *  Created by Malte Roemmermann on 28.03.12.
 *  Copyright 2012 __MyCompanyName__. All rights reserved.
 *
 */

#ifndef ID_LIST_CONFIG_H
#define ID_LIST_CONFIG_H

#ifdef _PRINT_HEADER_
#warning "IDListConfig.h"
#endif

#include <interfaces/MARSDefs.h>
#include <interfaces/SimulatorInterface.h>
#include <interfaces/ControlCenter.h>
#include <interfaces/LoadCenter.h>
#include <interfaces/LoadSceneInterface.h>
#include <interfaces/SensorInterface.h>

namespace mars {
  namespace sim {

    class IDListConfig : public interfaces::BaseConfig{
    public:
      IDListConfig(){
        name = "Unknown Sensor";
      }

      void parseConfig(interfaces::ControlCenter *control,
                       utils::ConfigMap *config,
                       interfaces::IDMapType mapType) {

        std::vector<utils::ConfigItem>::iterator it;
        unsigned long _id;
        utils::ConfigVector _ids = (*config)["id"];
        unsigned int mapIndex = (*config)["mapIndex"][0].getUInt();
        updateRate = (*config)["rate"][0].getULong();

        for(it=_ids.begin(); it!=_ids.end(); ++it) {
          if((_id = it->getULong())){
            if(mapIndex) {
              _id = control->loadCenter->loadScene->getMappedID(_id, mapType, mapIndex);
            }
            ids.push_back(_id);
          }
        }
      }

      std::vector<unsigned long> ids;
    };

  } // end of namespace sim
} // end of namespace mars

#endif
