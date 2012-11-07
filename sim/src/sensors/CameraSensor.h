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

#ifndef CAMERA_SENSOR_H
#define CAMERA_SENSOR_H

#ifdef _PRINT_HEADER_
#warning "CameraSensor.h"
#endif

#include <interfaces/ReceiverInterface.h>

#include <interfaces/sim/SensorInterface.h>
#include <utils/Vector.h>
#include <utils/Quaternion.h>
#include <interfaces/graphics/GraphicsWindowInterface.h>
#include <interfaces/graphics/GraphicsUpdateInterface.h>
#include <interfaces/graphics/GraphicsCameraInterface.h>

namespace mars {

  namespace graphics {
    class GraphicsWindowInterface;
    class GraphicsCameraInterface;
  }

  namespace sim {


    struct CameraConfigStruct: public interfaces::BaseConfig{
      CameraConfigStruct(){
        name = "Unknown Camera";
        width=640;
        height=480;
        show_cam = false;
        hud_pos=0;
        pos_offset.setZero();
        ori_offset.setIdentity();
        opening_width=90;
        opening_height=90;
        hud_width = 320;
        hud_height = 240;
      }

      unsigned long attached_node;
      int width;
      int height;
      bool show_cam;
      int hud_pos;
      utils::Vector pos_offset;
      utils::Quaternion ori_offset;
      double opening_width;
      double opening_height;
      int hud_width;
      int hud_height;
    };

    class CameraSensor : public interfaces::BaseNodeSensor,
                         public interfaces::SensorInterface,
                         public data_broker::ReceiverInterface,
                         public interfaces::GraphicsUpdateInterface {
    public:
      static interfaces::BaseSensor* instanciate(interfaces::ControlCenter *control,
                                           interfaces::BaseConfig *config );
      CameraSensor(interfaces::ControlCenter *control, CameraConfigStruct config);
      ~CameraSensor(void);

      virtual int getSensorData(interfaces::sReal** data) const;
      virtual void receiveData(const data_broker::DataInfo &info,
                               const data_broker::DataPackage &package,
                               int callbackParam);

      virtual void preGraphicsUpdate(void);
      void getCameraInfo( interfaces::cameraStruct *cs );

      static interfaces::BaseConfig* parseConfig(interfaces::ControlCenter *control,
                                     utils::ConfigMap *config);
      virtual utils::ConfigMap createConfig() const;

    private:
      CameraConfigStruct config;
      interfaces::BaseCameraSensor<double> depthCamera;
      interfaces::BaseCameraSensor<char*> imageCamera;
      unsigned long cam_window_id;
      interfaces::GraphicsWindowInterface *gw;
      interfaces::GraphicsCameraInterface* gc;
      long dbPosIndices[3];
      long dbRotIndices[4];
    };

  } // end of namespace sim
} // end of namespace mars

#endif
