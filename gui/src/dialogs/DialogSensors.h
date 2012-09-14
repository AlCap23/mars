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


#ifndef DIALOGSENSORS_H
#define DIALOGSENSORS_H

#ifdef _PRINT_HEADER_
#warning "DialogSensors.h"
#endif

#include <main_gui/BaseWidget.h>
#include <main_gui/PropertyDialog.h>
#include <interfaces/ControlCenter.h>
#include "SensorHandler.h"

namespace mars {

  namespace main_gui {
    class GuiInterface;
  }

  namespace gui {

    class DialogSensors : public main_gui::BaseWidget,
                          public main_gui::PropertyCallback {
      Q_OBJECT
    
      public:
      DialogSensors(interfaces::ControlCenter *c, main_gui::GuiInterface *gui);
      ~DialogSensors();
  
      main_gui::PropertyDialog *pDialog;  
  
      void show(void) {pDialog->show();}
      void hide(void) {pDialog->hide();}
      bool isHidden(void) {return pDialog->isHidden();}
      void close(void) {pDialog->close();}
    
    private:
      virtual void topLevelItemChanged(QtProperty* current);
      virtual void valueChanged(QtProperty *property, const QVariant &value);

      QtProperty *oldFocus; 
      bool filled;
      QPushButton *addButton;
      QPushButton *removeButton;
      QPushButton *objectsButton;
      std::vector<SensorHandler*> allDialogs;
      std::vector<SensorHandler*> newDialogs;
      std::vector<QtProperty*> allSensors_p;
      std::vector<QtProperty*> newSensors_p;
      std::vector<interfaces::core_objects_exchange, Eigen::aligned_allocator<interfaces::core_objects_exchange> > allSensors;
      interfaces::ControlCenter* control;
      main_gui::GuiInterface *mainGui;

    private slots:
      void on_new_sensor();
      void on_remove_sensor();
      void on_add_sensor();
      void on_objects_selection();
      void closeDialog();
    };
  } // end of namespace gui
} // end of namespace mars

#endif // DIALOGSENSORS_H
