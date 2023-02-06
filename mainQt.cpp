/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Anton Deguet
  Created on: 2017-03-21

  (C) Copyright 2017-2023 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#include <cisstCommon/cmnPath.h>
#include <cisstCommon/cmnUnits.h>
#include <cisstCommon/cmnCommandLineOptions.h>
#include <cisstCommon/cmnQt.h>
#include <cisstMultiTask/mtsTaskManager.h>
#include <sawSensablePhantom/mtsSensableHD.h>
#include <sawSensablePhantom/mtsSensableHDQtWidget.h>

#include <cisst_ros2_crtk/mts_ros_crtk_bridge_provided.h>

#include <QApplication>
#include <QMainWindow>


int main(int argc, char * argv[])
{
    // log configuration
    cmnLogger::SetMask(CMN_LOG_ALLOW_ALL);
    cmnLogger::SetMaskFunction(CMN_LOG_ALLOW_ALL);
    cmnLogger::SetMaskDefaultLog(CMN_LOG_ALLOW_ALL);
    cmnLogger::SetMaskClassMatching("mtsSensable", CMN_LOG_ALLOW_ALL);
    cmnLogger::AddChannel(std::cerr, CMN_LOG_ALLOW_ERRORS_AND_WARNINGS);

    // create ROS node handle
    std::vector<std::string> non_ros_arguments = rclcpp::init_and_remove_ros_arguments(argc, argv);
    auto rosNode = std::make_shared<rclcpp::Node>("SensableOmni");

    // parse options
    cmnCommandLineOptions options;
    std::string jsonConfigFile = "";
    double rosPeriod = 2.0 * cmn_ms;
    double tfPeriod = 20.0 * cmn_ms;
    std::list<std::string> managerConfig;

    options.AddOptionOneValue("j", "json-config",
                              "json configuration file",
                              cmnCommandLineOptions::OPTIONAL_OPTION, &jsonConfigFile);
    options.AddOptionOneValue("p", "ros-period",
                              "period in seconds to read all tool positions (default 0.002, 2 ms, 500Hz).  There is no point to have a period higher than the device",
                              cmnCommandLineOptions::OPTIONAL_OPTION, &rosPeriod);
    options.AddOptionOneValue("P", "tf-ros-period",
                              "period in seconds to read all components and broadcast tf2 (default 0.02, 20 ms, 50Hz).",
                              cmnCommandLineOptions::OPTIONAL_OPTION, &tfPeriod);
    options.AddOptionMultipleValues("m", "component-manager",
                                    "JSON files to configure component manager",
                                    cmnCommandLineOptions::OPTIONAL_OPTION, &managerConfig);
    options.AddOptionNoValue("D", "dark-mode",
                             "replaces the default Qt palette with darker colors");

    // check that all required options have been provided
    if (!options.Parse(non_ros_arguments, std::cerr)) {
        return -1;
    }
    std::string arguments;
    options.PrintParsedArguments(arguments);
    std::cout << "Options provided:" << std::endl << arguments << std::endl;

    // create the components
    mtsSensableHD * devices = new mtsSensableHD("SensableHD");
    devices->Configure(jsonConfigFile);

    // add the components to the component manager
    mtsManagerLocal * componentManager = mtsComponentManager::GetInstance();
    componentManager->AddComponent(devices);

    // ROS CRTK bridge
    mts_ros_crtk_bridge_provided * crtk_bridge
        = new mts_ros_crtk_bridge_provided("sensable_phantom_crtk_bridge", rosNode);
    componentManager->AddComponent(crtk_bridge);

    // create a Qt user interface
    QApplication application(argc, argv);
    cmnQt::QApplicationExitsOnCtrlC();
    if (options.IsSet("dark-mode")) {
        cmnQt::SetDarkMode();
    }

    // organize all widgets in a tab widget
    QTabWidget * tabWidget = new QTabWidget;
    mtsSensableHDQtWidget * deviceWidget;

    // configure all components
    std::vector<std::string> deviceNames;
    deviceNames = devices->DeviceNames(); // get names of all sensable devices
    for (size_t index = 0;
         index < deviceNames.size();
         ++index) {
        std::string name = deviceNames.at(index);
        deviceWidget = new mtsSensableHDQtWidget(name + "-gui");
        deviceWidget->Configure();
        componentManager->AddComponent(deviceWidget);
        componentManager->Connect(deviceWidget->GetName(), "Device",
                                  devices->GetName(), name);
        tabWidget->addTab(deviceWidget, name.c_str());
        crtk_bridge->bridge_interface_provided(devices->GetName(),
                                               name, rosPeriod, tfPeriod);
    }
    crtk_bridge->Connect();

    // custom user component
    if (!componentManager->ConfigureJSON(managerConfig)) {
        CMN_LOG_INIT_ERROR << "Configure: failed to configure component-manager, check cisstLog for error messages" << std::endl;
        return -1;
    }

    // create and start all components
    componentManager->CreateAllAndWait(5.0 * cmn_s);
    componentManager->StartAllAndWait(5.0 * cmn_s);

    // run Qt user interface
    tabWidget->show();
    application.exec();

    // stop all logs
    cmnLogger::Kill();

    // stop ROS node
    rclcpp::shutdown();

    // kill all components and perform cleanup
    componentManager->KillAllAndWait(5.0 * cmn_s);
    componentManager->Cleanup();

    return 0;
}
