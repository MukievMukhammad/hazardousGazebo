/*
 * Copyright (C) 2020 Laboratory of Intelligent Robotic Systems (LIRS)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/
#ifndef GAZEBO_USAR_GUI_HH_
#define GAZEBO_USAR_GUI_HH_

#include <QProgressBar>
#include <string>
#include <ros/callback_queue.h>
#include <ros/subscribe_options.h>
#include <thread>
#include "gazebo_usar_core/RadiationSubstance.h"
#include "gazebo_usar_core/RadiationSubstances.h"
#include "gazebo_usar_core/ChemicalSubstance.h"
#include "gazebo_usar_core/ChemicalSubstances.h"
#include "gazebo_usar_core/ChemicalConcentration.h"
#include <ros/ros.h>
#include <gazebo/common/Plugin.hh>
#include <gazebo/gui/GuiPlugin.hh>
// moc parsing error of tbb headers
#ifndef Q_MOC_RUN
#include <gazebo/transport/transport.hh>
#endif

namespace gazebo
{
  /*!
   \brief
    Class GazeboUsarGui contains methods to show incoming messages
    from contamination topics in a beautiful and pleasant form using Qt widgets.
  */
  class GAZEBO_VISIBLE GazeboUsarGui : public GUIPlugin
  {
    Q_OBJECT

    signals:
      /*!
        \brief
        Qt signal that updating chemical progressbars.
      */
      void updateChemicalProgressBarsSignal();
      /*!
        \brief
        Qt signal that updating radiation progressbars.
      */
      void updateRadiationProgressBarsSignal();
      /*!
        \brief
        Qt signal that updating radiation progressbars.
      */
      void updateChemicalConcentrationProgressBarsSignal();

    public slots:
      /*!
        \brief
        Qt slot that updating chemical progressbars.
      */
      void updateChemicalProgressBars();
      /*!
        \brief
        Qt slot that updating radiation progressbars.
      */
      void updateRadiationProgressBars();
      /*!
        \brief
        Qt slot that updating chemical progressbars.
      */
      void updateChemicalConcentrationProgressBars();

    public:
      /*!
        \brief
        Load function.
        Called when the GUI plugin is first loaded.
        \param[in] sdfPtr: The GUI plugin descriptor. Contains <plugin> data and its children.
      */
      void Load(sdf::ElementPtr sdfPtr) override;

      /*!
        \brief
        The constructor of GazeboUsarGui class.
      */
      GazeboUsarGui();

      /*!
        \brief
        The destructor of GazeboUsarGui class.
      */
      ~GazeboUsarGui() override;

    private:
      /*!
        \brief
        Parses GUI plugin's <robots> tag and receives names of the robots.
        \param[in] robots_data: Data that contains names of the robots. The <b>; delimeter</b> is used to separate names from each other.
      */
      void parseRobotsTagData(std::string &robots_data);

      /*!
        \brief
        Callback function that receives messages from radiation topicof the <b> gazebo_usar_core </b> plugin.
        \param[out] msg: Shared Pointer that contains a new arrived message of the <b> /radiation_substances </b> topic
      */
      void receiveRadiationData(const gazebo_usar_core::RadiationSubstancesConstPtr& msg);

      /*!
        \brief
        Callback function that receive messages from chemical topic of the <b> gazebo_usar_core </b> plugin.
        \param[out] msg: Shared Pointer that contains a new arrived message of the <b> /chemical_substances </b> topic
      */
      void receiveChemicalData(const gazebo_usar_core::ChemicalSubstancesConstPtr& msg);

      /*!
        \brief
        Callback function that receive messages from chemical topic of the <b> gazebo_usar_core </b> plugin.
        \param[out] msg: Shared Pointer that contains a new arrived message of the <b> /chemical_substances </b> topic
      */
      void receiveChemicalConcentration(const gazebo_usar_core::ChemicalConcentrationConstPtr& msg);

      /*!
        \brief
        Visualizes Qt widgets on the left side of the Gazebo simulator.
        \param[in] robots_names: A list of names of the robots parsed from <b> <robots> </b> tag.
      */
      void visualizeWidgets(std::vector<std::string>& robots_names);

      /*!
        \brief
        Thread function that handles incoming messages of the <b> /radiation_substances </b> topic.
      */
      void radiationQueueThread();

      /*!
        \brief
        Thread function that handles incoming messages of the <b> /chemical_substances </b> topic.
      */
      void chemicalQueueThread();

      /*!
        \brief
        Thread function that handles incoming messages of the <b> /chemical_concentration </b> topic.
      */
      void chemicalConcentrationQueueThread();


    /*! @brief A node used to establish communication with gzserver. */
    transport::NodePtr node;
    /*! @brief Shared Pointer for creating subscribers of contamination topics. */
    ros::NodeHandlePtr ros_node;
    /*! @brief ROS Subscribers to receive contamination data. */
    ///@{
    ros::Subscriber radiation_sub, chemical_sub, chemical_concentration_sub;
    ///@}
    /*! @brief ROS CallbackQueues to handle contamination receiving callbacks. */
    ///@{
    ros::CallbackQueue radiation_queue, chemical_queue, chemical_concentration_queue;
    ///@}
    /*! @brief std::thread objects to start handling incoming messages. */
    ///@{
    std::thread radiation_thread, chemical_thread, chemical_concentration_thread;
    ///@}
    /*! @brief A list of names of the robots */
    std::vector<std::string> robots_list;
    /*! @brief Contains accumulated radiation substances data of each robot. */
    QHash<QString, std::shared_ptr<QLabel>> radiation_data_widget;
    /*! @brief Contains accumulated chemical substances data of each robot. */
    QHash<QString, std::shared_ptr<QLabel>> chemical_data_widget;
    /*! @brief Contains accumulated chemical substances data of each robot. */
    QHash<QString, std::shared_ptr<QLabel>> chemical_sensor_data_widget;
    /*! @brief Contains the QProgressbar radiation widget of each robot. */
    QHash<QString, std::shared_ptr<QProgressBar>> radiation_visualize_widget;
    /*! @brief Contains the QProgressbar chemical widget of each robot. */
    QHash<QString, std::shared_ptr<QProgressBar>> chemical_visualize_widget;
    /*! @brief Contains the QProgressbar chemical widget of each robot. */
    QHash<QString, std::shared_ptr<QProgressBar>> chemical_sensor_visualize_widget;
    /*! @brief Contains info about all accumulated radiation contamination data of each robot. */
    gazebo_usar_core::ChemicalSubstances chemical_substances_data;
    /*! @brief Contains info about all accumulated chemical contamination data of each robot. */
    gazebo_usar_core::RadiationSubstances radiation_substances_data;
    /*! @brief Contains info about all accumulated radiation contamination data of each robot. */
    gazebo_usar_core::ChemicalConcentration chemical_concentration;
    /*! @brief A path to the gazebo plugins resources (icons, images, etc.). */
    QString resources_path;
    /*! @brief A timeout which lets specify the amount of time to wait for a callback to be available. <b> Measured in seconds. </b> */
    double callback_wait_time = 1.5;
    /*! @brief Determines whether parsing robots names is finished successfully. <b> Default false </b> */
    bool robots_names_parsed_succeed = false;
  };
}
#endif // GAZEBO_USAR_GUI_HH_
