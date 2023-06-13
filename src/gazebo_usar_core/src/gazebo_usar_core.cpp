#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <memory>
#include "hazardouszones.h"

namespace gazebo
{
/*!
  \brief
  Class GazeboUsarCore contains functions for <b><gazebo_usar_core></b> plugin loading and tuning.
*/
class GazeboUsarCore : public WorldPlugin
{
public:
  GazeboUsarCore() : WorldPlugin()
  {
    ROS_INFO("Loading GazeboUsarCorePlugin...\n");
  }

public:
  ~GazeboUsarCore() override
  {
    plugin_ros_node->shutdown();
    ROS_INFO("Finished:(\n");
  }

public:
  void Load(physics::WorldPtr gazebo_world, sdf::ElementPtr gazebo_world_sdf) override
  {
    plugin_ros_node.reset(new ros::NodeHandle("gazebo_usar_core"));

    contaminations_manager = std::make_unique<HazardousZones>(plugin_ros_node, gazebo_world, gazebo_world_sdf);

    gazebo_update_connection = event::Events::ConnectWorldUpdateBegin(
              std::bind(&GazeboUsarCore::OnUpdate, this));

    ROS_INFO("GazeboUsarCorePlugin loaded!\n");
  }

public:
  void OnUpdate()
  {    

    // first update needed for parsing parameters from loaded World file
    if(first_update_done)
    {
      ROS_WARN("Plugin tunning start...\n");
      contaminations_manager->SearchRadiationZones();
      contaminations_manager->SearchChemicalZones();
      first_update_done = false;
      ROS_INFO("Hazardous zones parameters are parsed\n");
    }

    // models are not loaded right away
    // so a number of iterations of the simulator update are needed
    if(!all_models_found)
    {
      all_models_found = contaminations_manager->SearchRobotModels();
      if(all_models_found)
      {
        ROS_INFO("All robot models are found.\n");
        ROS_INFO("Starting...\n");
        contaminations_manager->Run();
      }
    }
  }

private:
  /*! @brief Shared Pointer to ROS Node */
  ros::NodeHandlePtr plugin_ros_node;
  /*! @brief Shared Pointer to Gazebo event connection */
  event::ConnectionPtr gazebo_update_connection;
  /*! @brief Shows the status that all robot models are loaded and found */
  bool all_models_found = false;
  /*! @brief Shows the status that the first update is done */
  bool first_update_done = true;
  /*! @brief Contaminations manager instance */
  std::unique_ptr<HazardousZones> contaminations_manager;
};
GZ_REGISTER_WORLD_PLUGIN(GazeboUsarCore)
} // namespace gazebo

