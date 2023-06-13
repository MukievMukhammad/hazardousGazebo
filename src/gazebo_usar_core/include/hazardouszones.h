#ifndef HAZARDOUS_ZONES_H
#define HAZARDOUS_ZONES_H

#include "chemical_gas_propogation.h"

#include "gazebo_usar_core/RadiationSubstance.h"
#include "gazebo_usar_core/RadiationSubstances.h"
#include "gazebo_usar_core/ChemicalSubstance.h"
#include "gazebo_usar_core/ChemicalSubstances.h"
#include "gazebo_usar_core/ChemicalConcentration.h"

#include <ignition/math/Vector3.hh>
#include <ignition/math.hh>
#include <ignition/math/Quaternion.hh>

#include "gazebo/physics/physics.hh"
#include "gazebo/common/common.hh"
#include "gazebo/gazebo.hh"
#include "gazebo/physics/Model.hh"
#include <ros/ros.h>
#include "nav_msgs/OccupancyGrid.h"

#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>

#include <map>
#include <vector>
#include <sstream>
#include <thread>
#include <boost/shared_ptr.hpp>
#include <algorithm>
#include <boost/algorithm/string.hpp>

#include <dynamic_reconfigure/server.h>
#include <gazebo_usar_core/ContaminationConfig.h>

#include <boost/thread/thread.hpp>
#include <boost/asio.hpp>
#include <boost/array.hpp>

/*!
 \brief
 Config of the contamination map.
*/
typedef struct _CONTAMINATION_MAP_CONFIG
{
  /*! @brief Map resolution [m/cell]. */
  float Resolution;
  /*! @brief Map width and height [cells]. */
  ///@{
  uint32_t Width, Height;
  ///@}
  /*! @brief Map min and max values of pixels in range [0...255]. */
  ///@{
  uint8_t Max, Min;
  ///@}
} RADIATION_MAP_CONFIG, CHEMICAL_MAP_CONFIG;

/*!
 \brief
 Description of the radiation zone.
*/
typedef struct _RADIATION_ZONE
{
  /*! @brief Sim-world pose of the zone. */
  ignition::math::Vector3d Origin;
  /*! @brief Name of the zone. */
  std::string Name;
  /*! @brief Radius of the zone. */
  double Radius;
} RADIATION_ZONE;

/*!
 \brief
 Description of the chemical zone.
*/
typedef struct _CHEMICAL_ZONE
{
  /*! @brief Sim-world pose of the zone. */
  ignition::math::Vector3d Position;
  /*! @brief Real-world rot of the zone. */
  ignition::math::Quaterniond Rotation;
  /*! @brief Name of the zone. */
  std::string Name;
  /*! @brief Sizes of the zone. */
  ///@{
  double Lenght, Width, Height;
  ///@}
  /*! @brief Values of each boundary. */
  ///@{
  double X_MIN, X_MAX, Y_MIN, Y_MAX, Z_MIN, Z_MAX;
  ///@}
  /*! @brief Median radius of the zone. <b> Calculeted as (Lenght/2 + Width/2 + Height/2)/2. </b> */
  double MedianRadius;
} CHEMICAL_ZONE;

/*!
 \brief
 Config of the robot data for further radiation substance calculation.
*/
typedef struct _RADIATION_CONFIG
{
  /*! @brief Sim-world robot pose. */
  ignition::math::Vector3d *robot_pose;
  /*! @brief Accumulated radiation substance data. */
  gazebo_usar_core::RadiationSubstance *data;
} RADIATION_CONFIG;

/*!
 \brief
 Config of the robot data for further chemical substance calculation.
*/
typedef struct _CHEMICAL_CONFIG
{
  /*! @brief Sim-world robot pose. */
  ignition::math::Vector3d *robot_pose;
  /*! @brief Accumulated chemical substance data. */
  gazebo_usar_core::ChemicalSubstance *data;
} CHEMICAL_CONFIG;

/*!
 \brief
  Class HazardousZones contains functions to simulate contaminations.
  <b> Now supports Radiation and Chemical influence imitation. </b>
*/
class HazardousZones
{
public:
  /*!
    \brief
    Constructor of HazardousZones class
    \param[in] rosNodePtr: ROS Node instance
    \param[in] worldPtr: Virtual world instance
    \param[in] sdfPtr: Virtual world descriptor
  */
  HazardousZones(ros::NodeHandlePtr plugin_ros_node, gazebo::physics::WorldPtr gazebo_world, sdf::ElementPtr gazebo_world_sdf);

  /*!
    \brief
    Destructor of HazardousZones class
  */
  ~HazardousZones();

  /*!
    \brief
    The function to search radiation zones in the loaded simulation
  */
  void SearchRadiationZones();

  /*!
    \brief
    The function to search chemical zones in the loaded simulation
  */
  void SearchChemicalZones();

  /*!
    \brief
    Returns <b>true</b> if all robots are successfully loaded by Gazebo and
    analyzed by <b>gazebo_usar_core engine</b>.
    If the function fails, the return value is <b>false</b>.
  */
  bool SearchRobotModels();

  /*!
    \brief
    Runs <b>gazebo_usar_core simulation</b>.
  */
  void Run();

private:

  void createFakeNetworkInterfaces();

  void deleteFakeNetworkInterfaces();

  void sendNetworkRequestToRobot(std::string &robot_ip_address, std::string &data);

  /*!
    \brief
    Callback to handle input parameters from rqt_reconfigure.
    \param[in] config: Incoming data storing dynamically changed parameters.
    \param[in] level: Level of incoming data.
  */
  void dynamicReconfigureContaminationCallback(gazebo_usar_core::ContaminationConfig &config, uint32_t level);

  /*!
    \brief
    Loads radiation contamination map image via <b>map_server</b> as the <b>OccupancyGrid</b> message.
  */
  void loadRadiationMapFromMapServer();

  /*!
    \brief
    Loads chemical contamination map image via <b>map_server</b> as the <b>OccupancyGrid</b> message.
  */
  void loadChemicalMapFromMapServer();

  /*!
    \brief
    Wrapper to union <b>pcl::PointCloud<pcl::PointXYZRGB></b> clouds from the different generating modes to one cloud.
  */
  void generateHazardousZonesPointClouds(double dt);

  /*!
    \brief
    Generates the radiation <b>pcl::PointCloud<pcl::PointXYZRGB></b> cloud.
  */
  void generateRadiationZonesPointCloud();

  /*!
    \brief
    Generates the chemical <b>pcl::PointCloud<pcl::PointXYZRGB></b> cloud.
  */
  void generateChemicalZonesPointCloud();

  /*!
    \brief
    Generates the chemical <b>pcl::PointCloud<pcl::PointXYZRGB></b> cloud.
  */
  void simulateChemicalPropagation(double dt);
  

  /*!
    \brief
    Converts the <b>pcl::PointCloud<pcl::PointXYZRGB></b> cloud to the ROS sensor_msgs::PointCloud2 message.
    \param[in] pcl_cloud: <b>pcl::PointCloud<pcl::PointXYZRGB></b> cloud.
    \param[in,out] ros_cloud: </b>sensor_msgs::PointCloud2 message</b> message.
  */
  void convertPclToRosMsg(const pcl::PointCloud<pcl::PointXYZRGB> &pcl_cloud, sensor_msgs::PointCloud2 &ros_cloud);

  /*!
    \brief
    Publishes generated ROS sensor_msgs::PointCloud2 cloud to the <b>/hazardous_zones_point_cloud</b> topic.
    \param[in] wait_time: A timeout which lets specify the amount of time to wait for the next cloud publishing.
    <b> Default: 60 seconds </b>.
  */
  void pointCloudPublisherThread(double wait_time=10.0);

  /*!
    \brief
    Determines robots in the Gazebo and publishes their positions as the pose geometry_msgs::TransformStamped message.
    \param[in] frame_id: Frame name which is associated with the pose message.
    <b> Default: map </b>.
    \param[in] wait_time: A timeout which lets specify the amount of time to wait for the next pose publishing.
    <b> Default: 0.01 seconds </b>.
  */
  void showRobotPoseInRvizThread(std::string frame_id="map", double wait_time=0.01);

  /*!
    \brief
    Reads <b> gazebo_usar_core <plugin> <b> data.
  */
  void readPluginParameters();

  /*!
    \brief
    Generates the radiation <b>pcl::PointCloud<pcl::PointXYZRGB></b> cloud from the ROS OccupancyGrid message.
  */
  void generateRadiationPointCloudFromOccupancyGridNormalized();

  /*!
    \brief
    Generates the chemical <b>pcl::PointCloud<pcl::PointXYZRGB></b> cloud from the ROS OccupancyGrid message.
  */
  void generateChemicalPointCloudFromOccupancyGridNormalized();

  /*!
    \brief
    The function to check all available robots within chemical contaminations.
    \param[in] wait_time: A timeout which lets specify the amount of time to wait for the next robots checking.
    <b> Default: 2.0 seconds </b>.
  */
  void checkRobotsInChemicalZoneThread(double wait_time=2.0);

  /*!
    \brief
    The function to check all available robots within chemical contaminations.
    \param[in] wait_time: A timeout which lets specify the amount of time to wait for the next robots checking.
    <b> Default: 1.0 seconds </b>.
  */
  void CheckChemicalConcentrationThread(double wait_time=1.0);

  /*!
    \brief
    The <b>user-defined</b> callback function to perform math equation of chemical.
    \param[in,out] chemical_config: Config of the robot data for further chemical substance calculation.
    \param[in] chemical_zone: Description of the chemical zone.
    \return Calculated contamination dose. <b> Measured in Sv. </b>
  */
  double calculateParallelogramChemicalContamination(CHEMICAL_CONFIG &chemical_config, CHEMICAL_ZONE &chemical_zone);

  /*!
    \brief
    The function to check the position of an individual robot within chemical contaminations.
    \param[in,out] chemical_config: Config of the robot data for further chemical substance calculation.
    \param[in] calculation: Pointer to the function. User can write own callback function, which will calculate math equations of chemical representation.
    <b> The function should be matched the given signature. </b>
  */
  void checkRobotInChemicalZone(CHEMICAL_CONFIG &chemical_config, std::function<double(CHEMICAL_CONFIG &chemical_config, CHEMICAL_ZONE &chemical_zone)> calculation);

  /*!
    \brief
    The function to check all available robots within radiation contaminations.
    \param[in] wait_time: A timeout which lets specify the amount of time to wait for the next robots checking.
    <b> Default: 2.0 seconds </b>.
  */
  void checkRobotsInRadiationZoneThread(double wait_time=2.0);

  /*!
    \brief
    The <b>user-defined</b> callback function to perform math equation of radiation.
    \param[in,out] radiation_config: Config of the robot data for further radiation substance calculation.
    \param[in] rad_zone: Description of the radiation zone.
    \return Calculated contamination dose. <b> Measured in Gy. </b>
  */
  double calculateSphereRadiation(RADIATION_CONFIG &radiation_config, RADIATION_ZONE &rad_zone);

  /*!
    \brief
    The function to check the position of an individual robot within radiation contaminations.
    \param[in,out] radiation_config: Config of the robot data for further chemical substance calculation.
    \param[in] calculation: Pointer to the function. User can write own callback function, which will calculate math equations of radiation representation.
    <b> The function should be matched the given signature. </b>
  */
  void checkRobotInRadiationZone(RADIATION_CONFIG &radiation_config, std::function<double(RADIATION_CONFIG &radiation_config, RADIATION_ZONE &rad_zone)> calculation);

  /*!
    \brief
    Plays the sounds of the Geiger counter.\n
    The Geiger counter is an device for detecting and measuring ionizing radiation.\n
    This fake-detector plays 4 various audio streams depending on the number of accumulated contaminant.\n
    Implemented stream states:
      - <b>Low:</b> [min, min+max/4].
      - <b>Normal:</b> [min+max/4, min+max/2].
      - <b>High:</b> [max-max/2, max-max/4].
      - <b>Max High:</b> [max-max/4 <= accumulated_substance value].

     where:
      - <b>Min</b> - minimum value of radiation influence of the considered contamination.
      - <b>Max</b> - maximum value of radiation influence of the considered contamination.
      - <b>Accumulated_substance</b> - calculated value of radiation impact of the considered contamination.
    \param[in] radiation_data: Config of the robot data for further chemical substance calculation.
  */
  void playRadioactiveAudio(gazebo_usar_core::RadiationSubstance &radiation_data);

  /*! @brief Prefix of radiation zone in the XML file. */
  std::string radiation_zone_prefix = "rad_zone_";
  /*! @brief Prefix of chemical zone in the XML file. */
  std::string chemical_zone_prefix = "chemical_zone_";
  /*! @brief The name of the output ROS point cloud topic. */
  std::string pointcloud_topic_name = "/hazardous_zones_point_cloud";
  /*! @brief The ROS topic name where radiation substances will be published. */
  std::string radiation_substances_topic_name = "/radiation_substances";
  /*! @brief The ROS topic name where chemical substances will be published. */
  std::string chemical_substances_topic_name = "/chemical_substances";
  /*! @brief The ROS topic name where chemical concentration will be published. */
  std::string chemical_concentration_topic_name = "/chemical_concentration";
  /*! @brief A count of loaded radiation zones. */
  size_t radiation_zones_count = 0;
  /*! @brief A count of loaded chemical zones. */
  size_t chemical_zones_count = 0;
  /*! @brief A list contains loaded radiation contaminations. */
  std::vector<RADIATION_ZONE> radiation_contaminations;
  /*! @brief A list contains loaded chemical contaminations. */
  std::vector<CHEMICAL_ZONE> chemical_contaminations;
  /*! @brief Shared Pointer to the Gazebo virtual world. */
  gazebo::physics::WorldPtr gazebo_world;
  /*! @brief Shared Pointer to the Gazebo virtual world represented as the XML structure. */
  sdf::ElementPtr gazebo_world_sdf;
  /*! @brief Shared Pointer to the ROS Node. */
  ros::NodeHandlePtr plugin_ros_node;

  // ROS publishers
  /*! @brief ROS Publisher to send contaminations point cloud as the ROS <b>sensor_msgs::PointCloud2</b> message. */
  ros::Publisher output_pointcloud_pub;
  /*! @brief ROS Publisher to send chemical data as the <b>gazebo_usar_core::ChemicalSubstances</b> message. */
  ros::Publisher chemical_substances_pub;
  /*! @brief ROS Publisher to send chemical concentration as the <b>gazebo_usar_core::ChemicalConcentration</b> message. */
  ros::Publisher chemical_concentration_pub;
  /*! @brief ROS Publisher to send radiation data as the <b>gazebo_usar_core::RadiationSubstances</b> message. */
  ros::Publisher radiation_substances_pub;

  // std::threads
  /*! @brief Threads to check robots within hazardous zones. */
  ///@{
  std::thread thread_check_chemical_zones, thread_check_radiation_zones;
  ///@}
  std::thread thread_check_chemical_concentration;
  /*! @brief Thread to publish contaminations point cloud. */
  std::thread thread_pub_pointcloud;
  /*! @brief Threads to publish poses of robots. */
  std::thread thread_pub_poses;
  /*! @brief Threads to start Boost::asio network. */
  std::thread thread_network;

  boost::thread_group thread_pool;

  std::vector<std::string> ip_addresses;

  std::map<std::string, std::string> robots_to_ip_addresses;
  std::map<std::string, std::string> ip_addresses_to_names;

  bool easia_protocol_simulation_enabled = false;

  // point cloud storing
  /*! @brief PCL: representing hazardous zones as the <b>pcl::PointCloud<pcl::PointXYZRGB></b> cloud. */
  pcl::PointCloud<pcl::PointXYZRGB> pcl_point_cloud;
  /*! @brief ROS: representing hazardous zones as the ROS sensor_msgs::PointCloud2 cloud. */
  sensor_msgs::PointCloud2 ros_point_cloud;

  /*! @brief Origin coordinates (x, y, z) of the Gazebo virtual world.\n
   * <b>Default: [0,0,0]</b>
  */
  double gazebo_world_origin[3];
  /*! @brief A count of loaded robots. */
  int robots_count = 0;

  /*! @brief A list contains names of the robots. */
  std::vector<std::string> robots_names;
  /*! @brief A list contains Gazebo models (gazebo::physics::Model) of the robots. */
  std::vector<boost::shared_ptr<gazebo::physics::Model>> robots_models;

  /*! @brief The OccupancyGrid messages to store the contamination image maps. */
  ///@{
  nav_msgs::OccupancyGrid radiation_occ_grid_data, chemical_occ_grid_data;
  ///@}

  // Contamination Maps descriptions
  /*! @brief Config of the radiation contamination map. */
  RADIATION_MAP_CONFIG radiation_map_description;
  /*! @brief Config of the chemical contamination map. */
  CHEMICAL_MAP_CONFIG chemical_map_description;

  // Determine the image to occupancy_grid approach using
  /*! @brief Defines the initial OccupancyGrid map states. */
  ///@{
  bool radiation_map_exist = false, chemical_map_exist = false;
  ///@}

  //std::unique_ptr<GazeboAudio> gazMusic;
  /*! @brief Fake Geiger device. */
  //std::shared_ptr<GazeboAudioChannels> gazebo_geiger_device;
  /*! @brief Determines whether <geiger> tag is found in the <plugin> data. */
  bool gazebo_music_enabled = false;

  /*! @brief Frame name of the output ROS point cloud. */
  std::string point_cloud_map_frame_id = "map";

  /*! @brief A number of available threads that can be started simultaneously.\n
   * The optimal thread count will be found during plugin initialization.\n
   * <b>Default: 4.</b>
  */
  size_t num_available_threads = 4;

  /*! @brief A timeout which lets specify the amount of time to wait for the next ROS point cloud publishing.\n
   * <b> Default: 20.0 seconds </b>
  */
  double point_cloud_publish_rate = 20.0;
  /*! @brief A timeout which lets specify the amount of time to wait for the next robots checking within radiation areas.\n
   * <b> Default: 2.0 seconds </b>
  */
  double radiation_check_rate = 2.0;
  /*! @brief A timeout which lets specify the amount of time to wait for the next robots checking within chemical areas.\n
   * <b> Default: 2.0 seconds </b>
  */
  double chemical_check_rate = 2.0;

  /*! @brief Defines the dynamic_reconfigure server passing it our config type. */
  typedef dynamic_reconfigure::Server<gazebo_usar_core::ContaminationConfig> ContaminationConfigServer;
  /*! @brief Unique Pointer to store the defined dynamic_reconfigure server. */
  std::unique_ptr<ContaminationConfigServer> dynamic_reconfigure_contamination_server;

  Fluid* FluidGas;

};

#endif // HAZARDOUS_ZONES_H
