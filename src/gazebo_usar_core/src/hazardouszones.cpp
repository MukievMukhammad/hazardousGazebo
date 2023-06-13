#include "hazardouszones.h"
#include "chemical_gas_propogation.h"

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>

#include <tf/transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>

#include <nav_msgs/GetMap.h>

#include <algorithm>
#include <chrono>
#include <functional>
#include <sstream>

#include <gazebo/common/Console.hh>
#include <boost/bind.hpp>

#include <cassert>

#include <QString>
#include <QProcess>
#include <QFile>
#include <QTextStream>

#include <octomap/octomap.h>
#include <math.h>

HazardousZones::HazardousZones(ros::NodeHandlePtr rosNodePtr,
                               gazebo::physics::WorldPtr worldPtr,
                               sdf::ElementPtr sdfPtr)
{
  this->gazebo_world = worldPtr;
  this->gazebo_world_sdf = sdfPtr;
  this->plugin_ros_node = rosNodePtr;
  this->output_pointcloud_pub =
      this->plugin_ros_node->advertise<sensor_msgs::PointCloud2>(pointcloud_topic_name, 1);
  this->radiation_substances_pub =
      this->plugin_ros_node->advertise<gazebo_usar_core::RadiationSubstances>(
          radiation_substances_topic_name, 10);
  this->chemical_substances_pub =
      this->plugin_ros_node->advertise<gazebo_usar_core::ChemicalSubstances>(
          chemical_substances_topic_name, 10);
  this->chemical_concentration_pub =
      this->plugin_ros_node->advertise<gazebo_usar_core::ChemicalConcentration>(
          chemical_concentration_topic_name, 10);

  // Boundary of world map from (0,0,0) to (18.57 ,19 , 2.6)
  // TODO: Resolve case when map could be far from (0,0,0) take into account minPoint
  // Load the .bt file
  octomap::OcTree tree("/home/mukiev/catkin_ws/src/world2oct/hazardous_zones.bt");
  int maxPoint = 0;
  octomap::OcTree::iterator max_leaf;
  for (octomap::OcTree::iterator it = tree.begin(); it != tree.end(); ++it)
  {
    auto len = pow(it.getX(), 2) + pow(it.getY(), 2) + pow(it.getZ(), 2);
    if (len > maxPoint)
    {
      maxPoint = len;
      max_leaf = it;
    }
  }

  ROS_INFO("Max leaf: (%f,%f,%f)", max_leaf.getX(), max_leaf.getY(), max_leaf.getZ());

  auto X_length = max_leaf.getX();
  auto Y_length = max_leaf.getY();
  auto Z_length = max_leaf.getZ();
  auto h = Z_length / 25;
  int numX = abs(floor(Y_length / h));
  int numY = abs(floor(X_length / h));
  int numZ = 12; //abs(floor(Z_length / h));
  ROS_INFO("h: %f", h);
  ROS_INFO("Mesh size: (%i,%i,%i)", numX, numY, numZ);
  this->FluidGas = new Fluid(3214, numX, numY, numZ, h);

  // OcTree
  // std::fill(this->FluidGas->scaler_value, this->FluidGas->scaler_value+this->FluidGas->numCells, 1);
  // Iterate over all nodes in the tree and check if they are occupied
  for (octomap::OcTree::iterator it = tree.begin(); it != tree.end(); ++it)
  {
    int z = floor(it.getZ() / h);
    if(z < 13) continue;
    int y = this->FluidGas->numY - floor(it.getX() / h);
    int x = floor(it.getY() / h);

    auto xy_slice = this->FluidGas->numX * this->FluidGas->numY;
    int idx = (z-13) * xy_slice + x * this->FluidGas->numY + y;
    this->FluidGas->scaler_value[idx] = 0; // tree.isNodeOccupied(*it) ? 0 : 1;
  }

  num_available_threads = std::thread::hardware_concurrency();

  readPluginParameters();
}

void HazardousZones::readPluginParameters()
{
  if (gazebo_world_sdf->HasElement("robots"))
  {
    // get data from the 'robots' tag of the loaded SDF file
    std::string strRobots = gazebo_world_sdf->Get<std::string>("robots");
    std::istringstream robots;
    robots.str(strRobots);
    std::string robot;

    // parse names (';' - separator)
    while (std::getline(robots, robot, ';'))
    {
      if (!robot.empty())
        robots_names.push_back(robot);
    }

    // count a number of available robots
    robots_count = robots_names.size();
    ROS_WARN_STREAM("Loaded robot count: " << robots_count);

    // print parsed names to console
    for (const auto &item : robots_names)
      ROS_WARN_STREAM("Loaded robot: " << item);

    // check if the sound option is enabled
    if (gazebo_world_sdf->HasElement("sound"))
      gazebo_music_enabled = gazebo_world_sdf->Get<bool>("sound");
  }

  // get fake Ethernet IP addresses of robots
  if (gazebo_world_sdf->HasElement("ip_addresses"))
  {
    auto ip_addresses_el = gazebo_world_sdf->GetElement("ip_addresses");
    easia_protocol_simulation_enabled = true;
    for (int idx = 1; idx <= robots_count; idx++)
    {
      auto ip_el_name = "ip" + std::to_string(idx);
      if (ip_addresses_el->HasElement(ip_el_name))
      {
        auto ip_el = ip_addresses_el->Get<std::string>(ip_el_name);
        ip_addresses.push_back(ip_el);
        robots_to_ip_addresses[robots_names.at(idx - 1)] = ip_el;
        ip_addresses_to_names[ip_el] = robots_names.at(idx - 1);
        ROS_WARN("Received %s : %s\n", ip_el_name.c_str(), ip_el.c_str());
      }
    }
    std::stringstream ip_addresses_string_stream;
    std::for_each(std::begin(ip_addresses), std::end(ip_addresses),
                  [&ip_addresses_string_stream, sep = ';'](std::string &ip_address)
                  {
                    ip_addresses_string_stream << ip_address << sep;
                  });
    std::string ip_addresses_str = ip_addresses_string_stream.str();
    // remove the last character (it's ';')
    ip_addresses_str.pop_back();
    ros::param::set("/gazebo_usar_core/ip_addresses", ip_addresses_str);
    std::string robot_names_str = boost::algorithm::join(robots_names, ";");
    ros::param::set("/gazebo_usar_core/robot_names", robot_names_str);
  }

  ROS_WARN_STREAM("GAZEBO MUSIC SUPPORT: " << std::boolalpha << gazebo_music_enabled);
  ROS_WARN_STREAM("EASIA USAR PROTOCOL SUPPORT: " << std::boolalpha << easia_protocol_simulation_enabled);

  gazebo_world_origin[0] = gazebo_world_origin[1] = gazebo_world_origin[2] = 0;
}

void HazardousZones::SearchRadiationZones()
{
  for (const auto &model : gazebo_world->Models())
  {
    std::string modelName = model->GetName();
    if (modelName.find(radiation_zone_prefix) != std::string::npos)
    {
      RADIATION_ZONE rz;
      rz.Name = modelName;
      rz.Origin = model->WorldPose().Pos();
      rz.Origin.Set(rz.Origin.X() - gazebo_world_origin[0],
                    rz.Origin.Y() - gazebo_world_origin[1],
                    rz.Origin.Z() - gazebo_world_origin[2]);
      // store a radius as the first value of frame pose tag in the world (XML) file structure
      auto contamination_data = model->GetSDF()->GetElement("frame")->Get<ignition::math::Pose3d>("pose");
      rz.Radius = contamination_data.Pos().X();
      radiation_contaminations.push_back(rz);
      radiation_zones_count++;
    }
  }
  ROS_WARN_STREAM("Radiation zones count = " << radiation_zones_count);
}

void HazardousZones::SearchChemicalZones()
{
  for (const auto &model : gazebo_world->Models())
  {
    std::string modelName = model->GetName();
    if (modelName.find(chemical_zone_prefix) != std::string::npos)
    {
      CHEMICAL_ZONE cz;
      cz.Name = modelName;
      cz.Position = model->WorldPose().Pos();
      cz.Position.Set(cz.Position.X() - gazebo_world_origin[0],
                      cz.Position.Y() - gazebo_world_origin[1],
                      cz.Position.Z() - gazebo_world_origin[2]);
      // store lenght, width and height as the first three parameters
      auto contamination_data = model->GetSDF()->GetElement("frame")->Get<ignition::math::Pose3d>("pose");
      cz.Lenght = contamination_data.Pos().X();
      cz.Width = contamination_data.Pos().Y();
      cz.Height = contamination_data.Pos().Z();
      cz.MedianRadius = (cz.Lenght / 2 + cz.Width / 2 + cz.Height / 2) / 2;
      this->chemical_contaminations.push_back(cz);
      this->chemical_zones_count++;
    }
  }
  ROS_WARN_STREAM("Chemical zones count = " << chemical_zones_count);
}

bool HazardousZones::SearchRobotModels()
{
  bool allRobotsFound = false;
  for (const auto &model : gazebo_world->Models())
  {
    std::string modelName = model->GetName();
    for (const auto &robot : robots_names)
    {
      if (modelName.find(robot) != std::string::npos)
      {
        // if the model has not yet been found, add it
        if (std::find(robots_models.begin(), robots_models.end(), model) == robots_models.end())
          robots_models.push_back(model);
      }
    }
    if (robots_models.size() == robots_count)
      return !allRobotsFound;
  }
  return allRobotsFound;
}

HazardousZones::~HazardousZones()
{
  ROS_WARN("Clearing HazardousZones");
  thread_pool.join_all();
  if (thread_pub_pointcloud.joinable())
    thread_pub_pointcloud.join();
  if (thread_check_chemical_zones.joinable())
    thread_check_chemical_zones.join();
  if (thread_check_radiation_zones.joinable())
    thread_check_radiation_zones.join();
  if (thread_pub_poses.joinable())
    thread_pub_poses.join();
}

void HazardousZones::Run()
{
  ros::Duration repeat(1);
  // load radiation image via map_server as occupancy_grid message
  loadRadiationMapFromMapServer();
  // load chemical image via map_server as occupancy_grid message
  loadChemicalMapFromMapServer();
  // when contaminations are loaded and parsed generate point cloud message from occupancy_grid
  // generateHazardousZonesPointClouds();
  this->thread_pub_pointcloud = std::thread(std::bind(&HazardousZones::pointCloudPublisherThread, this, 0.05));
  // this->thread_check_chemical_zones =
  //     std::thread(std::bind(&HazardousZones::checkRobotsInChemicalZoneThread, this, 2.0));
  this->thread_check_chemical_concentration =
      std::thread(std::bind(&HazardousZones::CheckChemicalConcentrationThread, this, 1.0));
  // this->thread_check_radiation_zones =
  //     std::thread(std::bind(&HazardousZones::checkRobotsInRadiationZoneThread, this, 2.0));
  // // this->poseCheckThread = std::thread(std::bind(&RadiationZones::checkRobotPoseThread, this));
  // this->thread_pub_poses =
  //     std::thread(std::bind(&HazardousZones::showRobotPoseInRvizThread, this, "map", 0.01));

  ROS_WARN("Thread reconfigure is ready!\n");

  dynamic_reconfigure_contamination_server = std::make_unique<ContaminationConfigServer>(*plugin_ros_node.get());
  dynamic_reconfigure_contamination_server->setCallback(boost::bind(&HazardousZones::dynamicReconfigureContaminationCallback, this, _1, _2));
  ROS_WARN("Dynamic reconfigure is ready!\n");
}

void HazardousZones::CheckChemicalConcentrationThread(double wait_time)
{
  boost::thread_group threads;
  while (!ros::isShuttingDown())
  {
    for (size_t robot_idx = 0; robot_idx < robots_count; ++robot_idx)
    {
      threads.create_thread([this, robot_idx]()
                            {
        gazebo_usar_core::ChemicalConcentration concentration;

        auto cur_robot_model = robots_models.at(robot_idx);
        auto pose = cur_robot_model->WorldPose().Pos();
        auto robot_pose = &pose;
        concentration.robot_name = cur_robot_model->GetName();
        double z_delta = 13 * this->FluidGas->h;
        concentration.concentration_value = this->FluidGas->getConcentration(robot_pose->X(), robot_pose->Y(), robot_pose->Z()-z_delta);
        ROS_INFO("%f", concentration.concentration_value);
        chemical_concentration_pub.publish(concentration);
        ros::spinOnce(); });
    }
    threads.join_all();

    ros::Duration(wait_time).sleep();
  }
}

void HazardousZones::dynamicReconfigureContaminationCallback(gazebo_usar_core::ContaminationConfig &config, uint32_t level)
{
  ROS_INFO("Reconfigure Request: %f %f %f\n",
           config.point_cloud_publish_rate, config.radiation_check_rate,
           config.chemical_check_rate);
  point_cloud_publish_rate = config.point_cloud_publish_rate;
  radiation_check_rate = config.radiation_check_rate;
  chemical_check_rate = config.chemical_check_rate;
}

void HazardousZones::loadRadiationMapFromMapServer()
{
  nav_msgs::GetMap srv_rad_map;
  ros::ServiceClient radMapClient =
      plugin_ros_node->serviceClient<nav_msgs::GetMap>("/rad_static_map");
  if (radMapClient.exists())
  {
    radMapClient.waitForExistence();
    if (radMapClient.call(srv_rad_map))
    {
      ROS_WARN("RadioactiveMap service called successfully");
      radiation_occ_grid_data = srv_rad_map.response.map;
      generateRadiationPointCloudFromOccupancyGridNormalized();
      radiation_map_exist = true;
      ROS_WARN_STREAM("Radiation occupancy grid map found!");
      uint8_t max_temp = *radiation_occ_grid_data.data.begin();
      uint8_t min_temp = *radiation_occ_grid_data.data.begin();
      std::for_each(
          radiation_occ_grid_data.data.begin(), radiation_occ_grid_data.data.end(), [&](int8_t &p)
          {
          uint8_t l = (uint8_t)p;
          if(min_temp > l)
            min_temp = l;
          if(max_temp < l)
            max_temp = l; });
      radiation_map_description.Max = max_temp;
      radiation_map_description.Min = min_temp;
      radiation_map_description.Resolution = radiation_occ_grid_data.info.resolution;
      radiation_map_description.Width = radiation_occ_grid_data.info.width;
      radiation_map_description.Height = radiation_occ_grid_data.info.height;
    }
  }
  else
    ROS_WARN("RadioactiveMap service called failed");
}

void HazardousZones::loadChemicalMapFromMapServer()
{
  nav_msgs::GetMap srv_chem_map;
  ros::ServiceClient chem_map_client =
      plugin_ros_node->serviceClient<nav_msgs::GetMap>("/chem_static_map");
  if (chem_map_client.exists())
  {
    chem_map_client.waitForExistence();
    if (chem_map_client.call(srv_chem_map))
    {
      ROS_WARN("ChemicalMap service called successfully");
      chemical_occ_grid_data = srv_chem_map.response.map;
      generateRadiationPointCloudFromOccupancyGridNormalized();
      chemical_map_exist = true;
      ROS_WARN_STREAM("Chemical occupancy grid map found!");
      uint8_t max_temp = *chemical_occ_grid_data.data.begin();
      uint8_t min_temp = *chemical_occ_grid_data.data.begin();
      std::for_each(
          chemical_occ_grid_data.data.begin(), chemical_occ_grid_data.data.end(), [&](int8_t &p)
          {
          uint8_t l = (uint8_t)p;
          if(min_temp > l)
            min_temp = l;
          if(max_temp < l)
            max_temp = l; });
      chemical_map_description.Max = max_temp;
      chemical_map_description.Min = min_temp;
      chemical_map_description.Resolution = chemical_occ_grid_data.info.resolution;
      chemical_map_description.Width = chemical_occ_grid_data.info.width;
      chemical_map_description.Height = chemical_occ_grid_data.info.height;
    }
  }
  else
    ROS_WARN("ChemicalMap service called failed");
}

void HazardousZones::generateHazardousZonesPointClouds(double dt)
{
  if (radiation_map_exist)
    generateRadiationPointCloudFromOccupancyGridNormalized();
  if (chemical_map_exist)
    generateChemicalPointCloudFromOccupancyGridNormalized();
  // generateChemicalZonesPointCloud();
  simulateChemicalPropagation(dt);
  // generateRadiationZonesPointCloud();
  convertPclToRosMsg(pcl_point_cloud, ros_point_cloud);
  ros_point_cloud.header.frame_id = point_cloud_map_frame_id;
}

void HazardousZones::pointCloudPublisherThread(double wait_time)
{
  ROS_WARN("Published...\n");
  point_cloud_publish_rate = wait_time;
  auto diff = 0.01;
  while (!ros::isShuttingDown())
  {
    ros_point_cloud.header.stamp = ros::Time::now();
    auto start = std::chrono::steady_clock::now();
    generateHazardousZonesPointClouds(diff);
    auto end = std::chrono::steady_clock::now();
    diff = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count()*0.001;
    output_pointcloud_pub.publish(ros_point_cloud);
    ros::spinOnce();
    // ros::Duration(point_cloud_publish_rate).sleep();
  }
}

double truncateNumber(double number, unsigned int digits)
{
  double f = pow(10, digits);
  return ((int)(number * f)) / f;
}

bool inBounds(double value, double low, double high)
{
  return low <= value && value < high;
}

void HazardousZones::checkRobotsInChemicalZoneThread(double wait_time)
{
  // check every 2 seconds
  chemical_check_rate = wait_time;
  gazebo_usar_core::ChemicalSubstances chemical_data;
  std::chrono::steady_clock::time_point begin_time, end_time;
  size_t robots_count = robots_models.size();
  boost::thread_group threads;
  while (!ros::isShuttingDown())
  {
    chemical_data.chemicalSubstances.clear();
    begin_time = std::chrono::steady_clock::now();

    for (size_t robot_idx = 0; robot_idx < robots_count; ++robot_idx)
    {
      threads.create_thread([this, robot_idx, &chemical_data]()
                            {
        gazebo_usar_core::ChemicalSubstance chemical_substance;
        auto cur_robot_model = robots_models.at(robot_idx);
        auto pose = cur_robot_model->WorldPose().Pos();
        chemical_substance.robot_name = cur_robot_model->GetName();


        // DEMO: send handshake msg if network protocol is enabled
        /*if(easia_protocol_simulation_enabled)
          sendNetworkRequestToRobot(robots_to_ip_addresses[chemical_substance.robot_name], chemical_substance.robot_name);*/
        CHEMICAL_CONFIG chemical_config;
        chemical_config.robot_pose = &pose;
        chemical_config.data = &chemical_substance;
        checkRobotInChemicalZone(chemical_config, std::bind(&HazardousZones::calculateParallelogramChemicalContamination, this,
                                                        std::placeholders::_1, std::placeholders::_2));
        // Задает значения для гуй
        // chemical_substance.chemical_accumulated = 0.1;
        // chemical_substance.max = 1.0;
        // chemical_substance.min = 0;
        chemical_data.chemicalSubstances.push_back(chemical_substance);
        ros::spinOnce(); });
    }
    threads.join_all();

    end_time = std::chrono::steady_clock::now();
    gzerr << "Worker thread id: " << std::this_thread::get_id() << '\n';
    gzerr << "Chemical areas checking done!\n";
    gzerr << "Elapsed time: " << std::chrono::duration_cast<std::chrono::milliseconds>(end_time - begin_time).count() << "[ms]" << std::endl;
    chemical_substances_pub.publish(chemical_data);
    ros::Duration(chemical_check_rate).sleep();
  }
}

double HazardousZones::calculateParallelogramChemicalContamination(CHEMICAL_CONFIG &chemical_config, CHEMICAL_ZONE &chemical_zone)
{
  double chemicalValue = 0.0;
  ignition::math::Vector3d *robot_pose = chemical_config.robot_pose;
  gazebo_usar_core::ChemicalSubstance *data = chemical_config.data;

  double currentRobotX = robot_pose->X() - gazebo_world_origin[0];
  if (currentRobotX >= chemical_zone.X_MIN && currentRobotX <= chemical_zone.X_MAX)
  {
    double currentRobotY = robot_pose->Y() - gazebo_world_origin[1];
    if (currentRobotY >= chemical_zone.Y_MIN && currentRobotY <= chemical_zone.Y_MAX)
    {
      double currentRobotZ = robot_pose->Z() - gazebo_world_origin[2];
      if (currentRobotZ >= chemical_zone.Z_MIN &&
          currentRobotZ <= chemical_zone.Z_MAX)
      {
        chemicalValue += (std::abs(currentRobotX - chemical_zone.X_MAX) +
                          std::abs(currentRobotY - chemical_zone.Y_MAX) +
                          std::abs(currentRobotZ - chemical_zone.Z_MAX)) *
                         0.1;
      }
    }
  }
  return chemicalValue;
}

void HazardousZones::checkRobotInChemicalZone(CHEMICAL_CONFIG &chemical_config,
                                              std::function<double(CHEMICAL_CONFIG &chemical_config, CHEMICAL_ZONE &chemical_zone)> calculation)
{
  // TODO
  double chemicalValue = 0.0;
  float dose_on_tone = 0.005;
  double chemical_max = 0.001, chemical_min = 0.001;
  double epicenter_epsilon = 0.1;
  ignition::math::Vector3d *robot_pose = chemical_config.robot_pose;
  gazebo_usar_core::ChemicalSubstance *data = chemical_config.data;

  if (chemical_map_exist)
  {
    // first, check a gradient radiation map
    double x = round(robot_pose->X() / chemical_occ_grid_data.info.resolution);
    double y = round(robot_pose->Y() / chemical_occ_grid_data.info.resolution);
    size_t index = x + chemical_occ_grid_data.info.width * y;
    unsigned char cell_data;
    if (index < chemical_occ_grid_data.data.size())
    {
      cell_data = chemical_occ_grid_data.data[index];
      chemicalValue += cell_data;
      chemicalValue *= dose_on_tone;
      chemical_max += chemical_map_description.Max;
      chemical_max *= dose_on_tone;
      chemical_min += chemical_map_description.Min;
      chemical_min *= dose_on_tone;
    }
  }

  for (auto zone = chemical_contaminations.begin(); zone != chemical_contaminations.end(); zone++)
  {
    chemicalValue += calculation(chemical_config, *zone);
    double chemical_zone_min = 0.00001;
    if (chemical_min > chemical_zone_min)
      chemical_min = chemical_zone_min;
    double chemical_zone_max = (zone->MedianRadius - epicenter_epsilon) * 0.001;
    if (chemicalValue > chemical_zone_max)
      chemical_zone_max = chemicalValue;
    if (chemical_max < chemical_zone_max)
      chemical_max = chemical_zone_max;
  }
  data->chemical_accumulated = truncateNumber(chemicalValue, 5);
  data->max = truncateNumber(chemical_max, 5);
  data->min = truncateNumber(chemical_min, 5);
  // ROS_WARN("[Accumulated chem value]: %.3f\n", chemicalValue);
  gzerr << "Chemical Area was checked by thread: " << std::this_thread::get_id() << '\n';
}

void HazardousZones::checkRobotsInRadiationZoneThread(double wait_time)
{
  radiation_check_rate = wait_time;
  gazebo_usar_core::RadiationSubstances radiation_data;
  // ros::Duration(10).sleep();
  std::chrono::steady_clock::time_point begin_time, end_time;
  size_t robots_count = robots_models.size();
  std::vector<std::thread> thread_pool(robots_count);
  boost::thread_group threads;
  while (!ros::isShuttingDown())
  {
    radiation_data.radiationSubstances.clear();
    begin_time = std::chrono::steady_clock::now();

    for (size_t robot_idx = 0; robot_idx < robots_count; ++robot_idx)
    {
      threads.create_thread([this, robot_idx, &radiation_data]()
                            {
        gazebo_usar_core::RadiationSubstance radiation_substance;        
        auto cur_robot_model = robots_models.at(robot_idx);
        auto pose = cur_robot_model->WorldPose().Pos();
        radiation_substance.robot_name = cur_robot_model->GetName();
        RADIATION_CONFIG rad_config;
        rad_config.robot_pose = &pose;
        rad_config.data = &radiation_substance;
        checkRobotInRadiationZone(rad_config, std::bind(&HazardousZones::calculateSphereRadiation, this,
                                                        std::placeholders::_1, std::placeholders::_2));
        radiation_data.radiationSubstances.push_back(radiation_substance);
        ros::spinOnce(); });
    }
    threads.join_all();

    end_time = std::chrono::steady_clock::now();
    gzerr << "Worker thread id: " << boost::this_thread::get_id() << '\n';
    gzerr << "Radiation areas checking done!\n";
    gzerr << "Elapsed time: " << std::chrono::duration_cast<std::chrono::milliseconds>(end_time - begin_time).count() << "[ms]" << std::endl;
    radiation_substances_pub.publish(radiation_data);
    ros::Duration(radiation_check_rate).sleep();
  }
}

void HazardousZones::showRobotPoseInRvizThread(std::string frame_id, double wait_time)
{
  tf::TransformBroadcaster br;
  geometry_msgs::TransformStamped robot_pose_tf;
  robot_pose_tf.header.frame_id = frame_id;

  ros::Duration loop(wait_time);
  while (!ros::isShuttingDown())
  {
    for (auto &robot : robots_models)
    {
      robot_pose_tf.child_frame_id = robot->GetName() + "/pose_frame";
      auto pose = robot->WorldPose().Pos();
      auto rot = robot->WorldPose().Rot();

      robot_pose_tf.header.stamp = ros::Time::now();
      robot_pose_tf.transform.translation.x = pose.X() - gazebo_world_origin[0];
      robot_pose_tf.transform.translation.y = pose.Y() - gazebo_world_origin[1];
      robot_pose_tf.transform.translation.z = pose.Z() - gazebo_world_origin[2];

      robot_pose_tf.transform.rotation.x = rot.X();
      robot_pose_tf.transform.rotation.y = rot.Y();
      robot_pose_tf.transform.rotation.z = rot.Z();
      robot_pose_tf.transform.rotation.w = rot.W();

      br.sendTransform(robot_pose_tf);
    }
    loop.sleep();
  }
}

double HazardousZones::calculateSphereRadiation(RADIATION_CONFIG &radiation_config, RADIATION_ZONE &rad_zone)
{
  double radiationValue = 0.0;
  ignition::math::Vector3d *robot_pose = radiation_config.robot_pose;
  gazebo_usar_core::RadiationSubstance *data = radiation_config.data;
  auto radiation_zone_radius = rad_zone.Radius;

  double d1 =
      std::pow((robot_pose->X() - gazebo_world_origin[0] - rad_zone.Origin.X()), 2);
  double d2 =
      std::pow((robot_pose->Y() - gazebo_world_origin[1] - rad_zone.Origin.Y()), 2);
  double d3 =
      std::pow((robot_pose->Z() - gazebo_world_origin[2] - rad_zone.Origin.Z()), 2);
  double distanceToEpicenter = std::sqrt(d1 + d2 + d3);
  if (distanceToEpicenter <= radiation_zone_radius)
  {
    double P = radiation_zone_radius - distanceToEpicenter;
    double Z = P * 0.001;
    radiationValue += Z;
  }
  return radiationValue;
}

void HazardousZones::checkRobotInRadiationZone(RADIATION_CONFIG &radiation_config, std::function<double(RADIATION_CONFIG &radiation_config, RADIATION_ZONE &rad_zone)> calculation)
{
  double radiationValue = 0.0;
  double dose_on_tone = 0.001;
  double rad_max = 0.001, rad_min = 0.001;
  double epicenter_epsilon = 0.1;
  ignition::math::Vector3d *robot_pose = radiation_config.robot_pose;
  gazebo_usar_core::RadiationSubstance *data = radiation_config.data;

  try
  {

    if (radiation_map_exist)
    {
      // first, check a gradient radiation map
      double x = round(robot_pose->X() / radiation_occ_grid_data.info.resolution);
      double y = round(robot_pose->Y() / radiation_occ_grid_data.info.resolution);
      size_t index = x + radiation_occ_grid_data.info.width * y;
      unsigned char cell_data;
      if (index < radiation_occ_grid_data.data.size())
      {
        cell_data = radiation_occ_grid_data.data[index];
        if (cell_data != 0)
        {
          radiationValue += cell_data;
          radiationValue *= dose_on_tone;
          rad_max += radiation_map_description.Max;
          rad_max *= dose_on_tone;
          rad_min += radiation_map_description.Min;
          rad_min *= dose_on_tone;
        }
      }
    }

    // second, check Radiation zones
    for (auto zone = radiation_contaminations.begin(); zone != radiation_contaminations.end(); zone++)
    {
      radiationValue += calculation(radiation_config, *zone);
      double rad_zone_min = 0.00001;
      if (rad_min > rad_zone_min)
        rad_min = rad_zone_min;
      double rad_zone_max = (zone->Radius - epicenter_epsilon) * 0.001;
      if (radiationValue > rad_zone_max)
        rad_zone_max = radiationValue;
      if (rad_max < rad_zone_max)
        rad_max = rad_zone_max;
    }
    data->radiation_accumulated = truncateNumber(radiationValue, 5);
    data->max = truncateNumber(rad_max, 5);
    data->min = truncateNumber(rad_min, 5);
    if (gazebo_music_enabled)
      playRadioactiveAudio(*data);
    gzerr << "Radiation Area was checked by thread: " << std::this_thread::get_id() << '\n';
  }

  catch (std::exception &e)
  {
    std::cerr << "Exception caught : " << e.what() << std::endl;
  }
}

void HazardousZones::simulateChemicalPropagation(double dt)
{
  auto xy_slice = this->FluidGas->numX * this->FluidGas->numY;
  // refresh chemical contamination source
  auto fluid = this->FluidGas;
  double contamination_source_width = fluid->numX * 0.2;
  double contamination_source_length = fluid->numY * 0.2;
  double contamination_source_height = fluid->numZ * 0.2;

  int z_min = 1;//27; // floor(fluid->numZ * 0.75 - contamination_source_height*0.5); // - 1 // = 1
  int z_max = 2;//28; // floor(fluid->numZ * 0.75 + contamination_source_height*0.5); // + 1 // = 2

  int y_min = floor(fluid->numY * 0.5 - contamination_source_length * 0.5); // - 1
  int y_max = floor(fluid->numY * 0.5 + contamination_source_length * 0.5); // + 1

  int x_min = floor(fluid->numX * 0.5 - contamination_source_width * 0.5); // - 1
  int x_max = floor(fluid->numX * 0.5 + contamination_source_width * 0.5); // + 1

  // simulate dt
  // ROS_INFO("%f ms", dt);
  fluid->simulate(dt, -9.81, 40); // попробовать другой numIters

  for (int z = z_min; z < z_max; z++)
  {
    for (int y = y_min; y < y_max; y++)
    {
      for (int x = x_min; x < x_max; x++)
      {
        int current_cell_idx = z * fluid->numX * fluid->numY + x * fluid->numY + y;
        if (fluid->scaler_value[current_cell_idx] == 0)
          continue;
        fluid->density_mesh[current_cell_idx] = 0;
        fluid->z_velocity[current_cell_idx] = 1;
      }
    }
  }

  pcl::PointCloud<pcl::PointXYZRGB> new_pcl_state;
  // density to point
  for (int z = 0; z < fluid->numZ; z++)
  {
    for (int y = 0; y < fluid->numY; y++)
    {
      for (int x = 0; x < fluid->numX; x++)
      {
        int current_cell_idx = z * fluid->numX * fluid->numY + x * fluid->numY + y;
        double density = 1 - fluid->density_mesh[current_cell_idx];

        pcl::PointXYZRGB pt;
        if (fluid->scaler_value[current_cell_idx] == 1 && density < 0.0005)
          continue;

        if (fluid->scaler_value[current_cell_idx] == 0)
        {
          if (z == z_max)
          { // z >= 28 && z < 45){ // показыает в РВИЗ стены
            pt = pcl::PointXYZRGB(50, 205, 50);
          }
          else
          {
            continue;
          }
        }
        else
          pt = pcl::PointXYZRGB(255 * density, 153 * density, 1 * density);

        // не публиковать точки которые внутри облака, это повысит фпс
        bool hasEmptyNeighbour = false;
        if (x > 1 && y > 1 && z > 0 && x < fluid->numX - 2 && y < fluid->numY - 2 && z < fluid->numZ - 2)
        {
          for (int min_x = x - 1; min_x <= x + 1; min_x++)
          {
            for (int min_y = y - 1; min_y <= y + 1; min_y++)
            {
              for (int min_z = z - 1; min_z <= z + 1; min_z++)
              {
                int idx = min_z * fluid->numX * fluid->numY + min_x * fluid->numY + min_y;
                if (fluid->scaler_value[idx] == 0 || density < 0.2)
                {
                  hasEmptyNeighbour = true;
                  break;
                }
              }
              if (hasEmptyNeighbour)
                break;
            }
            if (hasEmptyNeighbour)
              break;
          }
        }

        if (!hasEmptyNeighbour)
          continue;

        pt.x = x * fluid->h + fluid->h * 0.5; // + 0.93;
        pt.y = y * fluid->h + fluid->h * 0.5; // + 0.8
        pt.z = (z + 13) * fluid->h + fluid->h * 0.5; // + 3

        new_pcl_state.points.push_back(pt);
        // this->pcl_point_cloud.points.push_back(pt);
      }
    }
  }
  this->pcl_point_cloud = new_pcl_state;
}

void HazardousZones::generateChemicalZonesPointCloud()
{
  std::random_device rd;
  std::mt19937 gen(rd());
  size_t num_points_ = 1000000;

  double min = -6, max = 14;
  double genNumber;

  for (size_t i = 0; i < this->chemical_zones_count; i++)
  {
    double R_X = chemical_contaminations[i].Lenght / 2;
    double R_Y = chemical_contaminations[i].Width / 2;
    double R_Z = chemical_contaminations[i].Height / 2;
    double X_MIN = chemical_contaminations[i].Position.X() - R_X;
    double X_MAX = chemical_contaminations[i].Position.X() + R_X;
    double Y_MIN = chemical_contaminations[i].Position.Y() - R_Y;
    double Y_MAX = chemical_contaminations[i].Position.Y() + R_Y;
    double Z_MIN = chemical_contaminations[i].Position.Z() - R_Z;
    double Z_MAX = chemical_contaminations[i].Position.Z() + R_Z;
    chemical_contaminations[i].X_MIN = X_MIN;
    chemical_contaminations[i].X_MAX = X_MAX;
    chemical_contaminations[i].Y_MIN = Y_MIN;
    chemical_contaminations[i].Y_MAX = Y_MAX;
    chemical_contaminations[i].Z_MIN = Z_MIN;
    chemical_contaminations[i].Z_MAX = Z_MAX;

    /*std::normal_distribution<double> uni_distX((X_MIN + X_MAX) / 2, (X_MAX - X_MIN) / 6);
    std::normal_distribution<double> uni_distY((Y_MIN + Y_MAX) / 2, (Y_MAX - Y_MIN) / 6);
    std::normal_distribution<double> uni_distZ((Z_MIN + Z_MAX) / 2, Z_MIN);*/
    std::normal_distribution<double> uni_distX(chemical_contaminations[i].Position.X(), R_X);
    std::normal_distribution<double> uni_distY(chemical_contaminations[i].Position.Y(), R_Y);
    std::normal_distribution<double> uni_distZ(chemical_contaminations[i].Position.Z(), R_X);

    for (size_t i = 0; i < num_points_; ++i)
    {
      const float fr = static_cast<float>(i) / static_cast<float>(num_points_);
      pcl::PointXYZRGB pt;
      // pt = pcl::PointXYZRGB(fr * back-25, back - fr * back+10, 18 + fr * 20+5);
      pt = pcl::PointXYZRGB(255 * fr, 153 * fr, 1 * fr);

      pt.x = uni_distX(gen);
      pt.y = uni_distY(gen);
      pt.z = uni_distZ(gen);

      this->pcl_point_cloud.points.push_back(pt);
    }
  }
}

void HazardousZones::convertPclToRosMsg(const pcl::PointCloud<pcl::PointXYZRGB> &pcl_cloud, sensor_msgs::PointCloud2 &ros_cloud)
{
  pcl::toROSMsg(pcl_point_cloud, ros_point_cloud);
}

void HazardousZones::generateRadiationPointCloudFromOccupancyGridNormalized()
{
  std::random_device rd;
  std::mt19937 gen(rd());
  size_t num_points_ = radiation_occ_grid_data.data.size();
  double z_on_step = 0.015;
  double z_max = 255 * z_on_step;
  double height = 4.0;
  size_t points_num = 7;

  auto get_points_num =
      [](float val, float val_max, float val_min, float lower_boundary, float upper_boundary)
  {
    // first, normalize on a [0...1] range
    auto val_norm_01 = (val - val_min) / (val_max - val_min);

    // second, normalize on a [l_b...u_b] range
    auto val_norm_lu = lower_boundary + (upper_boundary - lower_boundary) * val_norm_01;

    return round(val_norm_lu);
  };

  for (unsigned int x = 0; x < radiation_occ_grid_data.info.width; x++)
    for (unsigned int y = 0; y < radiation_occ_grid_data.info.height; y++)
    {
      // get cell data in range [0..127, -128, -1]
      signed char cell_data_ = radiation_occ_grid_data.data[x + radiation_occ_grid_data.info.width * y];
      // normalize cell data in range [0, 255]
      unsigned char cell_data = cell_data_;

      pcl::PointXYZRGB pt;
      // an orange color
      pt = pcl::PointXYZRGB(223, 252, 3);

      points_num = get_points_num(cell_data, 255, 0, 0, 20);

      double x_left = x * radiation_occ_grid_data.info.resolution;
      double x_right = x_left + radiation_occ_grid_data.info.resolution;

      double y_left = y * radiation_occ_grid_data.info.resolution;
      double y_right = y_left + radiation_occ_grid_data.info.resolution;

      std::uniform_real_distribution<double> x_rnd(x_left, x_right);
      std::uniform_real_distribution<double> y_rnd(y_left, y_right);
      std::uniform_real_distribution<double> z_rnd(0.1, 3.9);

      for (size_t p = 0; p < points_num; p++)
      {
        pt.x = x_rnd(gen);
        pt.y = y_rnd(gen);
        pt.z = z_rnd(gen);
        pcl_point_cloud.points.push_back(pt);
      }
    }
}

void HazardousZones::generateChemicalPointCloudFromOccupancyGridNormalized()
{
  std::random_device rd;
  std::mt19937 gen(rd());
  size_t num_points_ = chemical_occ_grid_data.data.size();
  double z_on_step = 0.015;
  double z_max = 255 * z_on_step;
  double height = 4.0;
  size_t points_num = 7;

  auto get_points_num =
      [](float val, float val_max, float val_min, float lower_boundary, float upper_boundary)
  {
    // first, normalize on a [0...1] range
    auto val_norm_01 = (val - val_min) / (val_max - val_min);

    // second, normalize on a [l_b...u_b] range
    auto val_norm_lu = lower_boundary + (upper_boundary - lower_boundary) * val_norm_01;

    return round(val_norm_lu);
  };

  for (unsigned int x = 0; x < chemical_occ_grid_data.info.width; x++)
    for (unsigned int y = 0; y < chemical_occ_grid_data.info.height; y++)
    {
      // get cell data in range [0..127, -128, -1]
      signed char cell_data_ = chemical_occ_grid_data.data[x + chemical_occ_grid_data.info.width * y];
      // normalize cell data in range [0, 255]
      unsigned char cell_data = cell_data_;

      pcl::PointXYZRGB pt;
      // an orange color
      pt = pcl::PointXYZRGB(255, 153, 1);

      points_num = get_points_num(cell_data, 255, 0, 0, 20);

      double x_left = x * chemical_occ_grid_data.info.resolution;
      double x_right = x_left + chemical_occ_grid_data.info.resolution;

      double y_left = y * chemical_occ_grid_data.info.resolution;
      double y_right = y_left + chemical_occ_grid_data.info.resolution;

      std::uniform_real_distribution<double> x_rnd(x_left, x_right);
      std::uniform_real_distribution<double> y_rnd(y_left, y_right);
      std::uniform_real_distribution<double> z_rnd(0.1, 3.9);

      for (size_t p = 0; p < points_num; p++)
      {
        pt.x = x_rnd(gen);
        pt.y = y_rnd(gen);
        pt.z = z_rnd(gen);
        pcl_point_cloud.points.push_back(pt);
      }
    }
}

void HazardousZones::generateRadiationZonesPointCloud()
{
  std::random_device rd;
  std::mt19937 gen(rd());
  size_t num_points_ = 100000;

  for (int i = 0; i < this->radiation_zones_count; i++)
  {
    double R = this->radiation_contaminations[i].Radius;
    double X = this->radiation_contaminations[i].Origin.X();
    double Y = this->radiation_contaminations[i].Origin.Y();
    double Z = this->radiation_contaminations[i].Origin.Z();

    /*std::uniform_real_distribution<double> uni_distX(X-R, X+R);
    std::uniform_real_distribution<double> uni_distY(Y-R, Y+R);
    std::uniform_real_distribution<double> uni_distZ(Z-R, Z+R);

    std::normal_distribution<> distrX(X, R);
    std::normal_distribution<> distrY(Y, R);
    std::normal_distribution<> distrZ(Z, R);*/
    std::uniform_real_distribution<double> angleDistribution(0, 1);

    for (int i = 0; i < num_points_; ++i)
    {
      const float fr = static_cast<float>(i) / static_cast<float>(num_points_);
      pcl::PointXYZRGB pt;
      pt = pcl::PointXYZRGB(fr * 255, 255 - fr * 255, 18 + fr * 20);

      double u = angleDistribution(gen);
      double v = angleDistribution(gen);
      double theta = 2 * M_PI * u;
      double phi = std::acos(2 * v - 1);

      pt.x = X + (R * std::sin(phi) * std::cos(theta));
      pt.y = Y + (R * std::sin(phi) * std::sin(theta));
      pt.z = Z + (R * std::cos(phi));

      this->pcl_point_cloud.points.push_back(pt);
    }
  }
}
