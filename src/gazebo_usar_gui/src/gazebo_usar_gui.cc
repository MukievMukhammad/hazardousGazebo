#include <sstream>
#include "gazebo_usar_gui.hh"
#include <QSettings>
#include <ros/package.h>
#include <math.h>

using namespace gazebo;
// Plugin registration macros for gui plugin.
GZ_REGISTER_GUI_PLUGIN(GazeboUsarGui)

void GazeboUsarGui::Load(sdf::ElementPtr sdf)
{
  if(sdf->HasElement("robots")) {
    std::string robots_data = sdf->Get<std::string>("robots");
    if(!robots_data.empty()) {
      parseRobotsTagData(robots_data);
      robots_names_parsed_succeed = true;
      visualizeWidgets(robots_list);
    } else {
      ROS_ERROR("<robots> tag data is empty!\n");
    }
  } else {
    ROS_ERROR("Couldn't find <robots> tag!\n");
  }

  if(robots_names_parsed_succeed)
    ROS_WARN("Parsing the names of the robot is done successfully!");
  else {
    ROS_ERROR("Parsing the names of the robot is failed!");
    return;
  }
}

GazeboUsarGui::GazeboUsarGui()
  : GUIPlugin()
{
  int argc = 0;
  char** argv = nullptr;
  ros::init(argc, argv, "gazebo_usar_gui");
  ros_node.reset(new ros::NodeHandle("gazebo_usar_gui"));

  //QSettings settings("Qt-Ros Package", "microsimulator");
  resources_path = QString::fromUtf8((ros::package::getPath("gazebo_usar_gui") + "/resource").c_str()); //settings.value("lirs_gazebo_resources_path", "").toString();

  QObject::connect(this, SIGNAL(updateChemicalProgressBarsSignal()), this, SLOT(updateChemicalProgressBars()));
  QObject::connect(this, SIGNAL(updateRadiationProgressBarsSignal()), this, SLOT(updateRadiationProgressBars()));
  QObject::connect(this, SIGNAL(updateChemicalConcentrationProgressBarsSignal()), this, SLOT(updateChemicalConcentrationProgressBars()));

  ros::SubscribeOptions radiation_substance_topic_description =
      ros::SubscribeOptions::create<gazebo_usar_core::RadiationSubstances>(
          "/radiation_substances",
          10,
          boost::bind(&GazeboUsarGui::receiveRadiationData, this, _1),
          ros::VoidPtr(), &radiation_queue);
  radiation_sub = ros_node->subscribe(radiation_substance_topic_description);
  radiation_thread = std::thread(boost::bind(&GazeboUsarGui::radiationQueueThread, this));

  ros::SubscribeOptions chemical_substance_topic_description =
      ros::SubscribeOptions::create<gazebo_usar_core::ChemicalSubstances>(
          "/chemical_substances",
          10,
          boost::bind(&GazeboUsarGui::receiveChemicalData, this, _1),
          ros::VoidPtr(), &chemical_queue);
  chemical_sub = ros_node->subscribe(chemical_substance_topic_description);
  chemical_thread = std::thread(std::bind(&GazeboUsarGui::chemicalQueueThread, this));

  ros::SubscribeOptions chemical_concentration_topic_description =
      ros::SubscribeOptions::create<gazebo_usar_core::ChemicalConcentration>(
          "/chemical_concentration",
          10,
          boost::bind(&GazeboUsarGui::receiveChemicalConcentration, this, _1),
          ros::VoidPtr(), &chemical_concentration_queue);
  chemical_concentration_sub = ros_node->subscribe(chemical_concentration_topic_description);
  chemical_concentration_thread = std::thread(std::bind(&GazeboUsarGui::chemicalConcentrationQueueThread, this));
}

GazeboUsarGui::~GazeboUsarGui()
{
  ROS_WARN("Clearing GazeboUsarGui plugin");
  ros::shutdown();
  if(chemical_thread.joinable())
    chemical_thread.join();
  if(radiation_thread.joinable())
    radiation_thread.join();
}

void GazeboUsarGui::parseRobotsTagData(std::string &robots_data)
{
  std::istringstream robots;
  robots.str(robots_data);
  std::string robot;
  while(std::getline(robots, robot, ';')) {
    if(!robot.empty())
      robots_list.push_back(robot);
  }
}

double truncateNumber(double In, unsigned int Digits)
{
    double f=pow(10, Digits);
    return ((int)(In*f))/f;
}

bool inBounds(double value, double low, double high)
{
  return low <= value && value < high;
}

void GazeboUsarGui::receiveRadiationData(const gazebo_usar_core::RadiationSubstancesConstPtr &msg)
{
  if(msg.get()) {
    radiation_substances_data = *msg;
    emit updateRadiationProgressBarsSignal();
  }
}

void GazeboUsarGui::receiveChemicalData(const gazebo_usar_core::ChemicalSubstancesConstPtr &msg)
{
  if(msg.get()) {
    chemical_substances_data = *msg;
    emit updateChemicalProgressBarsSignal();
  }
}

void GazeboUsarGui::receiveChemicalConcentration(const gazebo_usar_core::ChemicalConcentrationConstPtr &msg)
{
  if(msg.get()) {
    chemical_concentration = *msg;
    emit updateChemicalConcentrationProgressBarsSignal();
  }
}

void GazeboUsarGui::radiationQueueThread()
{
  while (!ros::isShuttingDown()) {
    radiation_queue.callAvailable(ros::WallDuration(callback_wait_time));
  }
}

void GazeboUsarGui::chemicalQueueThread()
{
  while (!ros::isShuttingDown()) {
    chemical_queue.callAvailable(ros::WallDuration(callback_wait_time));
  }
}

void GazeboUsarGui::chemicalConcentrationQueueThread()
{
  while (!ros::isShuttingDown()) {
    chemical_concentration_queue.callAvailable(ros::WallDuration(callback_wait_time));
  }
}

void GazeboUsarGui::updateRadiationProgressBars()
{
    if(radiation_substances_data.radiationSubstances.size())
    for(auto& robot_info: radiation_substances_data.radiationSubstances)
    {
      std::string robot_name = robot_info.robot_name;
      double rad = robot_info.radiation_accumulated;
      double max = robot_info.max;
      double min = robot_info.min;

      if(radiation_data_widget.contains(QString::fromStdString(robot_name)))
      {
        auto so = radiation_data_widget[QString::fromStdString(robot_name)];
        if(so) {
          auto numb = so->text().toDouble();
          numb += rad;
          so->setText(QString::number(numb));
        }
      }

      if(radiation_visualize_widget.contains(QString::fromStdString(robot_name)))
      {
        auto pb = radiation_visualize_widget[QString::fromStdString(robot_name)];
        if(pb.get()) {
          pb->setRange(min*10000, max*10000);
          if(inBounds(rad*10000, pb->minimum(), pb->maximum()))
            pb->setValue(rad*10000);
        }
      }
    }
}

void GazeboUsarGui::updateChemicalProgressBars()
{
  if(chemical_substances_data.chemicalSubstances.size())
  for(auto& robot_info: chemical_substances_data.chemicalSubstances)
  {
    std::string robot_name = robot_info.robot_name;
    double rad = robot_info.chemical_accumulated;
    double max = robot_info.max;
    double min = robot_info.min;
    std::mutex mtx;

    if(chemical_data_widget.contains(QString::fromStdString(robot_name)))
    {
      auto so = chemical_data_widget[QString::fromStdString(robot_name)];
      if(so) {
        auto numb = so->text().toDouble();
        numb += rad;
        so->setText(QString::number(numb));
      }
    }
    if(chemical_visualize_widget.contains(QString::fromStdString(robot_name)))
    {
      auto pb = chemical_visualize_widget[QString::fromStdString(robot_name)];
      if(pb.get()) {
        pb->setRange(min*10000, max*10000);
        if(inBounds(rad*10000, pb->minimum(), pb->maximum()))
          pb->setValue(rad*10000);
      }
    }
  }
}

void GazeboUsarGui::updateChemicalConcentrationProgressBars()
{
  // if(chemical_concentration)
  // for(auto& robot_info: chemical_substances_data.chemicalSubstances)
  {
    auto& robot_info = chemical_concentration;
    std::string robot_name = robot_info.robot_name;
    double concentration = robot_info.concentration_value;

    // Robot accumulated data update
    double numb;
    if(chemical_data_widget.contains(QString::fromStdString(robot_name)))
    {
      auto so = chemical_data_widget[QString::fromStdString(robot_name)];
      if(so) {
        float con = 1-concentration;
        con = roundf(con * 100) / 100;
        numb = so->text().toDouble();
        numb += con;
        so->setText(QString::number(numb));
      }
    }
    if(chemical_visualize_widget.contains(QString::fromStdString(robot_name)))
    {
      auto pb = chemical_visualize_widget[QString::fromStdString(robot_name)];
      if(pb.get()) {
        pb->setRange(0, 100);
        pb->setValue(numb);
      }
    }

    // Sensor data update
    if(chemical_sensor_data_widget.contains(QString::fromStdString(robot_name)))
    {
      auto so = chemical_sensor_data_widget[QString::fromStdString(robot_name)];
      if(so) {
        float numb = 1-concentration;
        numb = roundf(numb * 100) / 100;
        so->setText(QString::number(numb));
      }
    }
    if(chemical_sensor_visualize_widget.contains(QString::fromStdString(robot_name))){
      auto sensor_pb = chemical_sensor_visualize_widget[QString::fromStdString(robot_name)];
      if(sensor_pb.get()){
        sensor_pb->setRange(0, 1000);
        sensor_pb->setValue((1-concentration)*1000);
      }
    }
  }
}

void GazeboUsarGui::visualizeWidgets(std::vector<std::string>& robots_names)
{
  this->setStyleSheet(
      "QFrame { background-color : rgba(100, 100, 100, 255); color : white; }");

  QGridLayout *mainLayout = new QGridLayout();
  // Create the frames to hold all the widgets
  QFrame *radiationIconFrame = new QFrame();
  QFrame *robotsRadiationSubstancesFrame = new QFrame();
  QFrame *chemicalIconFrame = new QFrame();
  QFrame *robotsChemicalSubstancesFrame = new QFrame();

  // Create the layout that sits inside the frame
  QHBoxLayout *chemicalIconFrameLayout = new QHBoxLayout();
  QVBoxLayout *chemicalSubstancesFrameLayout = new QVBoxLayout();
  QHBoxLayout *radiationIconFrameLayout = new QHBoxLayout();
  QVBoxLayout *radiationSubstancesFrameLayout = new QVBoxLayout();

  QLabel* chemicalIcon = new QLabel();
  QPixmap chemicalIconPix(resources_path + "/gazebo_icons/lirs_chemical.png");
  chemicalIconPix.scaled(30, 30, Qt::AspectRatioMode::KeepAspectRatio);
  chemicalIcon->setPixmap(chemicalIconPix);

  QLabel* radiationIcon = new QLabel();
  QPixmap radiationIconPix(resources_path + "/gazebo_icons/lirs_radiation.png");
  radiationIconPix.scaled(30, 30, Qt::AspectRatioMode::KeepAspectRatio);
  radiationIcon->setPixmap(radiationIconPix);

  for(size_t i = 0; i < robots_names.size(); i++)
  {
    QHBoxLayout *robotDataLayout = new QHBoxLayout();
    QHBoxLayout *progressbarLayout = new QHBoxLayout();
    QLabel *robotName = new QLabel(robots_names[i].c_str());
    auto so = std::make_shared<QLabel>(tr("0.0"));
    radiation_data_widget.insert(QString::fromStdString(robots_names[i]), so);
    auto pb = std::make_shared<QProgressBar>();
    radiation_visualize_widget.insert(QString::fromStdString(robots_names[i]), pb);
    robotDataLayout->addWidget(robotName);
    robotDataLayout->addWidget(so.get());
    robotDataLayout->addWidget(new QLabel(tr(" mSv")));
    progressbarLayout->addWidget(pb.get());
    radiationSubstancesFrameLayout->addLayout(robotDataLayout);
    radiationSubstancesFrameLayout->addLayout(progressbarLayout);
  }

  for(size_t i = 0; i < robots_names.size(); i++)
  {
    QHBoxLayout *robotDataLayout = new QHBoxLayout();
    QHBoxLayout *progressbarLayout = new QHBoxLayout();
    QLabel *robotName = new QLabel(robots_names[i].c_str());
    auto so = std::make_shared<QLabel>(tr("0.0"));
    chemical_data_widget.insert(QString::fromStdString(robots_names[i]), so);
    auto pb = std::make_shared<QProgressBar>();
    chemical_visualize_widget.insert(QString::fromStdString(robots_names[i]), pb);
    robotDataLayout->addWidget(robotName);
    robotDataLayout->addWidget(so.get());
    robotDataLayout->addWidget(new QLabel(tr(" mGy")));
    progressbarLayout->addWidget(pb.get());
    chemicalSubstancesFrameLayout->addLayout(robotDataLayout);
    chemicalSubstancesFrameLayout->addLayout(progressbarLayout);

    // Add sensor progressbar
    QHBoxLayout *sensorDataLayout = new QHBoxLayout();
    QHBoxLayout *sensorProgressbarLayout = new QHBoxLayout();
    auto sensor_pb = std::make_shared<QProgressBar>();
    chemical_sensor_visualize_widget.insert(QString::fromStdString(robots_names[i]), sensor_pb);
    auto concentration_value = std::make_shared<QLabel>(tr("0.0"));
    chemical_sensor_data_widget.insert(QString::fromStdString(robots_names[i]), concentration_value);
    QLabel *sensorName = new QLabel(QString::fromStdString(robots_names[i] + "_sensor"));
    sensorDataLayout->addWidget(sensorName);
    sensorDataLayout->addWidget(concentration_value.get());
    sensorDataLayout->addWidget(new QLabel(tr(" mGy")));
    sensorProgressbarLayout->addWidget(sensor_pb.get());
    chemicalSubstancesFrameLayout->addLayout(sensorDataLayout);
    chemicalSubstancesFrameLayout->addLayout(sensorProgressbarLayout);
  }

  // Add the label to the frame's layout  
  radiationIconFrameLayout->addWidget(radiationIcon, Qt::AlignHCenter);
  radiationIconFrameLayout->setAlignment(radiationIcon, Qt::AlignHCenter);

  chemicalIconFrameLayout->addWidget(chemicalIcon, Qt::AlignHCenter);
  chemicalIconFrameLayout->setAlignment(chemicalIcon, Qt::AlignHCenter);

  // Add frameLayout to the frame
  radiationIconFrame->setLayout(radiationIconFrameLayout);
  robotsRadiationSubstancesFrame->setLayout(radiationSubstancesFrameLayout);
  chemicalIconFrame->setLayout(chemicalIconFrameLayout);
  robotsChemicalSubstancesFrame->setLayout(chemicalSubstancesFrameLayout);


  // Add the frame to the main layout
  mainLayout->addWidget(radiationIconFrame);
  mainLayout->addWidget(robotsRadiationSubstancesFrame);
  mainLayout->addWidget(chemicalIconFrame);
  mainLayout->addWidget(robotsChemicalSubstancesFrame);

  this->setLayout(mainLayout);

  // Position and resize this widget
  this->move(10, 10);
  this->resize(215, 355);
}
