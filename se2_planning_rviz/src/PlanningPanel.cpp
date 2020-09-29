/*
 *  Author: Edo Jelavic
 *  Institute: ETH Zurich, Robotic Systems Lab
 */

/* Original by:

BSD 3-Clause License

Copyright (c) 2018, ETHZ ASL
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */

#include <stdio.h>
#include <functional>

#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QPainter>
#include <QTimer>
#include <QVBoxLayout>
#include <QGroupBox>
#include <QFormLayout>
#include <QScrollArea>

#include <geometry_msgs/Twist.h>
#include <rviz/visualization_manager.h>

#include "se2_planning_rviz/PlanningPanel.hpp"
#include "se2_planning_rviz/PoseWidget.hpp"
#include "se2_planning_rviz/EditButton.hpp"

#include "se2_navigation_msgs/PathRequestMsg.h"
#include "se2_navigation_msgs/RequestPathSrv.h"
#include "se2_navigation_msgs/RequestCurrentStateSrv.h"
#include "se2_navigation_msgs/ControllerCommand.hpp"
#include "se2_navigation_msgs/SendControllerCommandSrv.h"

#include "spacebok_msgs/SpacebokControllerState.h"
#include "spacebok_msgs/SpacebokHighlevelState.h"

#include <thread>

namespace se2_planning_rviz {

template<typename Req, typename Res>
bool callService(Req& req, Res& res, const std::string& serviceName)
{
  try {
    // ROS_DEBUG_STREAM("Service name: " << service_name);
    if (!ros::service::call(serviceName, req, res)) {
      ROS_WARN_STREAM("Couldn't call service: " << serviceName);
      return false;
    }
  } catch (const std::exception& e) {
    ROS_ERROR_STREAM("Service Exception: " << e.what());
  }

  return true;
}

PlanningPanel::PlanningPanel(QWidget* parent)
    : rviz::Panel(parent),
      nh_(ros::NodeHandle()),
      interactive_markers_(nh_, "se2_planning_markers")
{

  createLayout();

}

void PlanningPanel::onInitialize()
{
  const double markerScale = 3.0;
  interactive_markers_.initialize(se2_visualization_ros::Color::Orange(), markerScale);
  interactive_markers_.setPoseUpdatedCallback(
      std::bind(&PlanningPanel::updateInteractiveMarkerPose, this, std::placeholders::_1));

  ROS_INFO_STREAM("Fixed frame: " << vis_manager_->getFixedFrame().toStdString());
  interactive_markers_.setFrameId(vis_manager_->getFixedFrame().toStdString());
  // Initialize all the markers.
  for (const auto& kv : pose_widget_map_) {
    geometry_msgs::Pose state;
    kv.second->getPose(&state);
    interactive_markers_.enableMarker(kv.first, state);
  }

  spacebokHighlevelStatePublisher_ = nh_.advertise<spacebok_msgs::SpacebokHighlevelState>(
    "/highlevel_state_command", 1);
  spacebokControllerStatePublisher_ = nh_.advertise<spacebok_msgs::SpacebokControllerState>(
    "/controller_state_command", 1);
  spacebokControlCommandPublisher_ = nh_.advertise<spacebok_msgs::SpacebokHighlevelCommands>(
    "/highlevel_commands", 1);
  spacebokPathManagerInfoSubscriber_ = nh_.subscribe("/spacebok_path_manager/info",
                                                     1,
                                                     &PlanningPanel::pathManagerInfoCallback,
                                                     this);
}

void PlanningPanel::createLayout()
{
  QGroupBox *formGroupBox = new QGroupBox(tr("Topics and Services"));
  QFormLayout *topic_layout = new QFormLayout;
  planningServiceNameEditor_ = new QLineEdit;
  controllerCommandTopicEditor_ = new QLineEdit;
  currStateServiceEditor_ = new QLineEdit;
  loadGlobalPathServiceEditor_ = new QLineEdit;
  globalPathFileNameEditor_ = new QLineEdit;
  pathManagerCommandServiceEditor_ = new QLineEdit;
  topic_layout->addRow(new QLabel(tr("Ctrl command topic:")), controllerCommandTopicEditor_);
  topic_layout->addRow(new QLabel(tr("Planning service:")), planningServiceNameEditor_);
  topic_layout->addRow(new QLabel(tr("Curr State Service:")), currStateServiceEditor_);
  topic_layout->addRow(new QLabel(tr("Load Global Path Service:")), loadGlobalPathServiceEditor_);
  topic_layout->addRow(new QLabel(tr("Global Path File Name:")), globalPathFileNameEditor_);
  topic_layout->addRow(new QLabel(tr("Path Manager Command Service:")), pathManagerCommandServiceEditor_);
  formGroupBox->setLayout(topic_layout);

  // Start and goal poses.
  QGridLayout* start_goal_layout = new QGridLayout;

  // Minimums...
  start_goal_layout->setColumnMinimumWidth(0, 50);
  start_goal_layout->setColumnMinimumWidth(1, 245);
  start_goal_layout->setColumnMinimumWidth(2, 80);
  start_goal_layout->setRowMinimumHeight(0, 55);
  start_goal_layout->setRowMinimumHeight(1, 55);
  start_goal_layout->setColumnStretch(0, 1);
  start_goal_layout->setColumnStretch(1, 9);
  start_goal_layout->setColumnStretch(2, 3);

  start_pose_widget_ = new PoseWidget("start");
  goal_pose_widget_ = new PoseWidget("goal");
  EditButton* start_edit_button = new EditButton("start");
  EditButton* goal_edit_button = new EditButton("goal");
  registerPoseWidget(start_pose_widget_);
  registerPoseWidget(goal_pose_widget_);
  registerEditButton(start_edit_button);
  registerEditButton(goal_edit_button);

  currentStateAsStartCheckBox_ = new QCheckBox();

  start_goal_layout->addWidget(new QLabel("Start:"), 0, 0, Qt::AlignTop);
  start_goal_layout->addWidget(start_pose_widget_, 0, 1);
  start_goal_layout->addWidget(start_edit_button, 0, 2);
  start_goal_layout->addWidget(new QLabel("Goal:"), 1, 0, Qt::AlignTop);
  start_goal_layout->addWidget(goal_pose_widget_, 1, 1);
  start_goal_layout->addWidget(goal_edit_button, 1, 2);
  start_goal_layout->addWidget(currentStateAsStartCheckBox_, 2, 0);
  start_goal_layout->addWidget(new QLabel("Start == current position"), 2, 1);

  // Global planner services
  QGroupBox *globalNavigationGroupBox = new QGroupBox(tr("Global Navigation"));
  QHBoxLayout* service_global_path_layout = new QHBoxLayout;
  global_plan_request_button_ = new QPushButton("Request Global Plan");
  global_tracking_command_button_ = new QPushButton("Start Tracking");
  global_stop_command_button_ = new QPushButton("Stop Tracking");
  service_global_path_layout->addWidget(global_plan_request_button_);
  service_global_path_layout->addWidget(global_tracking_command_button_);
  service_global_path_layout->addWidget(global_stop_command_button_);
  service_global_path_layout->setContentsMargins(0, 0, 0, 0);
  globalNavigationGroupBox->setLayout(service_global_path_layout);

  //
  global_path_manager_info_list_ = new QListWidget();

  // Planner services and publications.
  QGroupBox *localNavigationGroupBox = new QGroupBox(tr("Local Navigation"));
  QHBoxLayout* service_layout = new QHBoxLayout;
  plan_request_button_ = new QPushButton("Request Plan");
  tracking_command_button_ = new QPushButton("Start Tracking");
  stop_command_button_ = new QPushButton("Stop Tracking");
  service_layout->addWidget(plan_request_button_);
  service_layout->addWidget(tracking_command_button_);
  service_layout->addWidget(stop_command_button_);
  service_layout->setContentsMargins(0, 0, 0, 0);
  localNavigationGroupBox->setLayout(service_layout);

  // Spacebok services.
  QGroupBox *spacebokGroupBox = new QGroupBox(tr("Spacebok Interface"));
  QHBoxLayout* service_spacebok_layout = new QHBoxLayout;
  spacebok_standup_button_ = new QPushButton("Standup");
  spacebok_init_button_ = new QPushButton("Init");
  spacebok_start_button_ = new QPushButton("Walk");
  spacebok_stop_button_ = new QPushButton("Stop");
  spacebok_start_walk_init_button_ = new QPushButton("Walk Init");
  spacebok_stop_walk_init_button_ = new QPushButton("Stop Init");
  service_spacebok_layout->addWidget(spacebok_standup_button_);
  service_spacebok_layout->addWidget(spacebok_init_button_);
  service_spacebok_layout->addWidget(spacebok_start_button_);
  service_spacebok_layout->addWidget(spacebok_stop_button_);
  service_spacebok_layout->addWidget(spacebok_start_walk_init_button_);
  service_spacebok_layout->addWidget(spacebok_stop_walk_init_button_);
  service_spacebok_layout->setContentsMargins(0, 0, 0, 0);
  spacebokGroupBox->setLayout(service_spacebok_layout);

  // GPS orientation estimation services.
  QGroupBox *orientationEstimationGroupBox = new QGroupBox(tr("GPS Orientation Estimation"));
  QHBoxLayout* service_orientation_estimation_layout = new QHBoxLayout;
  orientation_estimation_reset_button_ = new QPushButton("Reset Orientation Estimation");
  service_orientation_estimation_layout->addWidget(orientation_estimation_reset_button_, 1, Qt::AlignTop);
  service_orientation_estimation_layout->setContentsMargins(0, 0, 0, 0);
  orientationEstimationGroupBox->setLayout(service_orientation_estimation_layout);

  // Elevation mapping cupy services.
  QGroupBox *elevationMappingCupyGroupBox = new QGroupBox(tr("Elevation Mapping Cupy"));
  QHBoxLayout* elevation_mapping_cupy_layout = new QHBoxLayout;
  elevation_mapping_cupy_init_button_ = new QPushButton("Init from foot positions");
  elevation_mapping_cupy_reset_button_ = new QPushButton("Reset");
  elevation_mapping_cupy_layout->addWidget(elevation_mapping_cupy_init_button_, 1, Qt::AlignTop);
  elevation_mapping_cupy_layout->addWidget(elevation_mapping_cupy_reset_button_, 1, Qt::AlignTop);
  elevation_mapping_cupy_layout->setContentsMargins(0, 0, 0, 0);
  elevationMappingCupyGroupBox->setLayout(elevation_mapping_cupy_layout);

  // First the names, then the start/goal, then service buttons.
  QVBoxLayout* layout = new QVBoxLayout;
  layout->addWidget(formGroupBox);
  layout->addLayout(start_goal_layout);
  layout->addWidget(globalNavigationGroupBox);
  layout->addWidget(global_path_manager_info_list_);
  layout->addWidget(localNavigationGroupBox);
  layout->addWidget(spacebokGroupBox);
  layout->addWidget(orientationEstimationGroupBox);
  layout->addWidget(elevationMappingCupyGroupBox);

  QWidget* scrollAreaContent = new QWidget;
  scrollAreaContent->setLayout( layout );
  QScrollArea* scrollArea = new QScrollArea;
  scrollArea->setHorizontalScrollBarPolicy( Qt::ScrollBarAlwaysOff );
  scrollArea->setVerticalScrollBarPolicy( Qt::ScrollBarAsNeeded );
  scrollArea->setWidgetResizable( true );
  scrollArea->setWidget( scrollAreaContent );

  QVBoxLayout *mainLayout = new QVBoxLayout;
  mainLayout->addWidget (scrollArea);
  setLayout (mainLayout);

  //set the default parameters
  currentStateAsStartCheckBox_->setChecked(false);

  // Hook up connections.
  connect(controllerCommandTopicEditor_, SIGNAL(editingFinished()), this,
          SLOT(updateControllerCommandTopic()));
  connect(planningServiceNameEditor_, SIGNAL(editingFinished()), this,
          SLOT(updatePathRequestTopic()));
  connect(currStateServiceEditor_, SIGNAL(editingFinished()), this,
          SLOT(updateGetCurrentStateService()));
  connect(loadGlobalPathServiceEditor_, SIGNAL(editingFinished()), this,
          SLOT(updateLoadGlobalPathService()));
  connect(globalPathFileNameEditor_, SIGNAL(editingFinished()), this,
          SLOT(updateGlobalPathFileName()));
  connect(pathManagerCommandServiceEditor_, SIGNAL(editingFinished()), this,
          SLOT(updatePathManagerCommandService()));
  connect(global_plan_request_button_, SIGNAL(released()), this, SLOT(callGlobalPlanningService()));
  connect(global_tracking_command_button_, SIGNAL(released()), this, SLOT(callPublishGlobalTrackingCommand()));
  connect(global_stop_command_button_, SIGNAL(released()), this, SLOT(callPublishGlobalStopTrackingCommand()));
  connect(plan_request_button_, SIGNAL(released()), this, SLOT(callPlanningService()));
  connect(tracking_command_button_, SIGNAL(released()), this, SLOT(callPublishTrackingCommand()));
  connect(stop_command_button_, SIGNAL(released()), this, SLOT(callPublishStopTrackingCommand()));
  connect(spacebok_standup_button_, SIGNAL(released()), this, SLOT(callPublishSpacebokStandUpCommand()));
  connect(spacebok_init_button_, SIGNAL(released()), this, SLOT(executeSpacebokStandbyInitMotion()));
  connect(spacebok_start_button_, SIGNAL(released()), this, SLOT(callPublishSpacebokStartCommand()));
  connect(spacebok_stop_button_, SIGNAL(released()), this, SLOT(callPublishSpacebokStopCommand()));
  connect(spacebok_start_walk_init_button_, SIGNAL(released()), this, SLOT(startSpacebokWalkingInitMotion()));
  connect(spacebok_stop_walk_init_button_, SIGNAL(released()), this, SLOT(stopSpacebokWalkingInitMotion()));
  connect(orientation_estimation_reset_button_, SIGNAL(released()), this, SLOT(callResetOrientationEstimation()));
  connect(elevation_mapping_cupy_init_button_, SIGNAL(released()), this, SLOT(callInitElevationMappingCupy()));
  connect(elevation_mapping_cupy_reset_button_, SIGNAL(released()), this, SLOT(callResetElevationMappingCupy()));
}

void PlanningPanel::updateControllerCommandTopic()
{
  setControllerCommandTopic(controllerCommandTopicEditor_->text());
}

// Set the topic name we are publishing to.
void PlanningPanel::setControllerCommandTopic(const QString& newControllerCommandTopic)
{
  // Only take action if the name has changed.
  if (newControllerCommandTopic != controllerCommandTopicName_) {
    controllerCommandTopicName_ = newControllerCommandTopic;
    Q_EMIT configChanged();
  }
}
//////////////////
void PlanningPanel::updateGetCurrentStateService()
{
  setGetCurrentStateService(currStateServiceEditor_->text());
}

void PlanningPanel::setGetCurrentStateService(const QString& newCurrentStateService)
{
  if (newCurrentStateService != currentStateServiceName_) {
    currentStateServiceName_ = newCurrentStateService;
    Q_EMIT configChanged();
  }
}
/////////////////////
void PlanningPanel::updatePathRequestTopic()
{
  setPathRequestTopic(planningServiceNameEditor_->text());
}

void PlanningPanel::setPathRequestTopic(const QString& newPathRequestTopicName)
{
  if (newPathRequestTopicName != planningServiceName_) {
    planningServiceName_ = newPathRequestTopicName;
    Q_EMIT configChanged();
  }
}
/////////////////////
void PlanningPanel::updateLoadGlobalPathService()
{
  setLoadGlobalPathService(loadGlobalPathServiceEditor_->text());
}

void PlanningPanel::setLoadGlobalPathService(const QString& newLoadGlobalPathServiceName)
{
  if (newLoadGlobalPathServiceName != loadGlobalPathServiceName_) {
    loadGlobalPathServiceName_ = newLoadGlobalPathServiceName;
    Q_EMIT configChanged();
  }
}
/////////////////////
void PlanningPanel::updateGlobalPathFileName()
{
  setGlobalPathFileName(globalPathFileNameEditor_->text());
}

void PlanningPanel::setGlobalPathFileName(const QString &newGlobalPathFileName)
{
  if (newGlobalPathFileName != globalPathFileName_) {
    globalPathFileName_ = newGlobalPathFileName;
    Q_EMIT configChanged();
  }
}
/////////////////////
void PlanningPanel::updatePathManagerCommandService()
{
  setPathManagerCommandService(pathManagerCommandServiceEditor_->text());
}

void PlanningPanel::setPathManagerCommandService(const QString& newPathManagerCommandServiceName)
{
  if (newPathManagerCommandServiceName != pathManagerCommandServiceName_) {
    pathManagerCommandServiceName_ = newPathManagerCommandServiceName;
    Q_EMIT configChanged();
  }
}

void PlanningPanel::startEditing(const std::string& id)
{
  //ROS_INFO_STREAM("Id: " << id << " Currently editing: " << currently_editing_);
  // Make sure nothing else is being edited.
  if (!currently_editing_.empty()) {
    auto search = edit_button_map_.find(currently_editing_);
    if (search != edit_button_map_.end()) {
      search->second->finishEditing();
    }
  }
  currently_editing_ = id;
  // Get the current pose:
  auto search = pose_widget_map_.find(currently_editing_);
  if (search == pose_widget_map_.end()) {
    return;
  }
  // Update fixed frame (may have changed since last time):
  interactive_markers_.setFrameId(vis_manager_->getFixedFrame().toStdString());

  geometry_msgs::Pose state;
  search->second->getPose(&state);
  interactive_markers_.enableSetPoseMarker(state);
  interactive_markers_.disableMarker(id);
}

void PlanningPanel::finishEditing(const std::string& id)
{
  if (currently_editing_ == id) {
    currently_editing_.clear();
    interactive_markers_.disableSetPoseMarker();
  }
  auto search = pose_widget_map_.find(id);
  if (search == pose_widget_map_.end()) {
    return;
  }
//  ros::spinOnce();
  geometry_msgs::Pose pose;

  search->second->getPose(&pose);

  interactive_markers_.enableMarker(id, pose);
}

void PlanningPanel::registerPoseWidget(PoseWidget* widget)
{
  pose_widget_map_[widget->id()] = widget;
connect(widget, SIGNAL(poseUpdated(const std::string&,
            geometry_msgs::Pose&)),
    this, SLOT(widgetPoseUpdated(const std::string&,
            geometry_msgs::Pose&)));
}

void PlanningPanel::registerEditButton(EditButton* button)
{
edit_button_map_[button->id()] = button;
connect(button, SIGNAL(startedEditing(const std::string&)), this,
  SLOT(startEditing(const std::string&)));
connect(button, SIGNAL(finishedEditing(const std::string&)), this,
  SLOT(finishEditing(const std::string&)));
}

// Save all configuration data from this panel to the given
// Config object.  It is important here that you call save()
// on the parent class so the class id and panel name get saved.
void PlanningPanel::save(rviz::Config config) const
{
rviz::Panel::save(config);
config.mapSetValue("path_request_topic", planningServiceName_);
config.mapSetValue("get_current_state_service", currentStateServiceName_);
config.mapSetValue("controller_command_topic", controllerCommandTopicName_);
config.mapSetValue("load_global_path_service_name", loadGlobalPathServiceName_);
config.mapSetValue("global_path_file_name", globalPathFileName_);
config.mapSetValue("path_manager_command_service_name", pathManagerCommandServiceName_);
}

// Load all configuration data for this panel from the given Config object.
void PlanningPanel::load(const rviz::Config& config)
{
rviz::Panel::load(config);
QString topic;
if (config.mapGetString("path_request_topic", &planningServiceName_)) {
planningServiceNameEditor_->setText(planningServiceName_);
}

if (config.mapGetString("get_current_state_service", &currentStateServiceName_)) {
currStateServiceEditor_->setText(currentStateServiceName_);
}

if (config.mapGetString("controller_command_topic", &controllerCommandTopicName_)) {
controllerCommandTopicEditor_->setText(controllerCommandTopicName_);
}

if (config.mapGetString("load_global_path_service_name", &loadGlobalPathServiceName_)) {
  loadGlobalPathServiceEditor_->setText(loadGlobalPathServiceName_);
}

if (config.mapGetString("global_path_file_name", &globalPathFileName_)) {
  globalPathFileNameEditor_->setText(globalPathFileName_);
}

if (config.mapGetString("path_manager_command_service_name", &pathManagerCommandServiceName_)) {
  pathManagerCommandServiceEditor_->setText(pathManagerCommandServiceName_);
}

}

void PlanningPanel::updateInteractiveMarkerPose(const geometry_msgs::Pose& pose)
{
if (currently_editing_.empty()) {
return;
}
auto search = pose_widget_map_.find(currently_editing_);
if (search == pose_widget_map_.end()) {
return;
}
search->second->setPose(pose);
}

void PlanningPanel::widgetPoseUpdated(const std::string& id, geometry_msgs::Pose& pose)
{
if (currently_editing_ == id) {
interactive_markers_.setPose(pose);
}

interactive_markers_.updateMarkerPose(id, pose);
}

void PlanningPanel::pathManagerInfoCallback(const std_msgs::String& msg) {
  clearListWidget();
  addListWidgetItem(msg.data, "green");
}

void PlanningPanel::addListWidgetItem(std::string text, std::string color) {
  QListWidgetItem *item = new QListWidgetItem(QString::fromStdString(text));
  item->setBackgroundColor(QString::fromStdString(color));
  global_path_manager_info_list_->addItem(item);

}

void PlanningPanel::clearListWidget() {
  global_path_manager_info_list_->clear();

}

void PlanningPanel::callGlobalPlanningService()
{

  std::thread t([this] {

    std::string service_name = loadGlobalPathServiceName_.toStdString();
    spacebok_global_planner::GetGlobalPath::Request req;
    req.filePath = globalPathFileName_.toStdString();
    spacebok_global_planner::GetGlobalPath::Response res;

    callService(req,res,service_name);

  });

  t.detach();

}

void PlanningPanel::callPublishGlobalTrackingCommand()
{
  se2_navigation_msgs::ControllerCommand command;
  command.command_ = se2_navigation_msgs::ControllerCommand::Command::StartTracking;
  callSendPathManagerCommandService(command);
}

void PlanningPanel::callPublishGlobalStopTrackingCommand()
{
  se2_navigation_msgs::ControllerCommand command;
  command.command_ = se2_navigation_msgs::ControllerCommand::Command::StopTracking;
  callSendPathManagerCommandService(command);
}


void PlanningPanel::callPlanningService()
{

std::thread t([this] {

se2_navigation_msgs::PathRequestMsg pathRequest;

const bool useCurrentStateAsStartingPose = currentStateAsStartCheckBox_->isChecked();

if (useCurrentStateAsStartingPose) {
  getStartPoseFromService(&(pathRequest.startingPose));
  lastPose_ = pathRequest.startingPose;  //update last state
  pose_widget_map_["start"]->setPose(lastPose_);
  this->widgetPoseUpdated("start", lastPose_);
  finishEditing("start");
} else {
  getStartPoseFromWidget(&(pathRequest.startingPose));
}

goal_pose_widget_->getPose(&(pathRequest.goalPose));

std::string service_name = planningServiceName_.toStdString();
se2_navigation_msgs::RequestPathSrv::Request req;
req.pathRequest = pathRequest;
se2_navigation_msgs::RequestPathSrv::Response res;

callService(req,res,service_name);

});

t.detach();

}

void PlanningPanel::executeSpacebokStandbyInitMotion() const
{

  std::thread t([this] {
    double maxPositionOffset = 0.1;
    spacebok_msgs::SpacebokHighlevelCommands controlCommand;
    controlCommand.header.frame_id = "";
    //! Default values
    controlCommand.height = 0.1;
    controlCommand.velocity = 0;
    controlCommand.turning_rate = 0;
    //! Set standby pose
    controlCommand.position_offset = {maxPositionOffset, 0};
    controlCommand.quaternion.w = 1;
    controlCommand.quaternion.x = 0;
    controlCommand.quaternion.y = 0;
    controlCommand.quaternion.z = 0;

    //! Move back and forth in standby
    ros::Rate rate(0.5);
    int counter = 0;
    while(ros::ok() && counter < 4) {
      controlCommand.header.seq = counter;
      controlCommand.header.stamp = ros::Time::now();
      controlCommand.position_offset[0] = -controlCommand.position_offset[0];
      spacebokControlCommandPublisher_.publish(controlCommand);
      counter++;
      rate.sleep();
    }

    //! Neutral position in the end
    counter++;
    controlCommand.header.seq = counter;
    controlCommand.header.stamp = ros::Time::now();
    controlCommand.position_offset[0] = 0.0;
    spacebokControlCommandPublisher_.publish(controlCommand);
  });

  t.detach();
}

void PlanningPanel::startSpacebokWalkingInitMotion() const
{

  std::thread t([this] {
    callPublishSpacebokStartCommand();

    ros::Duration(0.5).sleep();

    double desiredVelocity = 0.1;  // [m/s]
    spacebok_msgs::SpacebokHighlevelCommands controlCommand;
    controlCommand.header.frame_id = "";
    //! Default values
    controlCommand.height = 0.1;
    controlCommand.velocity = desiredVelocity;
    controlCommand.turning_rate = 0;
    //! Set standby pose
    controlCommand.position_offset = {0, 0};
    controlCommand.quaternion.w = 1;
    controlCommand.quaternion.x = 0;
    controlCommand.quaternion.y = 0;
    controlCommand.quaternion.z = 0;

    //! Start walking
    controlCommand.header.stamp = ros::Time::now();
    spacebokControlCommandPublisher_.publish(controlCommand);
  });

  t.detach();
}

void PlanningPanel::stopSpacebokWalkingInitMotion() const
{

  std::thread t([this] {

    double desiredVelocity = 0.0;  // [m/s]
    spacebok_msgs::SpacebokHighlevelCommands controlCommand;
    controlCommand.header.frame_id = "";
    //! Default values
    controlCommand.height = 0.1;
    controlCommand.velocity = desiredVelocity;
    controlCommand.turning_rate = 0;
    //! Set standby pose
    controlCommand.position_offset = {0, 0};
    controlCommand.quaternion.w = 1;
    controlCommand.quaternion.x = 0;
    controlCommand.quaternion.y = 0;
    controlCommand.quaternion.z = 0;

    //! Start walking
    controlCommand.header.stamp = ros::Time::now();
    spacebokControlCommandPublisher_.publish(controlCommand);

    ros::Duration(0.5).sleep();

    callPublishSpacebokStopCommand();
  });

  t.detach();
}

void PlanningPanel::callResetOrientationEstimation() const
{
  std_srvs::Trigger::Request req;
  std_srvs::Trigger::Response res;
  callService(req, res, "/gps_orientation_estimation/reset");
}

void PlanningPanel::callInitElevationMappingCupy() const
{
  std_srvs::Empty::Request req;
  std_srvs::Empty::Response res;
  callService(req, res, "/elevation_mapping/clear_map_with_initializer");
}

void PlanningPanel::callResetElevationMappingCupy() const
{
  std_srvs::Empty::Request req;
  std_srvs::Empty::Response res;
  callService(req, res, "/elevation_mapping/clear_map");
}

void PlanningPanel::getStartPoseFromWidget(geometry_msgs::Pose *startPoint)
{
start_pose_widget_->getPose(startPoint);
}

void PlanningPanel::getStartPoseFromService(geometry_msgs::Pose *startPoint)
{

se2_navigation_msgs::RequestCurrentStateSrv::Request req;
se2_navigation_msgs::RequestCurrentStateSrv::Response res;

std::string service_name = currentStateServiceName_.toStdString();
callService(req, res, service_name);
*startPoint = res.pose;
}

void PlanningPanel::callPublishTrackingCommand()
{
se2_navigation_msgs::ControllerCommand command;
command.command_ = se2_navigation_msgs::ControllerCommand::Command::StartTracking;
callSendControllerCommandService(command);
}

void PlanningPanel::callPublishStopTrackingCommand()
{
se2_navigation_msgs::ControllerCommand command;
command.command_ = se2_navigation_msgs::ControllerCommand::Command::StopTracking;
callSendControllerCommandService(command);
}

void PlanningPanel::callSendControllerCommandService(
  se2_navigation_msgs::ControllerCommand &command) const
{

  se2_navigation_msgs::SendControllerCommandSrv::Request req;
  se2_navigation_msgs::SendControllerCommandSrv::Response res;

  req.command = se2_navigation_msgs::convert(command);
  std::string service_name = controllerCommandTopicName_.toStdString();
  callService(req, res, service_name);

}

void PlanningPanel::callSendPathManagerCommandService(
  se2_navigation_msgs::ControllerCommand &command) const
{

  se2_navigation_msgs::SendControllerCommandSrv::Request req;
  se2_navigation_msgs::SendControllerCommandSrv::Response res;

  req.command = se2_navigation_msgs::convert(command);
  std::string service_name = pathManagerCommandServiceName_.toStdString();
  callService(req, res, service_name);

}

void PlanningPanel::callPublishSpacebokStandUpCommand() const
{
spacebok_msgs::SpacebokHighlevelState highlevelState;
highlevelState.header.stamp = ros::Time::now();
highlevelState.state = spacebok_msgs::SpacebokHighlevelState::OPERATIONAL;
publishSpacebokHighlevelState(highlevelState);

spacebok_msgs::SpacebokControllerState controllerState;
controllerState.header.stamp = ros::Time::now();
controllerState.state = spacebok_msgs::SpacebokControllerState::STAND_UP;
publishSpacebokControllerState(controllerState);
}

void PlanningPanel::callPublishSpacebokStartCommand() const
{
  spacebok_msgs::SpacebokControllerState controllerState;
  controllerState.header.stamp = ros::Time::now();
  controllerState.state = spacebok_msgs::SpacebokControllerState::WALKING_TROT;
  publishSpacebokControllerState(controllerState);
}

void PlanningPanel::callPublishSpacebokStopCommand() const
{
  spacebok_msgs::SpacebokControllerState controllerState;
  controllerState.header.stamp = ros::Time::now();
  controllerState.state = spacebok_msgs::SpacebokControllerState::STANDBY;
  publishSpacebokControllerState(controllerState);
}

void PlanningPanel::publishSpacebokHighlevelState(
  spacebok_msgs::SpacebokHighlevelState &state) const
{
  spacebokHighlevelStatePublisher_.publish(state);
}

void PlanningPanel::publishSpacebokControllerState(
  spacebok_msgs::SpacebokControllerState &state) const
{
  spacebokControllerStatePublisher_.publish(state);
}

} /* namespace se2_planning_rviz */

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(se2_planning_rviz::PlanningPanel, rviz::Panel)
