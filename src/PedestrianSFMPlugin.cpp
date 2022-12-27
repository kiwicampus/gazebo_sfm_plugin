/***********************************************************************/
/**                                                                    */
/** PedestrianSFMPlugin.cpp                                            */
/**                                                                    */
/** Copyright (c) 2022, Service Robotics Lab (SRL).                    */
/**                     http://robotics.upo.es                         */
/**                                                                    */
/** All rights reserved.                                               */
/**                                                                    */
/** Authors:                                                           */
/** Noé Pérez-Higueras (maintainer)                                    */
/** email: noeperez@upo.es                                             */
/**                                                                    */
/** This software may be modified and distributed under the terms      */
/** of the BSD license. See the LICENSE file for details.              */
/**                                                                    */
/** http://www.opensource.org/licenses/BSD-3-Clause                    */
/**                                                                    */
/***********************************************************************/

#include <functional>
#include <stdio.h>  
#include <string>
#include <vector>


#include <gazebo_sfm_plugin/PedestrianSFMPlugin.h>


#include <chrono>
using namespace std;
using namespace std::chrono;


using namespace gazebo;
GZ_REGISTER_MODEL_PLUGIN(PedestrianSFMPlugin)
#define WALKING_ANIMATION "walking"


PedestrianSFMPlugin::PedestrianSFMPlugin() {
}


void PedestrianSFMPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {

  // Create sfmActor
  this->sdf = _sdf;
  this->actor = boost::dynamic_pointer_cast<physics::Actor>(_model);
  this->world = this->actor->GetWorld();
  this->sfmActor.id = this->actor->GetId();
  cout << "sfmActor ID: "<< this->sfmActor.id<< std::endl;


  // Initialize sfmActor position
  ignition::math::Vector3d pos = this->actor->WorldPose().Pos();
  ignition::math::Vector3d rpy = this->actor->WorldPose().Rot().Euler();
  this->sfmActor.position.set(pos.X(), pos.Y());
  this->sfmActor.yaw = utils::Angle::fromRadian(rpy.Z()); // yaw
  ignition::math::Vector3d linvel = this->actor->WorldLinearVel();
  this->sfmActor.velocity.set(linvel.X(), linvel.Y());
  this->sfmActor.linearVelocity = linvel.Length();
  ignition::math::Vector3d angvel = this->actor->WorldAngularVel();
  this->sfmActor.angularVelocity = angvel.Z(); // Length()


  this->ros_node_ = gazebo_ros::Node::Get();

  // Read delta time to publish path 
  if (_sdf->HasElement("publish_time"))
    this->publish_time = _sdf->Get<int>("publish_time");  
  else
    this->publish_time = 1000;

  // Create timer to calculate and publish paths
  this->publish_future_poses_timer_ = this->ros_node_->create_wall_timer(
            std::chrono::milliseconds(this->publish_time),
            std::bind(&PedestrianSFMPlugin::Calculate_path_timerCallback, this));


  // Path Publisher for each actor created
  this->PathPublisher_  = this->ros_node_->create_publisher<nav_msgs::msg::Path>("path_" + to_string(this->sfmActor.id), 10);


    // Read parameters to calculate future positions
  if (_sdf->HasElement("look_ahead_time"))
    this->look_ahead_time = _sdf->Get<double>("look_ahead_time");  
  else
    this->look_ahead_time = 5.0;
  
  if (_sdf->HasElement("dt_calculations"))
    this->dt_calculations = _sdf->Get<double>("dt_calculations");
  
  else
    this->dt_calculations = 0.2;
  

  // Calculate number of iterations for next calculated positions
  this->iterations = (int) this->look_ahead_time / this->dt_calculations;


  // Read in the maximum velocity of the pedestrian
  if (_sdf->HasElement("velocity"))
    this->sfmActor.desiredVelocity = _sdf->Get<double>("velocity");
  else
    this->sfmActor.desiredVelocity = 0.8;

  // Read in the target weight
  if (_sdf->HasElement("goal_weight"))
    this->sfmActor.params.forceFactorDesired = _sdf->Get<double>("goal_weight");
  // Read in the obstacle weight
  if (_sdf->HasElement("obstacle_weight"))
    this->sfmActor.params.forceFactorObstacle =
        _sdf->Get<double>("obstacle_weight");
  // Read in the social weight
  if (_sdf->HasElement("social_weight"))
    this->sfmActor.params.forceFactorSocial =
        _sdf->Get<double>("social_weight");
  // Read in the group gaze weight
  if (_sdf->HasElement("group_gaze_weight"))
    this->sfmActor.params.forceFactorGroupGaze =
        _sdf->Get<double>("group_gaze_weight");
  // Read in the group coherence weight
  if (_sdf->HasElement("group_coh_weight"))
    this->sfmActor.params.forceFactorGroupCoherence =
        _sdf->Get<double>("group_coh_weight");
  // Read in the group repulsion weight
  if (_sdf->HasElement("group_rep_weight"))
    this->sfmActor.params.forceFactorGroupRepulsion =
        _sdf->Get<double>("group_rep_weight");

  // Read in the animation factor (applied in the OnUpdate function).
  if (_sdf->HasElement("animation_factor"))
    this->animationFactor = _sdf->Get<double>("animation_factor");
  else
    this->animationFactor = 4.5;

  if (_sdf->HasElement("animation_name")) {
    this->animationName = _sdf->Get<std::string>("animation_name");
  } else
    this->animationName = WALKING_ANIMATION;

  if (_sdf->HasElement("people_distance"))
    this->peopleDistance = _sdf->Get<double>("people_distance");
  else
    this->peopleDistance = 5.0;

  // Read in the pedestrians in your walking group
  if (_sdf->HasElement("group")) {
    this->sfmActor.groupId = this->sfmActor.id;
    sdf::ElementPtr modelElem = _sdf->GetElement("group")->GetElement("model");
    while (modelElem) {
      this->groupNames.push_back(modelElem->Get<std::string>());
      modelElem = modelElem->GetNextElement("model");
    }
    this->sfmActor.groupId = this->sfmActor.id;
  } else
    this->sfmActor.groupId = -1;

  // Read in the other obstacles to ignore
  if (_sdf->HasElement("ignore_obstacles")) {
    sdf::ElementPtr modelElem =
        _sdf->GetElement("ignore_obstacles")->GetElement("model");
    while (modelElem) {
      this->ignoreModels.push_back(modelElem->Get<std::string>());
      modelElem = modelElem->GetNextElement("model");
    }
  }
  // Add our own name to models we should ignore when avoiding obstacles.
  this->ignoreModels.push_back(this->actor->GetName());
  // Add the other pedestrians to the ignored obstacles
  for (unsigned int i = 0; i < this->world->ModelCount(); ++i) {
    physics::ModelPtr model = this->world->ModelByIndex(i); // GetModel(i);

    if (model->GetId() != this->actor->GetId() &&
        ((int)model->GetType() == (int)this->actor->GetType())) {
      this->ignoreModels.push_back(model->GetName());
    }
  }

  this->connections.push_back(event::Events::ConnectWorldUpdateBegin(
      std::bind(&PedestrianSFMPlugin::OnUpdate, this, std::placeholders::_1)));

  this->Reset();

}

void PedestrianSFMPlugin::Calculate_path_timerCallback() {

  RCLCPP_INFO(this->ros_node_->get_logger(), "Hello from ROS2");

  // Copy sfmActor to calculate next positions without affecting model in Gazebo
  this->copy_sfmActor = this->sfmActor;

    // Create path message
  auto path = nav_msgs::msg::Path();
  path.header.frame_id = "base_link";
  path.header.stamp = rclcpp::Clock().now();        

  // Create pose message
  auto next_pose = geometry_msgs::msg::PoseStamped();


  // Calculate future positions
  for (int i = 0; i < this->iterations; i++) {

        // Compute Social Forces
        sfm::SFM.computeForces(this->copy_sfmActor, this->otherActors);
        // Update model
        sfm::SFM.updatePosition(this->copy_sfmActor, this->dt_calculations);

        // Add position in x, y and angle to pose
        next_pose.pose.position.x = this->copy_sfmActor.position.getX();
        next_pose.pose.position.y = this->copy_sfmActor.position.getY();
        // Quaternion calculation. Roll and pitch are zero. 
        next_pose.pose.orientation.z = sin(this->copy_sfmActor.yaw.toRadian()/2);
        next_pose.pose.orientation.w = cos(this->copy_sfmActor.yaw.toRadian()/2); 

        // Append pose to path message
        path.poses.push_back(next_pose);
  }
  this->PathPublisher_->publish(path);    
}

void PedestrianSFMPlugin::Reset() {

  this->lastUpdate = 0;

  // Read in the goals to reach
  if (this->sdf->HasElement("trajectory")) {
    sdf::ElementPtr modelElemCyclic =
        this->sdf->GetElement("trajectory")->GetElement("cyclic");

    if (modelElemCyclic)
      this->sfmActor.cyclicGoals = modelElemCyclic->Get<bool>();

    sdf::ElementPtr modelElem =
        this->sdf->GetElement("trajectory")->GetElement("waypoint");
    while (modelElem) {
      ignition::math::Vector3d g = modelElem->Get<ignition::math::Vector3d>();
      sfm::Goal goal;
      goal.center.set(g.X(), g.Y());
      goal.radius = 0.3;
      this->sfmActor.goals.push_back(goal);
      modelElem = modelElem->GetNextElement("waypoint");
    }
  }

  auto skelAnims = this->actor->SkeletonAnimations();
  if (skelAnims.find(this->animationName) == skelAnims.end()) {
    gzerr << "Skeleton animation " << this->animationName << " not found.\n";
  } else {
    // Create custom trajectory
    this->trajectoryInfo.reset(new physics::TrajectoryInfo());
    this->trajectoryInfo->type = this->animationName;
    this->trajectoryInfo->duration = 1.0;

    this->actor->SetCustomTrajectory(this->trajectoryInfo);
  }
}


void PedestrianSFMPlugin::HandleObstacles() {
  double minDist = 10000.0;
  ignition::math::Vector3d closest_obs;
  ignition::math::Vector3d closest_obs2;
  this->sfmActor.obstacles1.clear();

  for (unsigned int i = 0; i < this->world->ModelCount(); ++i) {
    physics::ModelPtr model = this->world->ModelByIndex(i); // GetModel(i);
    if (std::find(this->ignoreModels.begin(), this->ignoreModels.end(),
                  model->GetName()) == this->ignoreModels.end()) {
      ignition::math::Vector3d actorPos = this->actor->WorldPose().Pos(); 
      ignition::math::Vector3d modelPos = model->WorldPose().Pos();
      std::tuple<bool, double, ignition::math::Vector3d> intersect =
          model->BoundingBox().Intersect(modelPos, actorPos, 0.05, 8.0);

      if (std::get<0>(intersect) == true) {

        ignition::math::Vector3d offset = std::get<2>(intersect) - actorPos;
        double modelDist = offset.Length(); 

        if (modelDist < minDist) {
          minDist = modelDist;
          closest_obs = std::get<2>(intersect);
        }
      }
    }
  }
  if (minDist <= 10.0) {
    utils::Vector2d ob(closest_obs.X(), closest_obs.Y());
    this->sfmActor.obstacles1.push_back(ob);
  }
}

void PedestrianSFMPlugin::HandlePedestrians() {
  this->otherActors.clear();

  for (unsigned int i = 0; i < this->world->ModelCount(); ++i) {
    physics::ModelPtr model = this->world->ModelByIndex(i); // GetModel(i);

    if (model->GetId() != this->actor->GetId() &&
        ((int)model->GetType() == (int)this->actor->GetType())) {


      ignition::math::Pose3d modelPose = model->WorldPose();
      ignition::math::Vector3d pos =
          modelPose.Pos() - this->actor->WorldPose().Pos();
      if (pos.Length() < this->peopleDistance) {
        sfm::Agent ped;
        ped.id = model->GetId();
        ped.position.set(modelPose.Pos().X(), modelPose.Pos().Y());
        ignition::math::Vector3d rpy = modelPose.Rot().Euler();
        ped.yaw = utils::Angle::fromRadian(rpy.Z());

        ped.radius = this->sfmActor.radius;
        ignition::math::Vector3d linvel = model->WorldLinearVel();
        ped.velocity.set(linvel.X(), linvel.Y());
        ped.linearVelocity = linvel.Length();
        ignition::math::Vector3d angvel = model->WorldAngularVel();
        ped.angularVelocity = angvel.Z(); // Length()

        // check if the ped belongs to my group
        if (this->sfmActor.groupId != -1) {
          std::vector<std::string>::iterator it;
          it = find(groupNames.begin(), groupNames.end(), model->GetName());
          if (it != groupNames.end())
            ped.groupId = this->sfmActor.groupId;
          else
            ped.groupId = -1;
        }
        this->otherActors.push_back(ped);
      }
    }
  }
}


void PedestrianSFMPlugin::OnUpdate(const common::UpdateInfo &_info) {

  double dt = (_info.simTime - this->lastUpdate).Double();

  ignition::math::Pose3d actorPose = this->actor->WorldPose(); 
  
  HandleObstacles();

  // update pedestrian around
  HandlePedestrians();

  // Compute Social Forces
  sfm::SFM.computeForces(this->sfmActor, this->otherActors);

  // Update model
  sfm::SFM.updatePosition(this->sfmActor, dt);

  utils::Angle h = this->sfmActor.yaw;
  utils::Angle add = utils::Angle::fromRadian(1.5707);
  h = h + add;
  double yaw = h.toRadian();

  ignition::math::Vector3d rpy = actorPose.Rot().Euler();
  utils::Angle current = utils::Angle::fromRadian(rpy.Z());
  double diff = (h - current).toRadian();
  if (std::fabs(diff) > IGN_DTOR(10)) {
    current = current + utils::Angle::fromRadian(diff * 0.005);
    yaw = current.toRadian();
  }

  actorPose.Pos().X(this->sfmActor.position.getX()); 
  actorPose.Pos().Y(this->sfmActor.position.getY());
  actorPose.Rot() =
      ignition::math::Quaterniond(1.5707, 0, yaw); 

  actorPose.Pos().Z(1.2138);

  // Distance traveled is used to coordinate motion with the walking
  // animation
  double distanceTraveled =
      (actorPose.Pos() - this->actor->WorldPose().Pos()).Length();

  this->actor->SetWorldPose(actorPose, false, false);
  this->actor->SetScriptTime(this->actor->ScriptTime() +
                             (distanceTraveled * this->animationFactor));
  this->lastUpdate = _info.simTime;
}