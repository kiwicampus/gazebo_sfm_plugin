/***********************************************************************/
/**                                                                    */
/** PedestrianSFMPlugin.cpp                                            */
/**                                                                    */
/** Copyright (c) 2021, Service Robotics Lab (SRL).                    */
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

//#include <ignition/math.hh>
//#include <ignition/math/gzmath.hh>
#include <gazebo_sfm_plugin/PedestrianSFMPlugin.h>


#include <chrono>
using namespace std;
using namespace std::chrono;

using namespace gazebo;
GZ_REGISTER_MODEL_PLUGIN(PedestrianSFMPlugin)

#define WALKING_ANIMATION "walking"

/////////////////////////////////////////////////
PedestrianSFMPlugin::PedestrianSFMPlugin() {}

/////////////////////////////////////////////////
void PedestrianSFMPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
  this->sdf = _sdf;
  this->actor = boost::dynamic_pointer_cast<physics::Actor>(_model);
  this->world = this->actor->GetWorld();


  this->sfmActor.id = this->actor->GetId();

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
  this->PathPublisher_  = this->ros_node_->create_publisher<nav_msgs::msg::Path>("path_" 
    + to_string(this->sfmActor.id), 10); 


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

  if (_sdf->HasElement("see_obstacles")){
    // Get first model of see obstacles from xml file
    sdf::ElementPtr modelElem =
        _sdf->GetElement("see_obstacles")->GetElement("model");
    while (modelElem) {

      // Get name of obstacle to see
      std::string obstacle_name = modelElem->Get<std::string>();

      // Check if it is a wildcard
      bool found_wildcard = obstacle_name.find("*") != std::string::npos;
      if(found_wildcard){
        RCLCPP_INFO_STREAM(this->ros_node_->get_logger(), "wildcard *: " << obstacle_name);
        uint end = obstacle_name.find("*");

        // Get index of models that begin with the name of the wildcard obstacle
        for (unsigned int i = 0; i < this->world->ModelCount(); ++i) {
            physics::ModelPtr model = this->world->ModelByIndex(i); // GetModel(i);
        
            //  if model begins with wildcard obstacle name, add index to see obstacle indexes vector
            if (model->GetName().rfind(obstacle_name.substr(0, end), 0) == 0) { 
              RCLCPP_INFO_STREAM(this->ros_node_->get_logger(), "Include obstacle from wildcard: " << model->GetName());
              RCLCPP_INFO_STREAM(this->ros_node_->get_logger(), "Index: " << i);
              this->see_obstacles_indexes.push_back(i);
            }
        }
      }
      else{ // in case it is not a wildcard

        // Get index of model with the name of the obstacle
        for (unsigned int i = 0; i < this->world->ModelCount(); ++i) {
            physics::ModelPtr model = this->world->ModelByIndex(i); // GetModel(i);
        
            // Add index to see obstacle indexes vector
            if (model->GetName().rfind(obstacle_name, 0) == 0) { 
              RCLCPP_INFO_STREAM(this->ros_node_->get_logger(), "Just include obstacle: " << model->GetName());
              RCLCPP_INFO_STREAM(this->ros_node_->get_logger(), "Index: " << i);
              this->see_obstacles_indexes.push_back(i);
            }
        }
      }

      // Get next model in the list of see obstacles
      modelElem = modelElem->GetNextElement("model");
    }

  }
  // Show all the obstacles to see for this instance
  for(size_t i = 0; i < this->see_obstacles_indexes.size(); i++){
    RCLCPP_INFO_STREAM(this->ros_node_->get_logger(), "Indexes obstacles to see: " << this->see_obstacles_indexes[i]);
  }

  this->connections.push_back(event::Events::ConnectWorldUpdateBegin(
      std::bind(&PedestrianSFMPlugin::OnUpdate, this, std::placeholders::_1)));

  this->Reset();


}

///////////////////////////////////////////////// 
void PedestrianSFMPlugin::Calculate_path_timerCallback() {

    // Copy sfmActor to calculate next positions without affecting model in Gazebo
  this->copy_sfmActor = this->sfmActor;
  this->copy_sfmActor.groupId = -1;

    // Create path message
  auto path = nav_msgs::msg::Path();
  path.header.frame_id = "base_link";
  path.header.stamp = rclcpp::Clock().now();        

  // Create pose message
  auto next_pose = geometry_msgs::msg::PoseStamped();

  // // Calculate future positions
  for (int i = 0; i < this->iterations; i++) {

        // // Compute Social Forces
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

/////////////////////////////////////////////////
void PedestrianSFMPlugin::Reset() {
  // this->velocity = 0.8;
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

/////////////////////////////////////////////////
void PedestrianSFMPlugin::HandleObstacles() {
  double minDist = 10000.0;
  ignition::math::Vector3d closest_obs;
  ignition::math::Vector3d closest_obs2;
  this->sfmActor.obstacles1.clear();

  for(size_t i = 0; i < this->see_obstacles_indexes.size(); i++){
      physics::ModelPtr model = this->world->ModelByIndex(this->see_obstacles_indexes[i]); 
      // Verify that obstacle is not a pedestrian
      if(((int)model->GetType() != (int)this->actor->GetType())){
      // RCLCPP_INFO_STREAM(this->ros_node_->get_logger(), "Obstacle to see: " << model->GetName());
      ignition::math::Vector3d actorPos = this->actor->WorldPose().Pos();
      ignition::math::Vector3d modelPos = model->WorldPose().Pos();
      std::tuple<bool, double, ignition::math::Vector3d> intersect =
          model->BoundingBox().Intersect(modelPos, actorPos, 0.05, 8.0);

      if (std::get<0>(intersect) == true) {

        // ignition::math::Vector3d = model->BoundingBox().Center();
        // double approximated_radius = std::max(model->BoundingBox().XLength(),
        //                                      model->BoundingBox().YLength());

        // ignition::math::Vector3d offset1 = modelPos - actorPos;
        // double modelDist1 = offset1.Length();
        // double dist1 = actorPos.Distance(modelPos);

        ignition::math::Vector3d offset = std::get<2>(intersect) - actorPos;
        double modelDist = offset.Length(); // - approximated_radius;
        // double dist2 = actorPos.Distance(std::get<2>(intersect));

        if (modelDist < minDist) {
          minDist = modelDist;
          // closest_obs = offset;
          closest_obs = std::get<2>(intersect);
        }
      }
    }
  }
      


      // printf("Actor %s x: %.2f y: %.2f\n", this->actor->GetName().c_str(),
      //        this->actor->WorldPose().Pos().X(),
      //        this->actor->WorldPose().Pos().Y());
      // printf("Model offset x: %.2f y: %.2f\n", closest_obs.X(), closest_obs.Y());
      // printf("Model intersec x: %.2f y: %.2f\n\n", closest_obs2.X(),
      //        closest_obs2.Y());
  if (minDist <= 10.0) {
    utils::Vector2d ob(closest_obs.X(), closest_obs.Y());
    this->sfmActor.obstacles1.push_back(ob);
  }
}

/////////////////////////////////////////////////
void PedestrianSFMPlugin::HandlePedestrians() {
  this->otherActors.clear();

  for (size_t i = 0; i < this->see_obstacles_indexes.size(); i++) {
    physics::ModelPtr model = this->world->ModelByIndex(this->see_obstacles_indexes[i]); // GetModel(i);
    // Verify that obstacle is another pedestrian
    if (model->GetId() != this->actor->GetId() &&
        ((int)model->GetType() == (int)this->actor->GetType())) {
      // printf("Actor %i has detected actor %i!\n", this->actor->GetId(),
      // model->GetId());
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
  // printf("Actor %s has detected %i actors!\n",
  // this->actor->GetName().c_str(),
  //        (int)this->otherActors.size());
}


/////////////////////////////////////////////////
void PedestrianSFMPlugin::OnUpdate(const common::UpdateInfo &_info) {
  // Time delta
  double dt = (_info.simTime - this->lastUpdate).Double();

  ignition::math::Pose3d actorPose = this->actor->WorldPose();

  // update closest obstacle
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
  // double yaw = this->sfmActor.yaw.toRadian();
  // Rotate in place, instead of jumping.
  // if (std::abs(yaw.Radian()) > IGN_DTOR(10))
  //{
  //  ActorPose.Rot() = ignition::math::Quaterniond(1.5707, 0, rpy.Z()+
  //      yaw.Radian()*0.001);
  //}
  // else
  //{
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
      ignition::math::Quaterniond(1.5707, 0, yaw); // rpy.Z()+yaw.Radian());
  //}

  // Make sure the actor stays within bounds
  // actorPose.Pos().X(std::max(-3.0, std::min(3.5, actorPose.Pos().X())));
  // actorPose.Pos().Y(std::max(-10.0, std::min(2.0, actorPose.Pos().Y())));
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