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

//#include <ignition/math.hh>
//#include <ignition/math/gzmath.hh>
#include <gazebo_sfm_plugin/PedestrianSFMPlugin.h>


#include <chrono>
using namespace std;
using namespace std::chrono;

// Use auto keyword to avoid typing long
// type definitions to get the timepoint
// at this instant use function now()


using namespace gazebo;
GZ_REGISTER_MODEL_PLUGIN(PedestrianSFMPlugin)
#define WALKING_ANIMATION "walking"

/////////////////////////////////////////////////
PedestrianSFMPlugin::PedestrianSFMPlugin() {
}

/////////////////////////////////////////////////
void PedestrianSFMPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {

  this->ros_node_ = gazebo_ros::Node::Get();

  this->timer_ = this->ros_node_->create_wall_timer(
            std::chrono::milliseconds(1000),
            std::bind(&PedestrianSFMPlugin::timerCallback, this));

  // Calculate number of iterations for next calculated positions
  this->iterations = (int) this->look_ahead_time / this->dt_calculations;

  // Create sfmActor
  this->sdf = _sdf;
  this->actor = boost::dynamic_pointer_cast<physics::Actor>(_model);
  this->world = this->actor->GetWorld();
  this->sfmActor.id = this->actor->GetId();
  cout << "sfmActor ID: "<< this->sfmActor.id<< std::endl;


  // Add publisher to the map with sfmActor.id as key
  this->PathPublisherMap.insert({this->sfmActor.id, this->ros_node_->create_publisher<nav_msgs::msg::Path>("path_" + to_string(this->sfmActor.id), 10)});
  
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

    // Save initial values of sfmActor
  utils::Vector2d initPos = sfmActor.position;
  utils::Angle initYaw = sfmActor.yaw;
  utils::Vector2d real_velocity = sfmActor.velocity;
  double real_linearVelocity = sfmActor.linearVelocity;
  double real_angularVelocity = sfmActor.angularVelocity;
  utils::Vector2d real_movement = sfmActor.movement;
  std::list<sfm::Goal> real_goals = sfmActor.goals;


    // Clear content of temporary vectors
  this->temp_next_positionsX.clear();
  this->temp_next_positionsY.clear();
  this->temp_next_yaw_angles.clear();

  // Calculate first future positions
  for (int i = 0; i < this->iterations; i++) {

        // These functions use the real actor model pose. We do not modify the real actor model pose
        // HandleObstacles();
        // HandlePedestrians();
        
        // Compute Social Forces
        sfm::SFM.computeForces(this->sfmActor, this->otherActors);
        // Update model
        sfm::SFM.updatePosition(this->sfmActor, this->dt_calculations);


        // Add position in x, y and angle yaw to the temporary vectors
        this->temp_next_positionsX.push_back(this->sfmActor.position.getX());
        this->temp_next_positionsY.push_back(this->sfmActor.position.getY());
        this->temp_next_yaw_angles.push_back(this->sfmActor.yaw.toRadian());
  }

  this->lastPos = sfmActor.position;
  this->lastYaw = sfmActor.yaw;
  this->lastVelocity = sfmActor.velocity;
  this->lastLinearVelocity = sfmActor.linearVelocity;
  this->lastAngularVelocity = sfmActor.angularVelocity;
  this->lastMovement = sfmActor.movement;
  this->lastGoals = sfmActor.goals;

  //Copy content of temporary vectors to the real vectors
  this->next_positionsX = this->temp_next_positionsX;
  this->next_positionsY = this->temp_next_positionsY;
  this->next_yaw_angles = this->temp_next_yaw_angles;

    // Reset initial values 
  this->sfmActor.position = initPos;
  this->sfmActor.yaw = initYaw;
  this->sfmActor.velocity = real_velocity;
  this->sfmActor.linearVelocity = real_linearVelocity;
  this->sfmActor.angularVelocity = real_angularVelocity;
  this->sfmActor.movement = real_movement;
  this->sfmActor.goals = real_goals;

}


void PedestrianSFMPlugin::timerCallback() {

        RCLCPP_INFO(this->ros_node_->get_logger(), "Helloooo from ROS2");
        for (float y : this->next_positionsY){
          cout << "Element in y = " << y << std::endl;
        }
        cout << "Iterations: " << this->iterations << std::endl;
        // Create path message
        auto path = nav_msgs::msg::Path();
        path.header.frame_id = "base_link";
        path.header.stamp = rclcpp::Clock().now();        

        // Create pose message
        auto next_pose = geometry_msgs::msg::PoseStamped();

        // Fill position and orientation values in pose message
        for (int i = 0; i < (int)this->next_positionsX.size(); i++) {
            next_pose.pose.position.x = this->next_positionsX[i];
            next_pose.pose.position.y = this->next_positionsY[i];
            double orientation_z = sin(this->next_yaw_angles[i]/2);
            double orientation_w = cos(this->next_yaw_angles[i]/2); 
            next_pose.pose.orientation.z = orientation_z;
            next_pose.pose.orientation.w = orientation_w; 

            // Append pose to path message
            path.poses.push_back(next_pose);
        }

        // Choose the publisher according to the sfmActor.id and publish to corresponding topic
        this->PathPublisherMap.find(this->sfmActor.id)->second->publish(path);
        
    }

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

  for (unsigned int i = 0; i < this->world->ModelCount(); ++i) {
    physics::ModelPtr model = this->world->ModelByIndex(i); // GetModel(i);
    if (std::find(this->ignoreModels.begin(), this->ignoreModels.end(),
                  model->GetName()) == this->ignoreModels.end()) {
      ignition::math::Vector3d actorPos = this->actor->WorldPose().Pos(); //////////// We use the current actor pose
      ignition::math::Vector3d modelPos = model->WorldPose().Pos();
      std::tuple<bool, double, ignition::math::Vector3d> intersect =
          model->BoundingBox().Intersect(modelPos, actorPos, 0.05, 8.0);

      if (std::get<0>(intersect) == true) {

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
  if (minDist <= 10.0) {
    utils::Vector2d ob(closest_obs.X(), closest_obs.Y());
    this->sfmActor.obstacles1.push_back(ob);
  }
}

/////////////////////////////////////////////////
void PedestrianSFMPlugin::HandlePedestrians() {
  this->otherActors.clear();

  for (unsigned int i = 0; i < this->world->ModelCount(); ++i) {
    physics::ModelPtr model = this->world->ModelByIndex(i); // GetModel(i);

    if (model->GetId() != this->actor->GetId() &&
        ((int)model->GetType() == (int)this->actor->GetType())) {
      // printf("Actor %i has detected actor %i!\n", this->actor->GetId(),
      //        model->GetId());

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

  double dt = (_info.simTime - this->lastUpdate).Double();

  this->dt_counter += dt;

  ignition::math::Pose3d actorPose = this->actor->WorldPose(); ////// Actor Pose

  // Save current values of sfmActor
  utils::Vector2d initPos = sfmActor.position;
  utils::Angle initYaw = sfmActor.yaw;
  utils::Vector2d real_velocity = sfmActor.velocity;
  double real_linearVelocity = sfmActor.linearVelocity;
  double real_angularVelocity = sfmActor.angularVelocity;
  utils::Vector2d real_movement = sfmActor.movement;
  std::list<sfm::Goal> real_goals = sfmActor.goals;


  if(this->dt_counter >= this->dt_calculations){

      cout<< "Pum. Llegamos. Calcular futuro siguiente. dt_counter is: " << this->dt_counter<< std::endl;
      this->dt_counter = 0;
        // Set last calculated state parameters of sfmActor 
      this->sfmActor.position = this->lastPos;
      this->sfmActor.yaw = this->lastYaw;
      this->sfmActor.velocity = this->lastVelocity;
      this->sfmActor.linearVelocity = this->lastLinearVelocity;
      this->sfmActor.angularVelocity = this->lastAngularVelocity;
      this->sfmActor.movement = this->lastMovement;
      this->sfmActor.goals = this->lastGoals;

        // Compute Social Forces
      sfm::SFM.computeForces(this->sfmActor, this->otherActors);
        // Update model
      sfm::SFM.updatePosition(this->sfmActor, this->dt_calculations);


        // Add new calculated position in x, y and angle yaw to the temporary vectors
      this->temp_next_positionsX.push_back(this->sfmActor.position.getX());
      this->temp_next_positionsY.push_back(this->sfmActor.position.getY());
      this->temp_next_yaw_angles.push_back(this->sfmActor.yaw.toRadian());

        // Remove first element of the temporary vectors
      this->temp_next_positionsX.erase(this->temp_next_positionsX.begin());
      this->temp_next_positionsY.erase(this->temp_next_positionsY.begin());
      this->temp_next_yaw_angles.erase(this->temp_next_yaw_angles.begin());

        //Copy content of temporary vectors to the real vectors
      this->next_positionsX = this->temp_next_positionsX;
      this->next_positionsY = this->temp_next_positionsY;
      this->next_yaw_angles = this->temp_next_yaw_angles;

      this->lastPos = sfmActor.position;
      this->lastYaw = sfmActor.yaw;
      this->lastVelocity = sfmActor.velocity;
      this->lastLinearVelocity = sfmActor.linearVelocity;
      this->lastAngularVelocity = sfmActor.angularVelocity;
      this->lastMovement = sfmActor.movement;
      this->lastGoals = sfmActor.goals;


  }
        
  // Reset current values 
  this->sfmActor.position = initPos;
  this->sfmActor.yaw = initYaw;
  this->sfmActor.velocity = real_velocity;
  this->sfmActor.linearVelocity = real_linearVelocity;
  this->sfmActor.angularVelocity = real_angularVelocity;
  this->sfmActor.movement = real_movement;
  this->sfmActor.goals = real_goals;
  
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


  actorPose.Pos().X(this->sfmActor.position.getX()); /////////We update the model after doing the prediction. That's why we do not predict obstacles
  actorPose.Pos().Y(this->sfmActor.position.getY());
  actorPose.Rot() =
      ignition::math::Quaterniond(1.5707, 0, yaw); // rpy.Z()+yaw.Radian());

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



