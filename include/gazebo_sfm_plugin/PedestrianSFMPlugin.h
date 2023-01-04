/***********************************************************************/
/**                                                                    */
/** PedestrianSFMPlugin.h                                              */
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

#ifndef GAZEBO_PLUGINS_PEDESTRIANSFMPLUGIN_HH_
#define GAZEBO_PLUGINS_PEDESTRIANSFMPLUGIN_HH_

// C++
#include <algorithm>
#include <string>
#include <map>

// Gazebo
#include "gazebo/common/Plugin.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/util/system.hh"

// Social Force Model
#include <lightsfm/sfm.hpp>

// ROS
#include "rclcpp/rclcpp.hpp"
#include <gazebo_ros/node.hpp>
#include <nav_msgs/msg/path.hpp>

namespace gazebo {
class GZ_PLUGIN_VISIBLE PedestrianSFMPlugin : public ModelPlugin {
  /// \brief Constructor
public:
  PedestrianSFMPlugin();

  /// \brief Load the actor plugin.
  /// \param[in] _model Pointer to the parent model.
  /// \param[in] _sdf Pointer to the plugin's SDF elements.
public:
  virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

  // Documentation Inherited.
public:
  virtual void Reset();

  /// \brief Function that is called every update cycle.
  /// \param[in] _info Timing information
private:
  void OnUpdate(const common::UpdateInfo &_info);

  // private: void InitializePedestrians();

  /// \brief Helper function to detect the closest obstacles.
private:
  void HandleObstacles();

  /// \brief Helper function to detect the nearby pedestrians (other actors).
private:
  void HandlePedestrians();

  /// \brief ROS Timer
private:
  void Calculate_path_timerCallback();
  //-------------------------------------------------

  // Time to calculate future positions
private:
  double look_ahead_time;

private:
  double prediction_time_step;

  /// \brief iterations to calculate first next positions
private:
  int iterations;

private:
  gazebo_ros::Node::SharedPtr ros_node_;

private:
  rclcpp::TimerBase::SharedPtr publish_future_poses_timer_;

  // rate in Hz to publish path topic
private:
  int publish_rate;

private:
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr PathPublisher_;

  /// \brief List of obstacles to see
private:
  std::vector<int> see_obstacles_indexes;

  /// \brief this actor as a SFM agent
private:
  sfm::Agent copy_sfmActor;

  /// \brief this actor as a SFM agent
private:
  sfm::Agent sfmActor;

  /// \brief names of the other models in my walking group.
private:
  std::vector<std::string> groupNames;

  /// \brief vector of pedestrians detected.
private:
  std::vector<sfm::Agent> otherActors;

  /// \brief Maximum distance to detect nearby pedestrians.
private:
  double peopleDistance;

  /// \brief Pointer to the parent actor.
private:
  physics::ActorPtr actor;

  /// \brief Pointer to the world, for convenience.
private:
  physics::WorldPtr world;

  /// \brief Pointer to the sdf element.
private:
  sdf::ElementPtr sdf;

  /// \brief Velocity of the actor
private:
  ignition::math::Vector3d velocity;

  /// \brief List of connections
private:
  std::vector<event::ConnectionPtr> connections;

  /// \brief Time scaling factor. Used to coordinate translational motion
  /// with the actor's walking animation.
private:
  double animationFactor = 1.0;

  /// \brief Time of the last update.
private:
  common::Time lastUpdate;

  /// \brief List of models to ignore. Used for vector field
private:
  std::vector<std::string> ignoreModels;

  /// \brief Animation name of this actor
private:
  std::string animationName;

  /// \brief Custom trajectory info.
private:
  physics::TrajectoryInfoPtr trajectoryInfo;
};
} // namespace gazebo
#endif
