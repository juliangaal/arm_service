/*
  Copyright (C) 2018  Julian Gaal
  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.
  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.
  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>
*/

#ifndef PROJECT_ARM_SERVICE_H
#define PROJECT_ARM_SERVICE_H

#include <ros/node_handle.h>
#include <anchor_msgs/Anchor.h>
#include <geometry_msgs/Point.h>
#include <arm_service/ArmInstruction.h>
#include <jaco_manipulation/client/jaco_manipulation_client.h>

namespace arm_service {

/**
 * Used to communicate with Jaco Robotic Arm with ReGround Syntax
 */
class ArmService {
 public:
  /**
   * Constructor
   */
  ArmService();

  /**
   * Destructor
   */
  virtual ~ArmService() = default;

 protected:

  /**
  * Creates Bounding Box for grasping from anchor msg
  * @param anchor to be grasped
  * @return jaco_manipulation::BoundingBox
  */
  const jaco_manipulation::BoundingBox createGraspBoundingBox(const anchor_msgs::Anchor &anchor) const;

 private:

  /**
   * Jaco Manipulation client
   */
  jaco_manipulation::client::JacoManipulationClient jmc_;

  /**
   * ROS Nodehandler
   */
  ros::NodeHandle nh_;

  /**
   * Current anchor: saved for dropping
   */
  anchor_msgs::Anchor current_anchor_;

  /**
   * ROS Service, see ../srv/ArmInstruction.srv
   */
  ros::ServiceServer service_;

  /**
   * Creates Bounding Box for drop
   * @param point to be dropped at
   * @return jaco_manipulation::BoundingBox
   */
  const jaco_manipulation::BoundingBox createDropBoundingBox(const geometry_msgs::Point &point) const;

  /**
   * Processes goal received by ArmInstruction service
   * @param req ArmInstruction request
   * @param res ArmInstrcuction response
   * @return
   */
  bool processGoal(arm_service::ArmInstruction::Request &req, arm_service::ArmInstruction::Response &res);
};
} // namespace arm_service

#endif //PROJECT_ARM_SERVICE_H
