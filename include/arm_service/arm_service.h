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

#include <ros/ros.h>
#include <anchor_msgs/Anchor.h>
#include <geometry_msgs/Point.h>
#include <arm_service/ArmInstruction.h>
#include <jaco_manipulation/client/jaco_manipulation_client.h>
#include <memory>

namespace arm_service {

class ArmService {
 public:
  ArmService();
  ~ArmService() = default;

 private:
  jaco_manipulation::client::JacoManipulationClient jmc_;
  ros::NodeHandle nh_;
  anchor_msgs::Anchor current_anchor_;
  ros::ServiceServer service_;

  const jaco_manipulation::BoundingBox createGraspBoundingBox(const anchor_msgs::Anchor &anchor);

  const jaco_manipulation::BoundingBox createDropBoundingBox(const geometry_msgs::Point &point);

  bool processGoal(arm_service::ArmInstruction::Request &req, arm_service::ArmInstruction::Response &res);
};
} // namespace arm_service

#endif //PROJECT_ARM_SERVICE_H
