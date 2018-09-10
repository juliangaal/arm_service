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

#include <ros/ros.h>
#include <arm_service/ArmInstruction.h>
#include <arm_service/arm_service.h>

using namespace arm_service;

ArmService::ArmService(std::string name) {
  service_ = nh_.advertiseService(name, &ArmService::processGoal, this);
  ROS_INFO("Ready to receive Arm Instructions");
}

const jaco_manipulation::BoundingBox ArmService::createGraspBoundingBox(const anchor_msgs::Anchor &anchor) const {
  jaco_manipulation::BoundingBox box;

  box.header.frame_id = "base_link";
  box.description = anchor.id;
  box.point = anchor.position.data.pose.position;
  box.point.x += anchor.shape.data.x * 0.5; // correction: centroid is infront of bounding box from kinect
  box.dimensions = anchor.shape.data;
  return box;
}

const jaco_manipulation::BoundingBox ArmService::createDropBoundingBox(const geometry_msgs::Point &point) const {
  jaco_manipulation::BoundingBox box;
  const auto &anchor = current_anchor_;

  box.header.frame_id = "base_link";
  box.description = anchor.id;
  box.point = point;
  box.point.z += anchor.shape.data.x * 0.5;
  box.dimensions = anchor.shape.data;
  return box;
}

bool ArmService::processGoal(arm_service::ArmInstruction::Request &req, arm_service::ArmInstruction::Response &res) {
  auto anchor = req.goal_anchor;
  auto instruction = req.instruction;
  auto target_location = req.target_location;
  auto description = anchor.caffe.symbols[0];

  if (instruction == "pick") {
    const jaco_manipulation::BoundingBox box = createGraspBoundingBox(anchor);
    current_anchor_ = anchor;
    jmc_.graspAt(box, description);
  } else {
    const jaco_manipulation::BoundingBox box = createDropBoundingBox(target_location);
    jmc_.dropAt(box, description);
  }

  return true;
}
