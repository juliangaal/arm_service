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
#include <jaco_manipulation/client/jaco_manipulation_client.h>

const jaco_manipulation::BoundingBox createBoundingBox(const anchor_msgs::Anchor &anchor) {
  jaco_manipulation::BoundingBox box;
//  box.header.stamp = ros::Time::now();
  box.header.frame_id = "base_link";
  box.description = anchor.caffe.symbols[0];
  box.point = anchor.position.data.pose.position;
  box.point.x += anchor.shape.data.x * 0.5;
  box.dimensions.x = 0.06;
  box.dimensions.y = 0.06;
  box.dimensions.z = anchor.shape.data.z;

  return box;
}

bool processGoal(arm_service::ArmInstruction::Request &req,
                 arm_service::ArmInstruction::Response &res) {

  jaco_manipulation::client::JacoManipulationClient jmc;
  auto anchor = req.goal_anchor;
  auto instruction = req.instruction;
//  res.status = true;
  const jaco_manipulation::BoundingBox box = createBoundingBox(anchor);
  jmc.graspAt(box);
  return true;
}



int main(int argc, char **argv) {
  ros::init(argc, argv, "arm_service_server");
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("arm_service", processGoal);
  ROS_INFO("Ready to add two ints.");
  ros::spin();

  return 0;
}