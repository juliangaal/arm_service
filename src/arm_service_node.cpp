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

jaco_manipulation::BoundingBox current_box;

const jaco_manipulation::BoundingBox createGraspBoundingBox(const anchor_msgs::Anchor &anchor) {
  jaco_manipulation::BoundingBox box;
//  box.header.stamp = ros::Time::now();
  box.header.frame_id = "base_link";
  box.description = anchor.caffe.symbols[0];
  box.point = anchor.position.data.pose.position;
  box.point.x += anchor.shape.data.x * 0.5; // correction: centroid is infront of bounding box from kinect
  box.dimensions = anchor.shape.data;
  current_box = box;
  return box;
}

const jaco_manipulation::BoundingBox createDropBoundingBox(const anchor_msgs::Anchor &anchor) {
  jaco_manipulation::BoundingBox box;
//  box.header.stamp = ros::Time::now();
// TODO check id of drop object
  box.header.frame_id = "base_link";
  box.description = anchor.caffe.symbols[0];
  box.point = anchor.position.data.pose.position;
  box.point.x += anchor.shape.data.x * 0.5;
  box.dimensions = current_box.dimensions;
//  b.header.frame_id = "base_link";
//  b.description = "ball";
//  b.point.x = 0.4;
//  b.point.y = 0.3;
//  b.point.z = 0.03;
//  b.dimensions.x = 0.06;
//  b.dimensions.y = 0.06;
//  b.dimensions.z = 0.06;
//  current_box = box;
  return box;
}

bool processGoal(arm_service::ArmInstruction::Request &req,
                 arm_service::ArmInstruction::Response &res) {

  jaco_manipulation::client::JacoManipulationClient jmc;
  auto anchor = req.goal_anchor;
  auto instruction = req.instruction;

  if (instruction == "pick") {
    const jaco_manipulation::BoundingBox box = createGraspBoundingBox(anchor);
    jmc.graspAt(box);
  } else {
    const jaco_manipulation::BoundingBox box = createDropBoundingBox(anchor);
    jmc.graspAt(box);
  }
//  res.status = true;

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