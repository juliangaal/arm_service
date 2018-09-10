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

#include <arm_service/arm_service.h>
#include <arm_service/visuals/visuals_updater.h>
#include <actionlib/client/simple_action_client.h>
#include <jaco_manipulation/PlanAndMoveArmAction.h>

using namespace arm_service;
using JacoActionClient = actionlib::SimpleActionClient<jaco_manipulation::PlanAndMoveArmAction>;

int main(int argc, char **argv) {
  ros::init(argc, argv, "arm_service_node");

  ROS_INFO("Trying to connect to Jaco");

  JacoActionClient jaco_conn("plan_and_move_arm", true);
  jaco_conn.waitForServer();

  ROS_INFO("Starting Arm Service");
  ArmService arm("arm_service");
  
  ROS_INFO("Starting Visuals Updater");
  visuals::VisualsUpdater viz("visuals_updater");
  
  ros::spin();

  return 0;
}