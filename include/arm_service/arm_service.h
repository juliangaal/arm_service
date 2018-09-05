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
#include <jaco_manipulation/jaco_manipulation_client.h>

class ArmService {
 public:
  ArmService();
  ~ArmService() = default;

 private:
  jaco_manipulation::client::JacoManipulationClient jmc_;
  ros::NodeHandle nh_;
  jaco_manipulation::BoundingBox current_box_;
  ros::ServiceServer service_;

};

#endif //PROJECT_ARM_SERVICE_H
