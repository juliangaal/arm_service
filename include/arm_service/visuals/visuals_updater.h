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

#ifndef PROJECT_VISUALS_UPDATER_H
#define PROJECT_VISUALS_UPDATER_H

#include <ros/node_handle.h>
#include <anchor_msgs/Anchor.h>
#include <anchor_msgs/AnchorArray.h>
#include <arm_service/AnchorVisuals.h>
#include <jaco_manipulation/BoundingBox.h>
#include <jaco_manipulation/client/jaco_manipulation_client.h>

namespace arm_service {
namespace visuals {

class VisualsUpdater {
 public:
  VisualsUpdater();
  ~VisualsUpdater() = default;

 private:
  ros::NodeHandle nh_;

  anchor_msgs::AnchorArray last_anchors_;

  ros::ServiceServer service_;
  /**
  * Jaco manipulation client
  */
  jaco_manipulation::client::JacoManipulationClient jmc_;

  bool processGoal(arm_service::AnchorVisuals::Request &req, arm_service::AnchorVisuals::Response &res);
  const jaco_manipulation::BoundingBox createBoundingBox(const anchor_msgs::Anchor &anchor) const;
};
} // namespace arm_service
} // namespace visuals_updater

#endif //PROJECT_VISUALS_UPDATER_H
