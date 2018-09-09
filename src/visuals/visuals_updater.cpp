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

#include <arm_service/visuals/visuals_updater.h>

using namespace arm_service::visuals;

VisualsUpdater::VisualsUpdater() {
  service_ = nh_.advertiseService("visuals_updater", &VisualsUpdater::processGoal, this);
  ROS_INFO("Ready to receive Anchors to display");
}

bool VisualsUpdater::processGoal(arm_service::AnchorVisuals::Request &req, arm_service::AnchorVisuals::Response &res) {
  if (req.instruction == "add") {
    for (const auto &anchor: req.anchors.anchors) {
      jmc_.updatePlanningScene(createBoundingBox(anchor));
    }
    last_anchors_ = req.anchors;
  } else if (req.instruction == "wipe") {
    jmc_.wipePlanningScene();
  } else {
    return false;
  }

  return true;
}

const jaco_manipulation::BoundingBox VisualsUpdater::createBoundingBox(const anchor_msgs::Anchor &anchor) const {
  jaco_manipulation::BoundingBox box;

  box.header.frame_id = "base_link";
  box.description = anchor.id;
  box.point = anchor.position.data.pose.position;
  box.point.x += anchor.shape.data.x * 0.5; // correction: centroid is infront of bounding box from kinect
  box.dimensions = anchor.shape.data;
  return box;
}
