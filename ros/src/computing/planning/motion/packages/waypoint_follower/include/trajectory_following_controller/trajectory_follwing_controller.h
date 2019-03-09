#ifndef WAYPOINT_FOLLWER_TRAJECTORY_FOLLWING_CONTROLLER_H_INCLUDED
#define WAYPOINT_FOLLWER_TRAJECTORY_FOLLWING_CONTROLLER_H_INCLUDED
/*
 * Copyright 2015-2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

//headers in ROS
#include <ros/ros.h>

//headers in Autoware
#include <trajectory_following_controller/clothoid.h>

class TrajectoryFollowingController
{
public:
    TrajectoryFollowingController(ros::NodeHandle nh,ros::NodeHandle pnh);
    ~TrajectoryFollowingController();
private:
    ros::NodeHandle nh;
    ros::NodeHandle pnh;
    ros::Subscriber waypoint_sub_;
    std::string waypoint_topic_;
};

#endif  //WAYPOINT_FOLLWER_TRAJECTORY_FOLLWING_CONTROLLER_H_INCLUDED