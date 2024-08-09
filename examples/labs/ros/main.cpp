
 /**
 * PS Move API - An interface for the PS Move Motion Controller
 * Copyright (c) 2012 Thomas Perl <m@thp.io>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *    1. Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *
 *    2. Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 **/

#include "tracker.h"
#include <iostream>
#include <stdio.h>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

using namespace std;

int main(int argc, char **argv){
    printf("All Well \n");

    Tracker tracker;

    ros::init(argc, argv, "psmove_ros");
    ros::NodeHandle nh;
    ros::Publisher con_pub = nh.advertise<geometry_msgs::PoseStamped>("/psmove/left/pose", 1);
    geometry_msgs::PoseStamped pose_msg;
    ros::Rate loop_rate(60);

    while(ros::ok()){
        tracker.update();
        pose_msg.header.frame_id = "World";
        pose_msg.header.stamp = ros::Time::now();
        pose_msg.pose.position.x = tracker.m_position[0];
        pose_msg.pose.position.y = tracker.m_position[1];
        pose_msg.pose.position.z = tracker.m_position[2];

        pose_msg.pose.orientation.w = tracker.m_quaternion[0];
        pose_msg.pose.orientation.x = tracker.m_quaternion[1];
        pose_msg.pose.orientation.y = tracker.m_quaternion[2];
        pose_msg.pose.orientation.z = tracker.m_quaternion[3];

        con_pub.publish(pose_msg);

        loop_rate.sleep();
    }

    tracker.cleanup();
    printf("Goodbye \n");
}

