/*
 * Copyright (C) 2008, Morgan Quigley and Willow Garage, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the names of Stanford University or Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
#include "ros/ros.h"
#include <demo_test/pos_data.h>

demo_test::pos_data pos;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "demo_udp");
  ros::NodeHandle n;
  ros::Publisher chatter_pub = n.advertise<demo_test::pos_data>("demo_udp", 1000);
  ROS_INFO("Hello ROS, ready to send rigidbody pos_data!!!");
  ros::Rate loop_rate(100);
  int i = 0;
  while (ros::ok())
  {
    pos.num = 2;
    pos.pos[0] = i+1.1;
    pos.pos[1] = i+1.2;
    pos.pos[2] = i+1.3;
    pos.pos[3] = i+1.4;
    pos.pos[4] = i+1.5;
    pos.pos[5] = i+1.6;
    ROS_INFO("sending rigidbody pos_data");
    ROS_INFO("UAV 1: x=%f, y=%f, z=%f", pos.pos[0], pos.pos[1], pos.pos[2]);
    chatter_pub.publish(pos);
    i++;
    loop_rate.sleep();
  }

  return 0;
}
