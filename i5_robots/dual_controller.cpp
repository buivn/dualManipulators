/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2017-2018, Hoang-Dung Bui
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *       * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *       * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *       * Neither the name of the Southwest Research Institute, nor the names
 *       of its contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
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

#include <string>
#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
// include a custom message - actually there is no theta.h file, only theta.msg
// #include <aubo_driver/theta.h>


// using namespace aubo_driver;
int main(int argc, char **argv)
{
    ros::init(argc, argv, "dual_controller");
    ros::NodeHandle n;

    std::string robotname1 = "robot1";
    std::string moveAPI_cmd1 = "/" + robotname1 + "/moveAPI_cmd";
    std::string robotname2 = "robot2";
    std::string moveAPI_cmd2 = "/" + robotname2 + "/moveAPI_cmd";
    
    // publish two topics
    ros::Publisher thetaPublish1 = n.advertise<std_msgs::Float32MultiArray>(moveAPI_cmd1, 2);
    ros::Publisher thetaPublish2 = n.advertise<std_msgs::Float32MultiArray>(moveAPI_cmd2, 2);
    
    ros::Rate loop_rate(0.1);


    int thetaMatrix[2][6];
    thetaMatrix[0][0] = 0;
    thetaMatrix[0][1] = 20;
    thetaMatrix[0][2] = 20;
    thetaMatrix[0][3] = 20;
    thetaMatrix[0][4] = 20;
    thetaMatrix[0][5] = 20;
    thetaMatrix[0][6] = 20;
    thetaMatrix[1][0] = 0;
    thetaMatrix[1][1] = 30;
    thetaMatrix[1][2] = 50;
    thetaMatrix[1][3] = 50;
    thetaMatrix[1][4] = 20;
    thetaMatrix[1][5] = 20;
    thetaMatrix[1][6] = 20;


    for(int i=0; i<20; i++)
    {
      
      std_msgs::Float32MultiArray msg1;
      std_msgs::Float32MultiArray msg2;
      // msg.layout = 0;
      msg1.data.push_back(thetaMatrix[i%2][0]*3.14/180);
      msg1.data.push_back(thetaMatrix[i%2][1]*3.14/180);
      msg1.data.push_back(thetaMatrix[i%2][2]*3.14/180);
      msg1.data.push_back(thetaMatrix[i%2][3]*3.14/180);
      msg1.data.push_back(thetaMatrix[i%2][4]*3.14/180);
      msg1.data.push_back(thetaMatrix[i%2][5]*3.14/180);
      msg1.data.push_back(thetaMatrix[i%2][6]*3.14/180);

      msg2.data.push_back(thetaMatrix[i%2][0]*3.14/180);
      msg2.data.push_back(thetaMatrix[i%2][1]*3.14/180);
      msg2.data.push_back(thetaMatrix[i%2][2]*3.14/180);
      msg2.data.push_back(thetaMatrix[i%2][3]*3.14/180);
      msg2.data.push_back(thetaMatrix[i%2][4]*3.14/180);
      msg2.data.push_back(thetaMatrix[i%2][5]*3.14/180);
      msg2.data.push_back(thetaMatrix[i%2][6]*3.14/180);



   
      // ROS_INFO("%s", msg.data.c_str());
       
           /**
           * The publish() function is how you send messages. The parameter
           * is the message object. The type of this object must agree with the type
           * given as a template parameter to the advertise<>() call, as was done
           * in the constructor above.
           */
      thetaPublish1.publish(msg1);
      thetaPublish2.publish(msg2);
       
      ros::spinOnce();
       
      loop_rate.sleep();

    }
    // ros::AsyncSpinner spinner(6);
    // spinner.start();

    // ros::Rate loop_rate(robot_driver.UPDATE_RATE_);
    // while(ros::ok())
    // {
    //     robot_driver.updateControlStatus();
    //     loop_rate.sleep();
    //     ros::spinOnce();
    // }
    // ROS_WARN("Exiting robot_driver");
    return(0);
}


