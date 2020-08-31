/******************************************************************************
Copyright (c) 2017, Alexander W. Winkler. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

#include <ros/ros.h>

#include <xpp_msgs/topic_names.h>

#include <xpp_states/convert.h>
#include <xpp_msgs/RobotStateJoint.h>



// crocoddyl imports
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/parsers/utils.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <crocoddyl/multibody/utils/quadruped-gaits.hpp>
#include <crocoddyl/core/utils/callbacks.hpp>
#include <crocoddyl/core/solvers/fddp.hpp>
#include <crocoddyl/core/utils/timer.hpp>



//using namespace xpp;

int main(int argc, char *argv[])
{

    // crocoddyl setup
    pinocchio::Model model;
//    std::string urdfPath = "/home/tannerliu/Research/catkin_ws/catkin_ws_xpp/src/xpp/robots/xpp_laikago/urdf/laikago.urdf";
//    pinocchio::urdf::buildModel(urdfPath, pinocchio::JointModelFreeFlyer(), model);
//    model.lowerPositionLimit(0,0) = -1; model.lowerPositionLimit(1, 0) = -1; model.lowerPositionLimit(2,0) =-1; model.lowerPositionLimit(3,0) = -1;
//    model.lowerPositionLimit(4,0) = -1; model.lowerPositionLimit(5,0) = -1; model.lowerPositionLimit(6,0) = -1;
//    model.upperPositionLimit(0, 0) = 1; model.upperPositionLimit(1,0) = 1; model.upperPositionLimit(2,0) = 1; model.upperPositionLimit(3,0) = 1;
//    model.upperPositionLimit(4,0) = 1; model.upperPositionLimit(5,0) = 1; model.upperPositionLimit(6,0) = 1;
//    crocoddyl::SimpleQuadrupedGaitProblem gait(model, "toeFL", "toeFR", "toeRL", "toeRR");
//    const Eigen::VectorXd& x0 = gait.get_defaultState();



    // Ros initialization
    ros::init(argc, argv, "laikago_publisher_node");

    ros::NodeHandle n;

    ros::Publisher joint_state_pub = n.advertise<xpp_msgs::RobotStateJoint>("xpp/joint_laikago_des", 1);
    ROS_DEBUG("Publishing to: %s", joint_state_pub.getTopic().c_str());

    ROS_INFO_STREAM("Waiting for Subscriber...");
    while(ros::ok() && joint_state_pub.getNumSubscribers() == 0)
        ros::Rate(100).sleep();
    ROS_INFO_STREAM("Subscriber to initial state connected");


    // publishes a sequence of states for a total duration of T spaced 0.01s apart.
    double dt = 0.01;
    unsigned int count = 0;
    while (ros::ok())
    {

        xpp_msgs::RobotStateJoint joint_msg;
        Eigen::VectorXd q(12);
        if (count % 2 == 0) q << 0.0, 0.6, -1.15, 0, 0.6, -1.15, 0, 0.6, -1.15, 0, 0.6, -1.15;
        else q << 0.0, 0.4, -1, 0, 0.4, -1, 0, 0.4, -1, 0, 0.4, -1;
        count++;
        joint_msg.joint_state.position = std::vector<double>(q.data(), q.data()+q.size());
        joint_msg.base.pose.position.z = 0.42;
        joint_msg.base.pose.orientation.w = 1;
        joint_state_pub.publish(joint_msg);
        ros::spinOnce();
        ros::Duration(dt).sleep(); // pause loop so visualization has correct speed.
    }


    return 0;
}

