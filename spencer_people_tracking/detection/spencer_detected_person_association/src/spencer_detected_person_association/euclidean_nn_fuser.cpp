/*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2014-2015, Timm Linder, Social Robotics Lab, University of Freiburg
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions are met:
*
*  * Redistributions of source code must retain the above copyright notice, this
*    list of conditions and the following disclaimer.
*  * Redistributions in binary form must reproduce the above copyright notice,
*    this list of conditions and the following disclaimer in the documentation
*    and/or other materials provided with the distribution.
*  * Neither the name of the copyright holder nor the names of its contributors
*    may be used to endorse or promote products derived from this software
*    without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
*  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
*  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
*  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
*  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
*  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
*  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
*  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
*  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <pluginlib/class_list_macros.h>
#include "euclidean_nn_fuser.h"

#include <Eigen/Core>
#include <limits>
#include <eigen_conversions/eigen_msg.h>


namespace spencer_detected_person_association
{
    float EuclideanNNFuserNodelet::computeDistance(const spencer_tracking_msgs::CompositeDetectedPerson& d1, const spencer_tracking_msgs::CompositeDetectedPerson& d2)
    {
        // Compute the Euclidean distance in X and Y
        float distance = hypot(d1.pose.pose.position.x - d2.pose.pose.position.x, d1.pose.pose.position.y - d2.pose.pose.position.y);

        // Gating: If it fails, set distance to infinity
        double gatingDistance = 1.0; getPrivateNodeHandle().getParamCached("gating_distance", gatingDistance);
        if(distance > gatingDistance) distance = std::numeric_limits<float>::infinity();

        return distance;
    }

    void EuclideanNNFuserNodelet::fusePoses(const spencer_tracking_msgs::CompositeDetectedPerson& d1, const spencer_tracking_msgs::CompositeDetectedPerson& d2, geometry_msgs::PoseWithCovariance& fusedPose)
    {
        double weight_1;
        double weight_2;
        // Obtain and normalize weights of arithmetic mean
        // It is assumed that one topic has detections coming from laser data and the other topic detections from rgb data. 
        // laser distances are more reliable, thus give them more weight! (params are set in launch file)
        if (d1.original_detections[0].modality == "laser2d" || d1.original_detections[0].modality == "laser3d"){
            weight_1 = 1; getPrivateNodeHandle().getParamCached("pose_weight_for_laser", weight_1);
            weight_2 = 0; getPrivateNodeHandle().getParamCached("pose_weight_for_rgb", weight_2);
        }
        else{
            weight_2 = 1; getPrivateNodeHandle().getParamCached("pose_weight_for_laser", weight_2);
            weight_1 = 0; getPrivateNodeHandle().getParamCached("pose_weight_for_rgb", weight_1);
        }

        const float weightSum = weight_1 + weight_2;
        weight_1 /= weightSum; weight_2 /= weightSum;

        // Compute the arithmetic mean of the X,Y,Z positions as well as the corresponding covariances.
        fusedPose.pose.position.x = weight_1 * d1.pose.pose.position.x + weight_2 * d2.pose.pose.position.x;
        fusedPose.pose.position.y = weight_1 * d1.pose.pose.position.y + weight_2 * d2.pose.pose.position.y;
        fusedPose.pose.position.z = weight_1 * d1.pose.pose.position.z + weight_2 * d2.pose.pose.position.z;

        // Interpolate the orientation using SLERP
        Eigen::Quaterniond q1, q2, qInterpolated;
        tf::quaternionMsgToEigen(d1.pose.pose.orientation, q1);
        tf::quaternionMsgToEigen(d2.pose.pose.orientation, q2);
        qInterpolated = q1.slerp(weight_2, q2);
        tf::quaternionEigenToMsg(qInterpolated, fusedPose.pose.orientation);

        // Take component-wise minimum of the covariance matrices
        for(size_t i = 0; i < d1.pose.covariance.size(); i++) {
            // FIXME: Is this a good idea for tracking? This is similar to using the 'stronger peak' of the two distributions,
            // as we hopefully don't get less-informed by fusing the two detections (if our association makes sense)
            // There is probably no 'correct way' as the mixture of two Gaussians is not a Gaussian
            fusedPose.covariance[i] = std::min(d1.pose.covariance[i], d2.pose.covariance[i]);
        }
    }
}


PLUGINLIB_DECLARE_CLASS(spencer_detected_person_association, EuclideanNNFuserNodelet, spencer_detected_person_association::EuclideanNNFuserNodelet, nodelet::Nodelet)
