// *****************************************************************************
//
// Copyright (c) 2014, Southwest Research Institute® (SwRI®)
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of Southwest Research Institute® (SwRI®) nor the
//       names of its contributors may be used to endorse or promote products
//       derived from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// *****************************************************************************

#include <boost/smart_ptr.hpp>

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geographic_msgs/GeoPose.h>
#include <geometry_msgs/PoseStamped.h>
#include <gps_common/GPSFix.h>

#include <swri_math_util/constants.h>
#include <swri_transform_util/local_xy_util.h>

/**
 * @file
 *
 * This is a lattitude/longitude analog to the tf_echo node in the tf
 * package. When run in a console, it periodically outputs the latitude,
 * longitude, and heading of the desired TF.
 *
 * <b>Usage:</b>
 * lat_lon_tf_echo fixed_frame_id target_frame_id
 *
 * fixed_frame_id is the id of the frame fixed at the /local_xy_origin, usually
 *    /far_field
 * target_frame_id is the id of the frame to find the coordinates of
 *
 * All outputs are in degrees, and heading follows the compass heading
 * convention (0° is North, clockwise angles are positive)
 *
 * <b>Subscribed Topics</b>
 * - \e /tf [geometry_msgs::Transform] - The transform from fixed_frame_id to
 *        target_frame_id must be published
 * - \e /local_xy_origin [geometry_msgs::PoseStamped] - This topic is used to 
 *        initialize the WGS84 transformer. Once it is initialized, the subscriber
 *        disconnects.
 *        The fields of this message should be filled as follows:
 *        - pose.position.y - longitude in degrees east of the prime meridian
 *        - pose.position.y - lattitude in degrees north of the equator.
 *        - pose.position.z - altitude in meters above the WGS84 ellipsoid
 *        All other fields in the message are ignored.
 */

class LatLonTFEchoNode
{
  public:
    LatLonTFEchoNode(ros::NodeHandle nh,
        std::string frame_id,
        std::string fixed_frame) :
        nh_(nh),
        frame_id_(frame_id),
        fixed_frame_(fixed_frame)
    {
      origin_sub_ = nh_.subscribe(
          "/local_xy_origin",
          1,
          &LatLonTFEchoNode::XYOriginCallback,
          this);
      timer_ = nh_.createTimer(ros::Duration(1),
          boost::bind(&LatLonTFEchoNode::TimerCallback, this));
    }

  private:
    ros::NodeHandle nh_;
    tf::TransformListener tf_listener;
    ros::Timer timer_;
    boost::shared_ptr<swri_transform_util::LocalXyWgs84Util> xy_wgs84_util_;
    ros::Subscriber origin_sub_;
    std::string frame_id_;
    std::string fixed_frame_;

    void XYOriginCallback(const topic_tools::ShapeShifter::ConstPtr msg)
    {
      try
      {
        const gps_common::GPSFixConstPtr origin = msg->instantiate<gps_common::GPSFix>();
        xy_wgs84_util_.reset(
            new swri_transform_util::LocalXyWgs84Util(
                origin->latitude,
                origin->longitude,
                origin->track,
                origin->altitude));
        origin_sub_.shutdown();
        return;
      }
      catch (...) {}

      try
      {
        const geometry_msgs::PoseStampedConstPtr origin = msg->instantiate<geometry_msgs::PoseStamped>();
        xy_wgs84_util_.reset(
            new swri_transform_util::LocalXyWgs84Util(
                origin->pose.position.y,    // Latitude
                origin->pose.position.x,    // Longitude
                0.0,                        // Heading
                origin->pose.position.z));  // Altitude
        origin_sub_.shutdown();
        return;
      }
      catch(...) {}

      try
      {
        const geographic_msgs::GeoPoseConstPtr origin = msg->instantiate<geographic_msgs::GeoPose>();
        xy_wgs84_util_.reset(
            new swri_transform_util::LocalXyWgs84Util(
                origin->position.latitude,
                origin->position.longitude,
                tf::getYaw(origin->orientation),
                origin->position.altitude));
        origin_sub_.shutdown();
        return;
      }
      catch(...) {}

      ROS_ERROR_THROTTLE(1.0, "Unsupported message type received for local_xy_origin.");
    }

    void TimerCallback()
    {
      if (!xy_wgs84_util_ || !xy_wgs84_util_->Initialized())
      {
        printf("Still waiting for /local_xy_origin\n");
        return;
      }
      if (!tf_listener.waitForTransform(fixed_frame_, frame_id_, ros::Time(0), ros::Duration(1.0)))
      {
        printf("Still waiting for transform from %s to %s\n",
            frame_id_.c_str(),
            fixed_frame_.c_str());
        return;
      }
      tf::StampedTransform transform;
      try
      {
        tf_listener.lookupTransform(fixed_frame_, frame_id_, ros::Time(0), transform);
      }
      catch (const tf::TransformException& ex)
      {
        ROS_ERROR("%s", ex.what());
        return;
      }
      double lat, lon;
      xy_wgs84_util_->ToWgs84(
          transform.getOrigin().x(), transform.getOrigin().y(),
          lat, lon);
      tf::Quaternion q = transform.getRotation();
      q.setY(0);
      q.setX(0);
      q.normalize();
      double heading = -q.getAngle() * swri_math_util::_rad_2_deg + 90;
      while (heading < 0)
        heading += 360;
      printf("Latitude: %f°, Longitude: %f°, Heading: %f°\n", lat, lon, heading);
    }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "lat_lon_tf_echo");
  if (argc < 3)
  {
    ROS_FATAL("No Frame Id specified");
    printf("Usage: lat_lon_tf_echo <fixed_frame_id> <target_frame_id>\n");
    ros::shutdown();
    return 1;
  }
  std::string fixed_frame(argv[1]);
  std::string frame_id(argv[2]);
  LatLonTFEchoNode node(ros::NodeHandle(), frame_id, fixed_frame);
  ros::spin();

  return (0);
}
