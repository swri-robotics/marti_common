// -----------------------------------------------------------------------------
// SECURITY CLASSIFICATION: UNCLASSIFIED
// -----------------------------------------------------------------------------
//
// Copyright (C) 2014 Southwest Research Institute
//
// Notwithstanding any copyright notice, U.S. Government rights in this work are
// defined by 252.227-7013 (f)(2) and 252.227-7014 (f)(2) as detailed below. Use
// of this work other than as specifically authorized by the U.S. Government may
// violate any copyrights that exist in this work.
//
// UNLIMITED RIGHTS
// DFARS Clause reference: 252.227-7013 (a)(15) and 252.227-7014 (a)(15)
//
// Unlimited Rights. The Government has the right to use, modify, reproduce,
// perform, display, release or disclose this (technical data or computer
// software) in whole or in part, in any manner, and for any purpose whatsoever,
// and to have or authorize others to do so.
//
// Contract No.  N00178-11-C-1005
// Contractor    Southwest Research Institute® (SwRI®)
// Address       6220 Culebra Road, San Antonio, Texas 78228-0510
//
// Distribution Statement D. Distribution authorized to the Department of
// Defense and U.S. DoD Contractors only in support of US DoD efforts. Other
// requests shall be referred to [PEO].
//
// Warning: - This document contains data whose export is restricted by the Arms
// Export Control Act (Title 22, U.S.C., Sec 2751, et seq.) as amended, or the
// Export Administration Act (Title 50, U.S.C., App 2401 et seq.) as amended.
// Violations of these export laws are subject to severe criminal and civil
// penalties. Disseminate in accordance with provisions of DoD Directive 5230.25
//

#include <boost/smart_ptr.hpp>

#include <ros/ros.h>
#include <tf/transform_listener.h>

#include <math_util/constants.h>
#include <transform_util/local_xy_util.h>

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
 * - \e /local_xy_origin [gsp_common::GPSFix] - This topic is used to initialize
 *        the WGS84 transformer. Once it is initialized, the subscriber
 *        disconnects.
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
    boost::shared_ptr<transform_util::LocalXyWgs84Util> xy_wgs84_util_;
    ros::Subscriber origin_sub_;
    std::string frame_id_;
    std::string fixed_frame_;

    void XYOriginCallback(const gps_common::GPSFixConstPtr origin)
    {
      xy_wgs84_util_.reset(
          new transform_util::LocalXyWgs84Util(
              origin->latitude,
              origin->longitude,
              origin->track,
              origin->altitude));
      origin_sub_.shutdown();
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
      double heading = -q.getAngle() * math_util::_rad_2_deg + 90;
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
