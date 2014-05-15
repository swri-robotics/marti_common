#include <math.h>

#include <ros/ros.h>

#include <nav_msgs/Odometry.h>
#include <novatel_oem628/NovatelPosition.h>
#include <geometry_msgs/Point.h>
#include <transform_util/local_xy_util.h>

/**
 * @file
 *
 * This node subscribes to a novatel_oem628/NovatelPosition message, converts
 * it to an odometry message with variances, and publishes the results.
 *
 * @author Ed Venator (evenator@swri.org)
 *
 * <b>Topics Subscribed</b>
 * - \e /local_xy_origin <tt>gps_common::GPSFix</tt> - The lattitude and
 *          longitude of the local world frame TF. The name of this TF is
 *          specified by a parameter.
 * - \e /bestpos <tt>novatel_oem628::NovatelPosition</tt> - The position data
 *          from the Novatel GPS.
 *
 * <b>Topics Published</b>
 * - \e /odom <tt>nav_msgs::Odometry</tt> - Odometry message created from the
 *          NovatelPosition message. One is published every time a new
 *          NovatelPosition message is received.
 *
 * <b>Parameters</b>
 * - \e ~/frame <tt>string</tt> - The local world frame TF name. Assumed to
 *          be oriented ENU. (See
 *          http://129.162.112.231/mediawiki/index.php/Coordinate_Frames).
 *          ["/far_field"]
 */

class NovatelPosition2OdometryNode
{
  public:
    NovatelPosition2OdometryNode(ros::NodeHandle nh):
      nh_(nh),
      large_variance_(1e20)
    {
      local_xy_ = boost::make_shared<transform_util::LocalXyWgs84Util>();
      sub_ = nh.subscribe("bestpos",
          50,
          &NovatelPosition2OdometryNode::novatelPositionCallback,
          this);
      pub_ = nh.advertise<nav_msgs::Odometry>("odom", 50);
      ros::NodeHandle pnh("~");
      pnh.param<std::string>("frame", frame_, "/far_field");
    }
  private:
    std::string frame_;
    ros::Publisher pub_;
    ros::Subscriber sub_;
    ros::NodeHandle nh_;
    transform_util::LocalXyWgs84UtilPtr local_xy_;
    const double large_variance_;

    /**
     * @brief      Converts NovatelPosition messages to Odometry messages
     *
     * @param[in]     novatel_pos     Novatel Position input
     */
    void novatelPositionCallback(
        novatel_oem628::NovatelPositionConstPtr best_pos)
    {
      if (!local_xy_->Initialized())
      {
        ROS_WARN_THROTTLE(5, "NovatelPosition to Odometry node waiting for LocalXYOrigin");
        return;
      }
      nav_msgs::Odometry msg_out;
      msg_out.header = best_pos->header;
      msg_out.header.frame_id = frame_;
      // Ok to hardcode child_frame since it's only used for twist
      msg_out.child_frame_id = "/gps";

      geometry_msgs::Point& X = msg_out.pose.pose.position;
      local_xy_->ToLocalXy(best_pos->lat, best_pos->lon, X.x, X.y);
      X.z = best_pos->height;
      geometry_msgs::Quaternion& Q = msg_out.pose.pose.orientation;
      Q.x = 0.0;
      Q.y = 0.0;
      Q.z = 0.0;
      Q.w = 1.0;

      // Set covariance
      // Assumes far_field frame is pointed East
      // (see http://129.162.112.231/mediawiki/index.php/Coordinate_Frames)
      double x_var = pow(best_pos->lon_sigma, 2);;
      double y_var = pow(best_pos->lat_sigma, 2);
      double z_var = pow(best_pos->height_sigma, 2);
      boost::array<double, 36>& cov = msg_out.pose.covariance;
      cov.assign(0.0);
      cov[0] = x_var;
      cov[7] = y_var;
      cov[14] = z_var;
      cov[21] = large_variance_;  // Yaw
      cov[28] = large_variance_;  // Pitch
      cov[35] = large_variance_;  // Roll

      pub_.publish(msg_out);
    }

};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "novatel_position2odometry");
  ros::NodeHandle nh;
  NovatelPosition2OdometryNode node(nh);
  ros::spin();
}
