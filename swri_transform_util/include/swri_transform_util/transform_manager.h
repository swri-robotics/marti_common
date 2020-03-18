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

#ifndef TRANSFORM_UTIL_TRANSFORM_MANAGER_H_
#define TRANSFORM_UTIL_TRANSFORM_MANAGER_H_

#include <map>
#include <string>

#include <rclcpp/logger.hpp>

#include <boost/make_shared.hpp>
#include <boost/shared_ptr.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>

#include <tf2/transform_datatypes.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <swri_transform_util/local_xy_util.h>
#include <swri_transform_util/transform.h>
#include <swri_transform_util/transformer.h>

namespace swri_transform_util
{
  typedef std::map<std::string, std::shared_ptr<Transformer> > TransformerMap;
  typedef std::map<std::string, TransformerMap> SourceTargetMap;

  /**
   * Wrapper around tf::TransformListener to support non-TF transforms
   *
   * TransformManager wraps tf::TransformListener and provides a similar
   * interface to get Transforms between TF frames, UTM, and WGS84.
   *
   * TransformManager uses PluginLib to load all of the
   * swri_transform_util::Transformer plugins it can find. To extend the
   * functionality of TranformManager, create a new
   * swri_transform_util::Transformer plugin that implements the desired transform.
   */
  class TransformManager
  {
  public:
    explicit TransformManager(rclcpp::Node::SharedPtr node);

    /**
     * Initialize the TransformManager with a tf2_ros::Buffer
     *
     * The TransformManager must be initialized before it can be used.
     *
     * @param tf_buffer A shared pointer to a tf2_ros::Buffer that the
     *    Transformer wraps.
     */
    void Initialize(std::shared_ptr<tf2_ros::Buffer> tf_buffer);

    /**
     * Get the Transform between two frames at a specified time
     *
     * This function gets the transform from source_frame to target_frame at
     * the specified time and returns it as a swri_transform_util::Transform.
     *
     * The frame IDs for target_frame and source_frame can be either a frame id
     * in the current TF tree or one of the special frames /UTM or /WGS84.
     *
     * This method waits for a 0.1 second timeout if the transform is not
     * immediately available.
     *
     * @param[in] target_frame The TF (or special) frame id of the target
     * @param[in] source_frame The TF (or special) frame id of the source
     * @param[in] time         The requested time to request the transform.
     *    tf2::TimePoint(0) means the most recent time for which a valid transform
     *    is available.
     * @param[out] transform   The transform requested. If the function returns
     *    false, transform is not mutated.
     * @return True if the transform is supported and available at the
     *    requested time. False otherwise.
     */
    bool GetTransform(
        const std::string& target_frame,
        const std::string& source_frame,
        const tf2::TimePoint& time,
        Transform& transform) const;

    /**
     * Get the most recent Transform between two frames
     *
     * This function gets the most recent transform from source_frame to
     * target_frame and returns it as a swri_transform_util::Transform.
     *
     * The frame IDs for target_frame and source_frame can be either a frame id
     * in the current TF tree or one of the special frames /UTM or /WGS84.
     *
     * This method waits for a 0.1 second timeout if the transform is not
     * immediately available.
     *
     * @param[in] target_frame The TF (or special) frame id of the target
     * @param[in] source_frame The TF (or special) frame id of the source
     * @param[out] transform   The transform requested. If the function returns
     *    false, transform is not mutated
     * @return True if the transform is supported and available. False otherwise.
     */
    bool GetTransform(
        const std::string& target_frame,
        const std::string& source_frame,
        Transform& transform) const;

    /**
     * Check whether the TransformManager supports transforms from
     *    source_frame to target_frame
     *
     * Supporting a transform does not imply that the TransformManager supports
     * the inverse.
     *
     * @param[in] target_frame The TF (or special) frame id of the target
     * @param[in] source_frame The TF (or special) frame id of the source
     * @return True if the transform is supported. False if the transform is
     *    unsupported.
     */
    bool SupportsTransform(
        const std::string& target_frame,
        const std::string& source_frame) const;

    /**
     * Get the tf::Transform between two frames at a specified time
     *
     * This function is a thin wrapper around the tf::TransformListener. Only
     * TF frames are supported.
     *
     * This method waits for a 0.1 second timeout if the transform is not
     * immediately available.
     *
     * @param[in] target_frame The TF frame id of the target
     * @param[in] source_frame The TF frame id of the source
     * @param[in] time         The requested time to request the transform.
     *    tf2::TimePoint(0) means the most recent time for which a valid transform
     *    is available.
     * @param[out] transform   The transform requested. If the function returns
     *    false, transform is not mutated.
     * @return True if the transform is available at the requested time. False
     *    otherwise.
     */
    bool GetTransform(
        const std::string& target_frame,
        const std::string& source_frame,
        const tf2::TimePoint& time,
        geometry_msgs::msg::TransformStamped& transform) const;

    /**
     * Get the most recent tf::Transform between two frames
     *
     * This function is a thin wrapper around the tf::TransformListener. Only
     * TF frames are supported.
     *
     * This method waits for a 0.1 second timeout if the transform is not
     * immediately available.
     *
     * @param[in] target_frame The TF frame id of the target
     * @param[in] source_frame The TF frame id of the source
     * @param[out] transform   The transform requested. If the function returns
     *    false, transform is not mutated.
     * @return True if the transform is available. False otherwise.
     */
    bool GetTransform(
        const std::string& target_frame,
        const std::string& source_frame,
        geometry_msgs::msg::TransformStamped& transform) const;

    /**
     * Get the tf::Transform between two frames at a specified time
     *
     * This function is a thin wrapper around the tf::TransformListener. Only
     * TF frames are supported.
     *
     * If the frames are not immediately available, this method will wait
     * for the frames for the specified timeout period.
     *
     * @param[in] target_frame The TF frame id of the target
     * @param[in] source_frame The TF frame id of the source
     * @param[in] time         The requested time to request the transform.
     *    tf2::TimePoint(0) means the most recent time for which a valid transform
     *    is available.
     * @param[in] timeout      How long to wait for the transform to be
     *    available before returning False.
     * @param[out] transform   The transform requested. If the function returns
     *    false, transform is not mutated.
     * @return True if the transform is available at the requested time. False
     *    otherwise.
     */
    bool GetTransform(
        const std::string& target_frame,
        const std::string& source_frame,
        const tf2::TimePoint& time,
        const tf2::Duration& timeout,
        geometry_msgs::msg::TransformStamped& transform) const;

    /**
     * Get the most recent tf::Transform between two frames
     *
     * This function is a thin wrapper around the tf::TransformListener. Only
     * TF frames are supported.
     *
     * If the frames are not immediately available, this method will wait
     * for the frames for the specified timeout period.
     *
     * @param[in] target_frame The TF frame id of the target
     * @param[in] source_frame The TF frame id of the source
     * @param[in] timeout      How long to wait for the transform to be
     *    available before returning False.
     * @param[out] transform   The transform requested. If the function returns
     *    false, transform is not mutated.
     * @return True if the transform is available. False otherwise.
     */
    bool GetTransform(
        const std::string& target_frame,
        const std::string& source_frame,
        const tf2::Duration& timeout,
        geometry_msgs::msg::TransformStamped& transform) const;

    /**
     * @brief LocalXyUtil exposes the private instance of LocalXyWgs84Util
     * @return
     */
    const LocalXyWgs84UtilPtr& LocalXyUtil() const;

  private:
    rclcpp::Node::SharedPtr node_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;

    LocalXyWgs84UtilPtr local_xy_util_;

    SourceTargetMap transformers_;
  };
  typedef std::shared_ptr<TransformManager> TransformManagerPtr;
}

#endif  // TRANSFORM_UTIL_TRANSFORM_MANAGER_H_
