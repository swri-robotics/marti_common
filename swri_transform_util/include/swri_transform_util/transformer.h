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

#ifndef TRANSFORM_UTIL_TRANSFORMER_H_
#define TRANSFORM_UTIL_TRANSFORMER_H_

#include <map>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <tf2/transform_datatypes.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <swri_transform_util/transform.h>
#include <swri_transform_util/local_xy_util.h>

namespace swri_transform_util
{
  /**
   * A base class for transformers.
   *
   * Instantiations of this class implement an interface to get transforms from
   * certain types of frames (e.g. TF, WGS84) to other types of frames.
   */
  class Transformer
  {
    public:
      Transformer();
      virtual ~Transformer() = default;

      /**
       * Initialize the Transformer from a tf::TransformListener.
       *
       * @param tf A shared pointer to a tf::TransformListener that the
       *    Transformer wraps. It is recommended that every Transformer in a
       *    node use the same tf::TransformListener to reduce redundant
       *    computation.
       */
      void Initialize(std::shared_ptr<tf2_ros::Buffer> tf,
                      std::shared_ptr<LocalXyWgs84Util> xy_util);

      /**
       * Get a map of the transforms supported by this Transformer
       * @return A map from source frame IDs to list of destination frame IDs.
       *   A source->destination entry does not imply that the inverse
       *   transform is supported as well.
       */
      virtual std::map<std::string, std::vector<std::string> > Supports() const = 0;

      /**
       * Get a swri_transform_util::Transform
       *
       * Gets the swri_transform_util::Transform that transforms coordinates
       * from the source_frame into the target_frame. If the transform is not
       * available, return false.
       *
       * @param[in] target_frame Destination frame for transform
       * @param[in] source_frame Source frame for transform
       * @param[in] time Time that the transform is valid for. To get the most
       *    recent transform, use tf2::TimePoint(0)
       * @param[out] transform Output container for the transform
       * @return True if the transform was found, false if no transform between
       *    the specified frames is available for the specified time.
       */
      virtual bool GetTransform(
        const std::string& target_frame,
        const std::string& source_frame,
        const tf2::TimePoint& time,
        Transform& transform) = 0;

    protected:
      bool initialized_;
      std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
      std::shared_ptr<LocalXyWgs84Util> local_xy_util_;
      rclcpp::Logger logger_;

      virtual bool Initialize();

      virtual bool GetTransform(
          const std::string& target_frame,
          const std::string& source_frame,
          const tf2::TimePoint& time,
          geometry_msgs::msg::TransformStamped& transform) const;
  };
}

#endif  // TRANSFORM_UTIL_TRANSFORMER_H_
