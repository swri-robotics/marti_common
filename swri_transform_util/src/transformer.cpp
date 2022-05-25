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

#include <swri_transform_util/transformer.h>

namespace swri_transform_util
{
  Transformer::Transformer() :
    initialized_(false),
    logger_(rclcpp::get_logger("swri_transform_util::Transformer"))
  {
  }

  void Transformer::Initialize(
      const std::shared_ptr<tf2_ros::Buffer> tf,
      const std::shared_ptr<LocalXyWgs84Util> xy_util)
  {
    tf_buffer_ = tf;
    local_xy_util_ = xy_util;
    initialized_ = Initialize();
  }

  bool Transformer::Initialize()
  {
    return true;
  }

  bool Transformer::GetTransform(
      const std::string& target_frame,
      const std::string& source_frame,
      const tf2::TimePoint& time,
      geometry_msgs::msg::TransformStamped& transform) const
  {
    if (!tf_buffer_)
    {
      return false;
    }

    bool has_transform = false;
    try
    {
      if (tf_buffer_->_frameExists(target_frame) &&
          tf_buffer_->_frameExists(source_frame))
      {
        transform = tf_buffer_->lookupTransform(
            target_frame,
            source_frame,
            time,
            std::chrono::milliseconds(10));

        has_transform = true;
      }
    }
    catch (const tf2::LookupException& e)
    {
      RCLCPP_ERROR(logger_, "[transformer]: %s", e.what());
    }
    catch (const tf2::ConnectivityException& e)
    {
      RCLCPP_ERROR(logger_, "[transformer]: %s", e.what());
    }
    catch (const tf2::ExtrapolationException& e)
    {
      RCLCPP_ERROR(logger_, "[transformer]: %s", e.what());
    }
    catch (...)
    {
      RCLCPP_ERROR(logger_, "[transformer]: Exception looking up transform");
    }

    return has_transform;
  }
}
