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

#include <swri_transform_util/transform_manager.h>

#include <vector>

#include <swri_transform_util/frames.h>
#include <swri_transform_util/utm_transformer.h>
#include <swri_transform_util/wgs84_transformer.h>

namespace swri_transform_util
{
  TransformManager::TransformManager()
  {
    std::vector<boost::shared_ptr<Transformer> > transformers;
    transformers.push_back(boost::make_shared<Wgs84Transformer>());
    transformers.push_back(boost::make_shared<UtmTransformer>());

    for (size_t i = 0; i < transformers.size(); i++)
    {
      boost::shared_ptr<Transformer> transformer = transformers[i];
      std::map<std::string, std::vector<std::string> > supports = transformer->Supports();

      std::map<std::string, std::vector<std::string> >::iterator iter;
      for (iter = supports.begin(); iter != supports.end(); ++iter)
      {
        for (uint32_t j = 0; j < iter->second.size(); j++)
        {
          if (transformers_[iter->first].count(iter->second[j]) > 0)
          {
            ROS_WARN("[transform_manager]: Transformer conflict for %s to %s",
                iter->first.c_str(), iter->second[j].c_str());
          }

          transformers_[iter->first][iter->second[j]] = transformer;
        }
      }
    }
  }

  TransformManager::~TransformManager()
  {
  }

  void TransformManager::Initialize(boost::shared_ptr<tf::TransformListener> tf)
  {
    tf_listener_ = tf;

    local_xy_util_ = boost::make_shared<LocalXyWgs84Util>();

    std::map<std::string, std::map<std::string, boost::shared_ptr<Transformer> > >::iterator iter1;
    for (iter1 = transformers_.begin(); iter1 != transformers_.end(); ++iter1)
    {
      std::map<std::string, boost::shared_ptr<Transformer> >::iterator iter2;
      for (iter2 = iter1->second.begin(); iter2 != iter1->second.end(); ++iter2)
      {
        iter2->second->Initialize(tf);
      }
    }
  }

  bool TransformManager::GetTransform(
      const std::string& target_frame,
      const std::string& source_frame,
      const ros::Time& time,
      Transform& transform) const
  {
    std::string src_frame = NormalizeFrameId(source_frame);
    std::string tgt_frame = NormalizeFrameId(target_frame);
    if (tgt_frame == src_frame)
    {
      transform = Transform();
      return true;
    }

    if (!tf_listener_)
    {
      ROS_WARN_THROTTLE(2.0, "[transform_manager]: TF listener not initialized.");
      return false;
    }

    // Check if the source frame is in the TF tree.
    std::string source = src_frame;
    if (tf_listener_->frameExists(source))
    {
      source = _tf_frame;
    }

    // Check if the target frame is in the TF tree.
    std::string target = tgt_frame;
    if (tf_listener_->frameExists(target))
    {
      target = _tf_frame;
    }

    // Check if either of the frames is local_xy
    if (source == _local_xy_frame)
    {
      source = _tf_frame;

      if (!local_xy_util_->Initialized())
      {
        ROS_WARN_THROTTLE(2.0, "[transform_manager]: Local XY frame has not been initialized.");
        return false;
      }

      src_frame = local_xy_util_->Frame();
    }

    if (target == _local_xy_frame)
    {
      target = _tf_frame;
      if (!local_xy_util_->Initialized())
      {
        ROS_WARN_THROTTLE(2.0, "[transform_manager]: Local XY frame has not been initialized.");
        return false;
      }

      tgt_frame = local_xy_util_->Frame();
    }

    if (source == target)
    {
      // Both frames are in the TF tree.

      tf::StampedTransform tf_transform;
      if (GetTransform(tgt_frame, src_frame, time, tf_transform))
      {
        transform = tf_transform;
        return true;
      }

      ROS_WARN_THROTTLE(
        2.0,
        "[transform_manager]: Failed to get tf transform ('%s' to '%s').  Both "
        "frames exist in tf.",
        source_frame.c_str(), target_frame.c_str());
      return false;
    }

    SourceTargetMap::const_iterator source_iter = transformers_.find(source);
    if (source_iter == transformers_.end())
    {
      ROS_WARN_THROTTLE(
        2.0,
        "[transform_manager]: No transformer from '%s' to '%s'."
        " If '%s' is a /tf frame, it may not have been broadcast recently.",
        source.c_str(), target.c_str(), source.c_str());

      return false;
    }

    TransformerMap::const_iterator target_iter = source_iter->second.find(target);
    if (target_iter == source_iter->second.end())
    {
      ROS_WARN_THROTTLE(
        2.0,
        "[transform_manager]: No transformer from '%s' to '%s'."
        " If '%s' is a /tf frame, it may not have been broadcast recently.",
        source.c_str(), target.c_str(), target.c_str());

      return false;
    }

    boost::shared_ptr<Transformer> transformer = target_iter->second;

    if (!transformer)
    {
      ROS_ERROR_THROTTLE(2.0, "[transform_manager]: Encountered null transformer for '%s' to '%s'.",
          source.c_str(), target.c_str());

      return false;
    }

    return transformer->GetTransform(tgt_frame, src_frame, time, transform);
  }

  bool TransformManager::GetTransform(
      const std::string& target_frame,
      const std::string& source_frame,
      Transform& transform) const
  {
    return GetTransform(target_frame, source_frame, ros::Time(0), transform);
  }

  bool TransformManager::SupportsTransform(
      const std::string& target_frame,
      const std::string& source_frame) const
  {
    std::string source = NormalizeFrameId(source_frame);
    std::string target = NormalizeFrameId(target_frame);
    if (target == source)
    {
      return true;
    }

    if (!tf_listener_)
    {
      return false;
    }

    // Check if the source frame is in the TF tree.
    if (tf_listener_->frameExists(source))
    {
      source = _tf_frame;
    }

    // Check if the target frame is in the TF tree.
    if (tf_listener_->frameExists(target))
    {
      target = _tf_frame;
    }

    // Check if either of the frames is local_xy
    if (source == _local_xy_frame)
    {
      source = _tf_frame;
      if (!local_xy_util_->Initialized())
      {
        ROS_WARN_THROTTLE(2.0, "[transform_manager]: Local XY frame has not been initialized.");
        return false;
      }
    }

    if (target == _local_xy_frame)
    {
      target = _tf_frame;
      if (!local_xy_util_->Initialized())
      {
        ROS_WARN_THROTTLE(2.0, "[transform_manager]: Local XY frame has not been initialized.");
        return false;
      }
    }

    if (source == target)
    {
      return true;
    }

    SourceTargetMap::const_iterator source_iter = transformers_.find(source);
    if (source_iter == transformers_.end())
    {
      ROS_WARN_THROTTLE(
        2.0,
        "[transform_manager]: No transformer for transforming '%s' to '%s'."
        " If '%s' is a /tf frame, it may not have been broadcast recently.",
        source.c_str(), target.c_str(), source.c_str());

      return false;
    }

    TransformerMap::const_iterator target_iter = source_iter->second.find(target);
    if (target_iter == source_iter->second.end())
    {
      ROS_WARN_THROTTLE(
        2.0,
        "[transform_manager]: No transformer for transforming '%s' to '%s'."
        " If '%s' is a /tf frame, it may not have been broadcast recently.",
        source.c_str(), target.c_str(), target.c_str());

      return false;
    }

    return true;
  }

  bool TransformManager::GetTransform(
      const std::string& target_frame,
      const std::string& source_frame,
      const ros::Time& time,
      tf::StampedTransform& transform) const
  {
    return GetTransform(target_frame, source_frame, time, ros::Duration(0.1), transform);
  }

  bool TransformManager::GetTransform(
      const std::string& target_frame,
      const std::string& source_frame,
      const ros::Time& time,
      const ros::Duration& timeout,
      tf::StampedTransform& transform) const
  {
    if (!tf_listener_)
      return false;

    bool has_transform = false;
    try
    {
      tf_listener_->waitForTransform(
          target_frame,
          source_frame,
          time,
          timeout);

      tf_listener_->lookupTransform(
          target_frame,
          source_frame,
          time,
          transform);

      has_transform = true;
    }
    catch (const tf::LookupException& e)
    {
      ROS_ERROR_THROTTLE(2.0, "[transform_manager]: %s", e.what());
    }
    catch (const tf::ConnectivityException& e)
    {
      ROS_ERROR_THROTTLE(2.0, "[transform_manager]: %s", e.what());
    }
    catch (const tf::ExtrapolationException& e)
    {
      ROS_ERROR_THROTTLE(2.0, "[transform_manager]: %s", e.what());
    }
    catch (...)
    {
      ROS_ERROR_THROTTLE(2.0, "[transform_manager]: Exception looking up transform");
    }

    return has_transform;
  }

  bool TransformManager::GetTransform(
      const std::string& target_frame,
      const std::string& source_frame,
      tf::StampedTransform& transform) const
  {
    return GetTransform(target_frame, source_frame, ros::Time(0), transform);
  }

  bool TransformManager::GetTransform(
      const std::string& target_frame,
      const std::string& source_frame,
      const ros::Duration& timeout,
      tf::StampedTransform& transform) const
  {
    return GetTransform(target_frame, source_frame, ros::Time(0), timeout, transform);
  }

  const LocalXyWgs84UtilPtr &TransformManager::LocalXyUtil() const
  {
    return local_xy_util_;
  }
}
