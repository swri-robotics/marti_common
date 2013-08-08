// *****************************************************************************
//
// Copyright (C) 2013 All Right Reserved, Southwest Research Institute® (SwRI®)
//
// Contract No.  10-R8248
// Contractor    Southwest Research Institute® (SwRI®)
// Address       6220 Culebra Road, San Antonio, Texas 78228-0510
// Contact       Kris Kozak <kkozak@swri.org> (210) 522-3854
//
// This code was developed as part of an internal research project fully funded
// by Southwest Research Institute®.
//
// THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
// KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
// PARTICULAR PURPOSE.
//
// *****************************************************************************

#include <transform_util/transform_manager.h>

#include <vector>

#include <transform_util/frames.h>

namespace transform_util
{
  TransformManager::TransformManager() :
      loader_("transform_util", "transform_util::Transformer")
  {
    std::vector<std::string> class_names = loader_.getDeclaredClasses();

    for (uint32_t i = 0; i < class_names.size(); i++)
    {
      try
      {
        boost::shared_ptr<Transformer> transformer = loader_.createInstance(class_names[i]);

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
      catch (const pluginlib::CreateClassException& e)
      {
        ROS_ERROR("[transform_manager]: Failed to load transformer plugin '%s': %s",
            class_names[i].c_str(), e.what());
      }
    }
  }

  TransformManager::~TransformManager()
  {
  }

  void TransformManager::Initialize(boost::shared_ptr<tf::TransformListener> tf)
  {
    tf_listener_ = tf;

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
      Transform& transform)
  {
    std::string src_frame = source_frame;
    std::string tgt_frame = target_frame;
    if (tgt_frame == src_frame)
    {
      transform = Transform();
      return true;
    }

    if (!tf_listener_)
    {
      ROS_WARN("[transform_manager]: TF listener not initialized.");
      return false;
    }

    // Check if the source frame is in the TF tree.
    std::string source = src_frame;
    if (tf_listener_->frameExists(src_frame))
    {
      source = _tf_frame;
    }

    // Check if the target frame is in the TF tree.
    std::string target = tgt_frame;
    if (tf_listener_->frameExists(tgt_frame))
    {
      target = _tf_frame;
    }

    // Check if either of the frames is local_xy
    if (source == _local_xy_frame)
    {
      source = _tf_frame;
      if (!ros::param::get("/local_xy_frame", src_frame))
      {
        ROS_ERROR("[transform_manager]: Failed to parse /local_xy_frame.");
        return false;
      }
    }

    if (target == _local_xy_frame)
    {
      target = _tf_frame;
      if (!ros::param::get("/local_xy_frame", tgt_frame))
      {
        ROS_ERROR("[transform_manager]: Failed to parse /local_xy_frame.");
        return false;
      }
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

      ROS_ERROR("[transform_manager]: Failed to get tf transform.");
      return false;
    }

    if (transformers_[source].count(target) == 0)
    {
      ROS_ERROR("[transform_manager]: No transformer for transforming %s to %s",
          source.c_str(), target.c_str());

      return false;
    }

    boost::shared_ptr<Transformer> transformer = transformers_[source][target];

    if (!transformer)
    {
      ROS_ERROR("[transform_manager]: No transformer for transforming %s to %s",
          source.c_str(), target.c_str());

      return false;
    }

    return transformer->GetTransform(tgt_frame, src_frame, time, transform);
  }

  bool TransformManager::GetTransform(
      const std::string& target_frame,
      const std::string& source_frame,
      Transform& transform)
  {
    return GetTransform(target_frame, source_frame, ros::Time(0), transform);
  }

  bool TransformManager::GetTransform(
      const std::string& target_frame,
      const std::string& source_frame,
      const ros::Time& time,
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
          ros::Duration(0.1));

      tf_listener_->lookupTransform(
          target_frame,
          source_frame,
          time,
          transform);

      has_transform = true;
    }
    catch (const tf::LookupException& e)
    {
      ROS_ERROR("[transform_manager]: %s", e.what());
    }
    catch (const tf::ConnectivityException& e)
    {
      ROS_ERROR("[transform_manager]: %s", e.what());
    }
    catch (const tf::ExtrapolationException& e)
    {
      ROS_ERROR("[transform_manager]: %s", e.what());
    }
    catch (...)
    {
      ROS_ERROR("[transform_manager]: Exception looking up transform");
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
}
