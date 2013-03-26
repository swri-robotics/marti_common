// *****************************************************************************
//
// Copyright (C) 2013 All Right Reserved, Southwest Research Institute® (SwRI®)
//
// Contract No.  10-62987
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

#include <transform_util/transformer.h>

namespace transform_util
{
  Transformer::Transformer() : initialized_(false)
  {

  }

  Transformer::~Transformer()
  {

  }

  void Transformer::Initialize(
      const boost::shared_ptr<tf::TransformListener>& tf)
  {
    tf_listener_ = tf;
    initialized_ = Initialize();
  }

  bool Transformer::Initialize()
  {
    return true;
  }

  bool Transformer::GetTransform(
      const std::string& target_frame,
      const std::string& source_frame,
      const ros::Time& time,
      tf::StampedTransform& transform) const
  {
    if (!tf_listener_)
    {
      return false;
    }

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
    catch (tf::LookupException& e)
    {
      ROS_ERROR("[transformer]: %s", e.what());
    }
    catch (tf::ConnectivityException& e)
    {
      ROS_ERROR("[transformer]: %s", e.what());
    }
    catch (tf::ExtrapolationException& e)
    {
      ROS_ERROR("[transformer]: %s", e.what());
    }
    catch (...)
    {
      ROS_ERROR("[transformer]: Exception looking up transform");
    }

    return has_transform;
  }
}
