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

#include <boost/make_shared.hpp>
#include <boost/shared_ptr.hpp>

#include <pluginlib/class_loader.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

#include <swri_transform_util/local_xy_util.h>
#include <swri_transform_util/transform.h>
#include <swri_transform_util/transformer.h>

namespace swri_transform_util
{
  typedef std::map<std::string, boost::shared_ptr<Transformer> > TransformerMap;
  typedef std::map<std::string, TransformerMap> SourceTargetMap;

  class TransformManager
  {
  public:
    TransformManager();
    ~TransformManager();

    void Initialize(boost::shared_ptr<tf::TransformListener> tf
        = boost::make_shared<tf::TransformListener>());

    bool GetTransform(
        const std::string& target_frame,
        const std::string& source_frame,
        const ros::Time& time,
        Transform& transform) const;

    bool GetTransform(
        const std::string& target_frame,
        const std::string& source_frame,
        Transform& transform) const;

    bool SupportsTransform(
        const std::string& target_frame,
        const std::string& source_frame) const;

    bool GetTransform(
        const std::string& target_frame,
        const std::string& source_frame,
        const ros::Time& time,
        tf::StampedTransform& transform) const;

    bool GetTransform(
        const std::string& target_frame,
        const std::string& source_frame,
        tf::StampedTransform& transform) const;

  private:
    pluginlib::ClassLoader<swri_transform_util::Transformer> loader_;
    boost::shared_ptr<tf::TransformListener> tf_listener_;

    boost::shared_ptr<LocalXyWgs84Util> local_xy_util_;

    SourceTargetMap transformers_;
  };
}

#endif  // TRANSFORM_UTIL_TRANSFORM_MANAGER_H_
