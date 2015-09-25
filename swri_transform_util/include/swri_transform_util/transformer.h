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

#include <boost/shared_ptr.hpp>

#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

#include <swri_transform_util/transform.h>

namespace swri_transform_util
{
  class Transformer
  {
    public:
      Transformer();
      virtual ~Transformer();

      void Initialize(const boost::shared_ptr<tf::TransformListener> tf);

      virtual std::map<std::string, std::vector<std::string> > Supports() const = 0;

      virtual bool GetTransform(
        const std::string& target_frame,
        const std::string& source_frame,
        const ros::Time& time,
        Transform& transform) = 0;

    protected:
      bool initialized_;
      boost::shared_ptr<tf::TransformListener> tf_listener_;

      virtual bool Initialize();

      virtual bool GetTransform(
          const std::string& target_frame,
          const std::string& source_frame,
          const ros::Time& time,
          tf::StampedTransform& transform) const;
  };
}

#endif  // TRANSFORM_UTIL_TRANSFORMER_H_
