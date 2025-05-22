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

#include <swri_transform_util/transform.h>

namespace swri_transform_util
{
  Transform::Transform() :
    transform_(std::make_shared<IdentityTransform>())
  {
  }

  Transform::Transform(const tf2::Transform& transform) :
    transform_(std::make_shared<TfTransform>(transform))
  {
  }
  
  Transform::Transform(const tf2::Stamped<tf2::Transform>& transform) :
    transform_(std::make_shared<TfTransform>(transform))
  {
  }

  Transform::Transform(std::shared_ptr<TransformImpl> transform) :
    transform_(transform)
  {
  }

  Transform& Transform::operator=(const tf2::Transform transform)
  {
    transform_ = std::make_shared<TfTransform>(transform);

    return *this;
  }

  Transform& Transform::operator=(std::shared_ptr<TransformImpl> transform)
  {
    transform_ = transform;

    return *this;
  }

  tf2::Vector3 Transform::operator()(const tf2::Vector3& v) const
  {
    tf2::Vector3 transformed;

    transform_->Transform(v, transformed);

    return transformed;
  }

  tf2::Vector3 Transform::operator*(const tf2::Vector3& v) const
  {
    tf2::Vector3 transformed;

    transform_->Transform(v, transformed);

    return transformed;
  }
  
  tf2::Quaternion Transform::operator*(const tf2::Quaternion& q) const
  {
    return q * GetOrientation();
  }

  tf2::Vector3 Transform::GetOrigin() const
  {
    tf2::Vector3 origin;

    transform_->Transform(tf2::Vector3(0, 0, 0), origin);

    return origin;
  }

  tf2::Quaternion Transform::GetOrientation() const
  {
    return transform_->GetOrientation();
  }

  Transform Transform::Inverse() const
  {
    return Transform(transform_->Inverse());
  }

  tf2::Transform Transform::GetTF() const
  {
    return tf2::Transform(GetOrientation(),GetOrigin());
  }

  void IdentityTransform::Transform(const tf2::Vector3& v_in, tf2::Vector3& v_out) const
  {
    v_out = v_in;
  }
  
  std::shared_ptr<TransformImpl> IdentityTransform::Inverse() const
  {
    TransformImplPtr inverse = 
        std::make_shared<IdentityTransform>();
    inverse->SetStamp(stamp_);
    return inverse;
  }

  TfTransform::TfTransform(const tf2::Transform& transform) :
    transform_(transform)
  {
    Tf2StampStampInterface::SetStamp(std::chrono::system_clock::now());
  }
  
  TfTransform::TfTransform(const tf2::Stamped<tf2::Transform>& transform) :
    transform_(transform)
  {
    Tf2StampStampInterface::SetStamp(transform.stamp_);
  }

  void TfTransform::Transform(const tf2::Vector3& v_in, tf2::Vector3& v_out) const
  {
    v_out = transform_ * v_in;
  }
  
  tf2::Quaternion TfTransform::GetOrientation() const
  {
    return transform_.getRotation();
  }

  TransformImplPtr TfTransform::Inverse() const
  {
    TransformImplPtr inverse = 
        std::make_shared<TfTransform>(transform_.inverse());
    inverse->SetStamp(stamp_);
    return inverse;
  }
}
