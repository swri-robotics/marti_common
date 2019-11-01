// *****************************************************************************
//
// Copyright (c) 2018, Southwest Research Institute® (SwRI®)
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
// ARE DISCLAIMED. IN NO EVENT SHALL SOUTHWEST RESEARCH INSTITUTE BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// *****************************************************************************
#ifndef SWRI_ROSCPP_TOPIC_SERVICE_SERVER_H_
#define SWRI_ROSCPP_TOPIC_SERVICE_SERVER_H_

#include <rclcpp/rclcpp.hpp>

namespace swri
{
class ImplRoot
{
public:
  virtual ~ImplRoot() = default;
};

template<class MReq, class MRes, class T>
class TopicServiceServerImpl: public ImplRoot
{
  std::shared_ptr<rclcpp::Subscription<MReq> > request_sub_;
  std::shared_ptr<rclcpp::Publisher<MRes> > response_pub_;

  bool(T::*callback_)(const MReq &, MRes &);
  T* obj_;

public:
  explicit TopicServiceServerImpl() :
    ImplRoot(),
    node_(nullptr)
  {}

  void initialize(rclcpp::Node &nh,
              const std::string &service,
              bool(T::*srv_func)(const MReq &, MRes &),
              T *obj)
  {
    node_ = &nh;
    callback_ = srv_func;
    obj_ = obj;

    response_pub_ = nh.create_publisher<MRes>(service + "/response", 10);
    request_sub_ = nh.create_subscription<MReq>(service + "/request",
        10, std::bind(&TopicServiceServerImpl<MReq, MRes, T>::request_callback, this, std::placeholders::_1));
  }

private:

  void request_callback(const std::shared_ptr<MReq> message)
  {
    RCLCPP_DEBUG(node_->get_logger(),
        "Got request from %s with sequence %i", message->srv_header.sender.c_str(), message->srv_header.sequence);

    MRes response;

    bool result = (obj_->*callback_)(*message, response);
    response.srv_header.result = result;
    response.srv_header.sequence = message->srv_header.sequence;
    response.srv_header.sender = message->srv_header.sender;
    response_pub_->publish(response);
  }

  rclcpp::Node* node_;
};

class TopicServiceServer
{
 private:

  std::shared_ptr<ImplRoot> impl_;

 public:
  TopicServiceServer() = default;

  template<class MReq, class MRes, class T>
  void initialize(rclcpp::Node &nh,
                const std::string &service,
                bool(T::*srv_func)(const MReq&, MRes&),
                T *obj);
};  // class TopicServiceServer

template<class MReq, class MRes, class T>
inline
void TopicServiceServer::initialize(rclcpp::Node &nh,
                          const std::string &service,
                          bool(T::*srv_func)(const MReq&, MRes&),
                          T *obj)
{
  auto* impl = new TopicServiceServerImpl<MReq, MRes, T>();
  impl->initialize(nh, service, srv_func, obj);
  impl_ = std::shared_ptr<ImplRoot>(impl);
}

}  // namespace swri
#endif  // SWRI_ROSCPP_TOPIC_SERVICE_SERVER_H_
