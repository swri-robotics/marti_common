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

#include <ros/node_handle.h>

namespace swri
{
class ImplRoot
{
public:
  virtual ~ImplRoot()
  {

  }
};

template<class MReq, class MRes, class T>
class TopicServiceServerImpl: public ImplRoot
{
  ros::Subscriber request_sub_;
  ros::Publisher response_pub_;

  bool(T::*callback_)(const MReq &, MRes &);
  T* obj_;

public:
  void initialize(ros::NodeHandle &nh,
              const std::string &service,
              bool(T::*srv_func)(const MReq &, MRes &),
              T *obj)
  {
    callback_ = srv_func;
    obj_ = obj;

    ros::NodeHandle pnh("~");

    std::string rservice = nh.resolveName(service);

    response_pub_ = nh.advertise<MRes>(rservice + "/response", 10);
    request_sub_ = nh.subscribe(rservice + "/request", 10, &TopicServiceServerImpl<MReq, MRes, T>::request_callback, this);
  }

private:

  void request_callback(const MReq& message)
  {
    ROS_DEBUG("Got request from %s with sequence %i", message.srv_header.sender.c_str(), message.srv_header.sequence);

    MRes response;

    bool result = (obj_->*callback_)(message, response);
    response.srv_header.result = result;
    response.srv_header.sequence = message.srv_header.sequence;
    response.srv_header.sender = message.srv_header.sender;
    response_pub_.publish(response);
  }
};

class TopicServiceServer
{
 private:

  boost::shared_ptr<ImplRoot> impl_;

 public:
  TopicServiceServer();

  template<class MReq, class MRes, class T>
  void initialize(ros::NodeHandle &nh,
                const std::string &service,
                bool(T::*srv_func)(const MReq &, MRes &),
                T *obj);
};  // class TopicServiceServer

inline
TopicServiceServer::TopicServiceServer()
{

}

template<class MReq, class MRes, class T>
inline
void TopicServiceServer::initialize(ros::NodeHandle &nh,
                          const std::string &service,
                          bool(T::*srv_func)(const MReq &, MRes &),
                          T *obj)
{
  TopicServiceServerImpl<MReq, MRes, T>* impl = new TopicServiceServerImpl<MReq, MRes, T>();
  impl->initialize(nh, service, srv_func, obj);
  impl_ = boost::shared_ptr<ImplRoot>(impl);
}

}  // namespace swri
#endif  // SWRI_ROSCPP_TOPIC_SERVICE_SERVER_H_
