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
#ifndef SWRI_ROSCPP_TOPIC_SERVICE_CLIENT_H_
#define SWRI_ROSCPP_TOPIC_SERVICE_CLIENT_H_

#include <ros/node_handle.h>

#include <boost/thread/mutex.hpp>

#include <map>

namespace swri
{
template<class MReq, class MRes>
class TopicServiceClient
{
 private:
  boost::mutex request_lock_;
  ros::Subscriber response_sub_;
  ros::Publisher request_pub_;
  MRes::SharedPtr response_;

  ros::Duration timeout_;

  std::map<int, boost::function<void(const MReq&, const MRes&)>& fun> callbacks_;

  int sequence_;

  boost::function<bool(const MReq &, MRes &)> callback_;

 public:
  TopicServiceClient();

  initialize(ros::NodeHandle &nh,
                const std::string &service,
                const std::string &client_name = "");

  bool call(MReq& req, MRes& response);

  bool call_async(MReq& request, boost::function<void(const MReq&, const MRes&)>& fun);

  // The service server can output a console log message when the
  // service is called if desired.
  void setLogCalls(bool enable);
  bool logCalls() const;
private:

  void response_callback(const MRes::SharedPtr& message)
  {
    ROS_INFO("Got response for %s with sequence %i", message->header.sender.c_str(), message->header.sequence);

    if (MRes->header.sender != name_)
    {
      ROS_INFO("Got response from another client, ignoring..");
      return;
    }

    if (MRes->header.sequence != sequence_)
    {
      ROS_INFO("Got wrong sequence number, ignoring..");
      return;
    }

    response_ = message;
  }
};  // class TopicServiceClient

inline
TopicServiceClient::TopicServiceClient() : sequence_(0), timeout_(ros::Duration(4.0))
{

}

inline
TopicServiceClient::initialize(ros::NodeHandle &nh,
                          const std::string &service,
                          const std::string &client_name)
{
  name_ = client_name.length() ? client_name : nh.getNamespace();
  response_sub_ = nh.subscribe(service + "/request", 10, response_callback, this);
  request_pub_ = nh.advertise(service + "/response", 10);
}

inline
bool TopicServiceClient::call(MReq& request, MRes& response)
{
  boost::mutex::scoped_lock scoped_lock(request_lock_);

  // block for response
  request.header.stamp = ros::Time::now();
  request.header.sequence = sequence_;
  request.header.sender = name_;
  request_pub_.send(request);

  // Wait until we get a response
  while (!response_ && ros::Time::now() - request.header.stamp < timeout_)
  {
    ros::Duration(0.002).sleep();
    ros::spinOnce();
  }

  sequence_++;
  if (response_)
  {
    response = *response_;
    response_.reset();
    return response_.result;
  }
  return false;
}

inline
bool TopicServiceClient::call_async(MReq& request, boost::function<void(const MReq&, const MRes&)>& fun)
{

}

}  // namespace swri
#endif  // SWRI_ROSCPP_TOPIC_SERVICE_SERVER_H_
