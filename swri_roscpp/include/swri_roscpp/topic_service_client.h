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
#include <ros/this_node.h>

#include <boost/thread/mutex.hpp>
#include <boost/uuid/random_generator.hpp>
#include <boost/uuid/uuid_io.hpp>

#include <map>

namespace swri
{
template<class MReq, class MRes>
class TopicServiceClientImpl
{
public:
  typedef
  boost::mutex request_lock_;
  ros::Subscriber response_sub_;
  ros::Publisher request_pub_;
  boost::shared_ptr<MRes> response_;

  ros::Duration timeout_;
  std::string name_;
  std::string service_name_;

  int sequence_;

  TopicServiceClientImpl() : sequence_(0), timeout_(ros::Duration(4.0))
  {

  }

  void initialize(ros::NodeHandle &nh,
                const std::string &service,
                const std::string &client_name = "")
  {
    //Converts using string stream instead of to_string so non C++ 11 nodes won't fail
    boost::uuids::random_generator gen;
    boost::uuids::uuid u = gen();
    std::string random_str = boost::uuids::to_string(u);
    name_ = client_name.length() ? client_name : (ros::this_node::getName() + random_str);
    std::string rservice = nh.resolveName(service);
    service_name_ = rservice;

    request_pub_ = nh.advertise<MReq>(rservice + "/request", 10);
    response_sub_ = nh.subscribe(rservice + "/response", 10, &TopicServiceClientImpl<MReq, MRes>::response_callback, this);
  }

  bool call(MReq& request, MRes& response)
  {
    boost::mutex::scoped_lock scoped_lock(request_lock_);

    // block for response
    request.srv_header.stamp = ros::Time::now();
    request.srv_header.sequence = sequence_;
    request.srv_header.sender = name_;

    // Wait until we get a subscriber and publisher
    while (request_pub_.getNumSubscribers() == 0 || response_sub_.getNumPublishers() == 0)
    {
      ros::Duration(0.002).sleep();
      ros::spinOnce();

      if (ros::Time::now() - request.srv_header.stamp > timeout_)
      {
        ROS_ERROR("Topic service timeout exceeded");
        return false;
      }
    }
    response_.reset();
    request_pub_.publish(request);

    // Wait until we get a response
    while (!response_ && ros::Time::now() - request.srv_header.stamp < timeout_)
    {
      ros::Duration(0.002).sleep();
      ros::spinOnce();
    }

    sequence_++;
    if (response_)
    {
      response = *response_;
      response_.reset();
      return response.srv_header.result;
    }
    return false;
  }

private:

  void response_callback(const boost::shared_ptr<MRes>& message)
  {
    ROS_DEBUG("Got response for %s with sequence %i",
             message->srv_header.sender.c_str(), message->srv_header.sequence);

    if (message->srv_header.sender != name_)
    {
      ROS_DEBUG("Got response from another client, ignoring..");
      return;
    }
    
    if (message->srv_header.sequence != sequence_)
    {
      ROS_WARN("Got wrong sequence number, ignoring..");
      ROS_DEBUG("message seq:%i vs current seq: %i", message->srv_header.sequence, sequence_);
      return;
    }

    response_ = message;
  }
};  // class TopicServiceClientImpl

template<class MReq>
class TopicServiceClient 
{
  boost::shared_ptr<TopicServiceClientImpl<typename MReq:: Request, typename MReq:: Response> > impl_;

public:

  void initialize(ros::NodeHandle &nh,
                const std::string &service,
                const std::string &client_name = "")
  {
    impl_ = boost::shared_ptr<TopicServiceClientImpl<typename MReq:: Request, typename MReq:: Response> >(
      new TopicServiceClientImpl<typename MReq:: Request, typename MReq:: Response>());

    impl_->initialize(nh, service, client_name);
  }

  std::string getService()
  {
    return impl_->service_name_;
  }

  bool exists()
  {
    return impl_->request_pub_.getNumSubscribers() > 0 && impl_->response_sub_.getNumPublishers() > 0;
  }

  bool call(MReq& req)
  {
    return impl_->call(req.request, req.response);
  }
};  // class TopicServiceClient


}  // namespace swri
#endif  // SWRI_ROSCPP_TOPIC_SERVICE_SERVER_H_
