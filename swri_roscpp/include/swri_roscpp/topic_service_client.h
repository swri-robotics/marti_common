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

#include <rclcpp/rclcpp.hpp>

#include <boost/thread/mutex.hpp>
#include <boost/uuid/random_generator.hpp>
#include <boost/uuid/uuid_io.hpp>

#include <map>

namespace swri
{
template<class MReq, class MRes>
class TopicServiceClientRaw
{
private:
  typedef
  boost::mutex request_lock_;
  std::shared_ptr<rclcpp::Subscription<MRes> > response_sub_;
  std::shared_ptr<rclcpp::Publisher<MReq> > request_pub_;
  std::shared_ptr<MRes> response_;

  rclcpp::Duration timeout_;
  std::string name_;
  std::string service_name_;

  int sequence_;

public:
  TopicServiceClientRaw() :
    timeout_(std::chrono::duration<float>(4.0)),
    sequence_(0),
    node_(nullptr)
  {

  }

  void initialize(rclcpp::Node &nh,
                const std::string &service,
                const std::string &client_name = "")
  {
    node_ = &nh;
    //Converts using string stream instead of to_string so non C++ 11 nodes won't fail
    boost::uuids::random_generator gen;
    boost::uuids::uuid u = gen();
    std::string random_str = boost::uuids::to_string(u);
    name_ = client_name.length() ? client_name : (nh.get_name() + random_str);
    service_name_ = service;

    request_pub_ = nh.create_publisher<MReq>(service + "/request", 10);
    response_sub_ = nh.create_subscription<MRes>(service + "/response",
        10, std::bind(&TopicServiceClientRaw<MReq, MRes>::response_callback, this, std::placeholders::_1));
  }

  bool call(MReq& request, MRes& response)
  {
    boost::mutex::scoped_lock scoped_lock(request_lock_);

    // block for response
    request.srv_header.stamp = node_->now();
    request.srv_header.sequence = sequence_;
    request.srv_header.sender = name_;

    // Wait until we get a subscriber and publisher
    while (request_pub_->get_subscription_count() == 0 || response_sub_->get_publisher_count() == 0)
    {
      rclcpp::sleep_for(std::chrono::milliseconds(2));
      rclcpp::spin_some(node_->shared_from_this());

      if (node_->now() - request.srv_header.stamp > timeout_)
      {
        RCLCPP_ERROR(node_->get_logger(), "Topic service timeout exceeded");
        return false;
      }
    }
    response_.reset();
    request_pub_->publish(request);

    // Wait until we get a response
    while (!response_ && node_->now() - request.srv_header.stamp < timeout_)
    {
      rclcpp::sleep_for(std::chrono::milliseconds(2));
      rclcpp::spin_some(node_->shared_from_this());
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

  std::string getService()
  {
    return service_name_;
  }

  bool exists()
  {
    return request_pub_->get_subscription_count() > 0 && response_sub_->get_publisher_count() > 0;
  }

  // The service server can output a console log message when the
  // service is called if desired.
  void setLogCalls(bool enable);
  bool logCalls() const;
private:

  void response_callback(const std::shared_ptr<MRes> message)
  {
    RCLCPP_DEBUG(node_->get_logger(), "Got response for %s with sequence %i",
             message->srv_header.sender.c_str(), message->srv_header.sequence);

    if (message->srv_header.sender != name_)
    {
      RCLCPP_DEBUG(node_->get_logger(), "Got response from another client, ignoring..");
      return;
    }
    
    if (message->srv_header.sequence != sequence_)
    {
      RCLCPP_WARN(node_->get_logger(), "Got wrong sequence number, ignoring..");
      RCLCPP_DEBUG(node_->get_logger(), "message seq:%i vs current seq: %i", message->srv_header.sequence, sequence_);
      return;
    }

    response_ = message;
  }

  rclcpp::Node* node_;
};  // class TopicServiceClientRaw

template<class MReq>
class TopicServiceClient: public TopicServiceClientRaw<typename MReq:: Request, typename MReq:: Response>
{
public:
  bool call(MReq& req)
  {
    return TopicServiceClientRaw<typename MReq:: Request, typename MReq:: Response>::call(req.request, req.response);
  }

};  // class TopicServiceClient


}  // namespace swri
#endif  // SWRI_ROSCPP_TOPIC_SERVICE_SERVER_H_
