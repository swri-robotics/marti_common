// *****************************************************************************
//
// Copyright (c) 2021, Southwest Research Institute® (SwRI®)
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
#ifndef SWRI_ROSCPP_NODE_HANDLE_H_
#define SWRI_ROSCPP_NODE_HANDLE_H_

#include <ros/node_handle.h>

#include <swri_roscpp/subscriber.h>
#include <swri_roscpp/topic_service_client.h>
#include <swri_roscpp/topic_service_server.h>

#include <swri_roscpp/optional_subscriber.h>

#include <marti_common_msgs/NodeInfo.h>

// Macro which adds line number info
#define SWRI_NODE_HANDLE(nh, pnh, description) swri::NodeHandle(nh, pnh, description, __FILE__)

// This is a smart nodehandle that handles storing documentation as well as tracking node names in nodelet managers.

namespace swri
{
class DynamicParameters;
class NodeHandle
{
  friend class DynamicParameters;
  struct NodeHandleInternal
  {
    std::string node_name_;
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;

    // This is additional details about the node
    bool enable_docs_;
    marti_common_msgs::NodeInfo info_msg_;
    ros::Publisher info_pub_;
  };

  boost::shared_ptr<NodeHandleInternal> nh_;

public:

  NodeHandle()
  {
    // we arent valid here
  }

  NodeHandle(ros::NodeHandle nh, ros::NodeHandle pnh,
             const std::string description = "",
             const char* source_file = "")
  {
    // create a new nh
    NodeHandleInternal* inh = new NodeHandleInternal;
    inh->nh_ = nh;
    inh->pnh_ = pnh;
    inh->node_name_ = pnh.getNamespace();
    nh_.reset(inh);

    // read a global parameter indicating if we should advertise debug info
    // this lets people turn it on/off for deployments as it can add a lot of topics
    nh.param("/swridocs", inh->enable_docs_, true);

    if (inh->enable_docs_)
    {
      // create the node specific publisher and initialize message
      inh->info_pub_ = pnh.advertise<marti_common_msgs::NodeInfo>("documentation", 1, true);

      inh->info_msg_.name = inh->node_name_;
      inh->info_msg_.description = description;
      inh->info_msg_.location = source_file;
      std::string nm = ros::NodeHandle("~").getNamespace();
      if (inh->info_msg_.name != nm)
      {
        inh->info_msg_.nodelet_manager = nm;// we are indeed running in a nodelet manager
      }
      inh->info_pub_.publish(inh->info_msg_);// do the initial publish
    }
  }

  operator void*() const { return nh_ ? (void*)1 : (void*)0; }

  // param always uses the private namespace
  inline
  void param(const std::string &name,
      double &variable,
      const double default_value,
      const std::string description = "",
      const bool dynamic = false)
  {
    if (!nh_)
      throw 7;// for now

    //std::string resolved_name = nh_->pnh_.resolveName(name);
    //_used_params.insert(resolved_name);
    nh_->pnh_.param(name, variable, default_value);
    if (!dynamic)
    {
      ROS_INFO("Read parameter %s = %lf", name.c_str(), variable);
    }

    // todo deduplicate
    if (nh_->enable_docs_)
    {
      marti_common_msgs::ParamInfo info;
      info.name = name;
      info.description = description;
      info.resolved_name = nh_->pnh_.resolveName(name);
      info.type = marti_common_msgs::ParamInfo::TYPE_DOUBLE;
      info.default_double = default_value;
      info.dynamic = dynamic;
      nh_->info_msg_.parameters.push_back(info);
      nh_->info_pub_.publish(nh_->info_msg_);
    }
  }

  inline
  void param(const std::string &name,
      float &variable,
      const float default_value,
      const std::string description = "",
      const bool dynamic = false)
  {
    if (!nh_)
      throw 7;// for now

    //std::string resolved_name = nh_->pnh_.resolveName(name);
    //_used_params.insert(resolved_name);
    nh_->pnh_.param(name, variable, default_value);
    if (!dynamic)
    {
      ROS_INFO("Read parameter %s = %lf", name.c_str(), variable);
    }

    // todo deduplicate
    if (nh_->enable_docs_)
    {
      marti_common_msgs::ParamInfo info;
      info.name = name;
      info.description = description;
      info.resolved_name = nh_->pnh_.resolveName(name);
      info.type = marti_common_msgs::ParamInfo::TYPE_DOUBLE;
      info.default_double = default_value;
      info.dynamic = dynamic;
      nh_->info_msg_.parameters.push_back(info);
      nh_->info_pub_.publish(nh_->info_msg_);
    }
  }

  // param always uses the private namespace
  inline
  void param(const std::string &name,
      int &variable,
      const int default_value,
      const std::string description = "",
      const bool dynamic = false)
  {
    if (!nh_)
      throw 7;// for now

    //std::string resolved_name = nh_->pnh_.resolveName(name);
    //_used_params.insert(resolved_name);
    nh_->pnh_.param(name, variable, default_value);
    if (!dynamic)
    {
      ROS_INFO("Read parameter %s = %i", name.c_str(), variable);
    }

    // todo deduplicate
    if (nh_->enable_docs_)
    {
      marti_common_msgs::ParamInfo info;
      info.name = name;
      info.description = description;
      info.resolved_name = nh_->pnh_.resolveName(name);
      info.type = marti_common_msgs::ParamInfo::TYPE_INT;
      info.default_int = default_value;
      info.dynamic = dynamic;
      nh_->info_msg_.parameters.push_back(info);
      nh_->info_pub_.publish(nh_->info_msg_);
    }
  }

  // param always uses the private namespace
  inline
  void param(const std::string &name,
      std::string &variable,
      const std::string default_value,
      const std::string description = "",
      const bool dynamic = false)
  {
    if (!nh_)
      throw 7;// for now

    //std::string resolved_name = nh_->pnh_.resolveName(name);
    //_used_params.insert(resolved_name);
    nh_->pnh_.param(name, variable, default_value);
    if (!dynamic)
    {
      ROS_INFO("Read parameter %s = %s", name.c_str(), variable.c_str());
    }

    // todo deduplicate
    if (nh_->enable_docs_)
    {
      marti_common_msgs::ParamInfo info;
      info.name = name;
      info.description = description;
      info.resolved_name = nh_->pnh_.resolveName(name);
      info.type = marti_common_msgs::ParamInfo::TYPE_STRING;
      info.default_string = default_value;
      info.dynamic = dynamic;
      nh_->info_msg_.parameters.push_back(info);
      nh_->info_pub_.publish(nh_->info_msg_);
    }
  }

  // param always uses the private namespace
  inline
  void param(const std::string &name,
      bool &variable,
      const bool default_value,
      const std::string description = "",
      const bool dynamic = false)
  {
    if (!nh_)
      throw 7;// for now

    //std::string resolved_name = nh_->pnh_.resolveName(name);
    //_used_params.insert(resolved_name);
    nh_->pnh_.param(name, variable, default_value);
    if (!dynamic)
    {
      ROS_INFO("Read parameter %s = %s", name.c_str(), variable ? "true" : "false");
    }

    // todo deduplicate
    if (nh_->enable_docs_)
    {
      marti_common_msgs::ParamInfo info;
      info.name = name;
      info.description = description;
      info.resolved_name = nh_->pnh_.resolveName(name);
      info.type = marti_common_msgs::ParamInfo::TYPE_BOOL;
      info.default_bool = default_value;
      info.dynamic = dynamic;
      nh_->info_msg_.parameters.push_back(info);
      nh_->info_pub_.publish(nh_->info_msg_);
    }
  }

  // Using class method callback.
  template<class M , class T >
  swri::Subscriber subscribe(const std::string &name,
             uint32_t queue_size,
             void(T::*fp)(const boost::shared_ptr< M const > &),
             T *obj,
             const std::string description = "",
             const ros::TransportHints &transport_hints=ros::TransportHints())
  {
    if (!nh_)
      throw 7;// for now

    // todo deduplicate
    if (nh_->enable_docs_)
    {
      const std::string resolved_name = nh_->nh_.resolveName(name);
      marti_common_msgs::TopicInfo info;
      info.name = name;
      info.resolved_name = resolved_name;
      info.message_type = ros::message_traits::DataType<M>().value();
      info.advertised = false;
      info.description = description;
      nh_->info_msg_.topics.push_back(info);
      nh_->info_pub_.publish(nh_->info_msg_);
    }

    return swri::Subscriber(nh_->nh_, name, queue_size, fp, obj, transport_hints);
  }

  template<class M>
  swri::Subscriber subscribe(
             const std::string &name,
             boost::shared_ptr< M const > *dest,
             const ros::TransportHints &transport_hints=ros::TransportHints(),
             const std::string description = "")
  {
    if (!nh_)
      throw 7;// for now

    // todo deduplicate
    if (nh_->enable_docs_)
    {
      const std::string resolved_name = nh_->nh_.resolveName(name);
      marti_common_msgs::TopicInfo info;
      info.name = name;
      info.resolved_name = resolved_name;
      info.message_type = ros::message_traits::DataType<M>().value();
      info.advertised = false;
      info.description = description;
      nh_->info_msg_.topics.push_back(info);
      nh_->info_pub_.publish(nh_->info_msg_);
    }

    return swri::Subscriber(nh_->nh_, name, dest, transport_hints);
  }

  // Using public node handle and class method callback.
  // Only use this for strange things like message filters
  template<class M>
  swri::Subscriber subscribe_later(const std::string &name,
             const std::string description = "")
  {
    if (!nh_)
      throw 7;// for now

    // todo deduplicate
    if (nh_->enable_docs_)
    {
      const std::string resolved_name = nh_->nh_.resolveName(name);
      marti_common_msgs::TopicInfo info;
      info.name = name;
      info.resolved_name = resolved_name;
      info.message_type = ros::message_traits::DataType<M>().value();
      info.advertised = false;
      info.description = description;
      nh_->info_msg_.topics.push_back(info);
      nh_->info_pub_.publish(nh_->info_msg_);
    }
  }

  // Using public node handle and class method callback.
  template<class M , class T >
  swri::OptionalSubscriber subscribe_optional(const std::string &name,
             uint32_t queue_size,
             void(T::*fp)(const boost::shared_ptr< M const > &),
             T *obj,
             const std::string description = "",
             const ros::TransportHints &transport_hints=ros::TransportHints())
  {
    if (!nh_)
      throw 7;// for now

    // todo deduplicate
    if (nh_->enable_docs_)
    {
      const std::string resolved_name = nh_->nh_.resolveName(name);
      marti_common_msgs::TopicInfo info;
      info.name = name;
      info.resolved_name = resolved_name;
      info.message_type = ros::message_traits::DataType<M>().value();
      info.advertised = false;
      info.description = description;
      nh_->info_msg_.topics.push_back(info);
      nh_->info_pub_.publish(nh_->info_msg_);
    }

    return swri::OptionalSubscriber(nh_->nh_, name, queue_size, fp, obj);
  }

  // Uses the public node handle
  template<class MReq, class MRes, class T>
  swri::TopicServiceServer topic_service_server(const std::string &name,
                bool(T::*srv_func)(const MReq &, MRes &),
                T *obj,
                const std::string description = "")
  {
    if (!nh_)
      throw 7;// for now

    // todo deduplicate
    if (nh_->enable_docs_)
    {
      const std::string resolved_name = nh_->nh_.resolveName(name);
      marti_common_msgs::ServiceInfo info;
      info.name = name;
      info.resolved_name = resolved_name;
      info.message_type = ros::message_traits::DataType<MReq>().value();
      info.topic_service = true;
      info.server = true;
      info.description = description;
      nh_->info_msg_.services.push_back(info);
      nh_->info_pub_.publish(nh_->info_msg_);
    }

    swri::TopicServiceServer tss;
    tss.initialize(nh_->nh_, name, srv_func, obj);
    return tss;
  }

  // Advertising uses the public nh
  template<typename M>
  ros::Publisher advertise(
    const std::string name,
    uint32_t queue_size,
    bool latched=false,
    const std::string description = "")
  {
    if (!nh_)
      throw 7;// for now

    const std::string resolved_name = nh_->nh_.resolveName(name);
    ROS_INFO("Publishing [%s] to '%s' from node %s.",
           name.c_str(),
           resolved_name.c_str(),
           nh_->node_name_.c_str());

    // todo deduplicate
    if (nh_->enable_docs_)
    {
      marti_common_msgs::TopicInfo info;
      info.name = name;
      info.resolved_name = resolved_name;
      info.message_type = ros::message_traits::DataType<M>().value();
      info.advertised = true;
      info.description = description;
      nh_->info_msg_.topics.push_back(info);
      nh_->info_pub_.publish(nh_->info_msg_);
    }

    return nh_->nh_.advertise<M>(name, queue_size, latched);
  }
};

inline void param(swri::NodeHandle& nh,
  const std::string name,
  std::string& value,
  const std::string def,
  const std::string description = "")
{
  nh.param(name, value, def, description);
}

template<typename T>
void param(swri::NodeHandle& nh,
  const std::string name,
  T& value,
  const T def)
{
  nh.param(name, value, def, "");
}

template<typename T>
void param(swri::NodeHandle& nh,
  const std::string name,
  T& value,
  const T def,
  const std::string description)
{
  nh.param(name, value, def, description);
}

// some simple utility functions
template<typename M>
ros::Publisher advertise(swri::NodeHandle& nh,
  const std::string name,
  uint32_t queue_size,
  bool latched=false,
  const std::string description = "")
{
  return nh.advertise<M>(name, queue_size, latched, description);
}

// some simple utility functions
// Using class method callback.
template<class M , class T >
swri::Subscriber subscribe(swri::NodeHandle& nh,
           const std::string &name,
           uint32_t queue_size,
           void(T::*fp)(const boost::shared_ptr< M const > &),
           T *obj,
           const std::string description = "",
           const ros::TransportHints &transport_hints=ros::TransportHints())
{
  return nh.subscribe(name, queue_size, fp, obj, description, transport_hints);
}

}  // namespace swri
#endif  // SWRI_ROSCPP_NODE_HANDLE_H_
