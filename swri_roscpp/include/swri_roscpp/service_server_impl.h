// *****************************************************************************
//
// Copyright (c) 2015, Southwest Research Institute® (SwRI®)
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
#ifndef SWRI_ROSCPP_SERVICE_SERVER_IMPL_H_
#define SWRI_ROSCPP_SERVICE_SERVER_IMPL_H_

#include <swri_roscpp/service_server_statistics.h>

namespace swri
{
class ServiceServer;
class ServiceServerImpl
{
 protected:
  ros::ServiceServer server_;
  std::string unmapped_service_;
  std::string mapped_service_;

  ServiceServerStatistics total_stats_;
  bool instrument_per_client_;
  std::map<std::string, ServiceServerStatistics> client_stats_;

  bool log_calls_;
  
  void processServing(const std::string caller_name,
                      bool success,
                      const ros::WallDuration &runtime)
  {
    total_stats_.merge(success, runtime);
    if (instrument_per_client_) {
      client_stats_[caller_name].merge(success, runtime);
    }
  }

 public:
  ServiceServerImpl()
    :
    unmapped_service_("uninitialized"),
    mapped_service_("initialized"),
    instrument_per_client_(false),
    log_calls_(false)
  {
  }

  void resetStatistics()
  {
    total_stats_.reset();
    client_stats_.clear();
  }

  const std::string& unmappedService() const { return unmapped_service_; }
  const std::string& mappedService() const { return mapped_service_; }

  const ServiceServerStatistics& totalStats() const { return total_stats_; }

  void setInstrumentPerClient(bool enable)
  {
    instrument_per_client_ = enable;
    if (!instrument_per_client_) {
      client_stats_.clear();
    }
  }

  bool instrumentPerClient() const { return instrument_per_client_; }

  std::vector<std::string> clientNames() const
  {
    std::vector<std::string> names;
    names.reserve(client_stats_.size());

    std::map<std::string, ServiceServerStatistics>::const_iterator it;
    for (it = client_stats_.begin(); it != client_stats_.end(); it++) {
      names.push_back(it->first);
    }
    return names;
  }

  ServiceServerStatistics clientStatistics(
    const std::string &name) const
  {
    std::map<std::string, ServiceServerStatistics>::const_iterator it;
    it = client_stats_.find(name);
    if (it == client_stats_.end()) {
      return ServiceServerStatistics();
    } else {
      return it->second;
    }
  }

  void setLogCalls(bool enable)
  {
    log_calls_ = enable;
  }

  bool logCalls() const { return log_calls_; }
};

template<class MReq, class MRes, class T>
class TypedServiceServerImpl : public ServiceServerImpl
{
  T *obj_;
  bool (T::*callback_plain_)(MReq &, MRes &);
  bool (T::*callback_with_event_)(ros::ServiceEvent< MReq, MRes > &);
  bool (T::*callback_with_name_)(const std::string &, const MReq &, MRes &);

  void initialize(ros::NodeHandle &nh,
                  const std::string &service)
  {
    unmapped_service_ = service;
    mapped_service_ = nh.resolveName(service);

    if (unmapped_service_ == mapped_service_) {
      ROS_INFO("Serving to '%s'.", mapped_service_.c_str());
    } else {
      ROS_INFO("Serving to '%s' at '%s'.",
               unmapped_service_.c_str(),
               mapped_service_.c_str());
    }

    server_ = nh.advertiseService(mapped_service_,
                                  &TypedServiceServerImpl::handleService,
                                  this);
  }

  bool handleService(ros::ServiceEvent<MReq, MRes> &event)
  {
    if (logCalls()) {
      ROS_INFO("Service '%s' called by '%s'",
               mapped_service_.c_str(),
               event.getCallerName().c_str());
    }
    
    bool result;
    ros::WallTime start = ros::WallTime::now();

    try {
      if (callback_plain_) {
        result = (obj_->*callback_plain_)(
          const_cast<MReq&>(event.getRequest()), event.getResponse());
      } else if (callback_with_name_) {
        result = (obj_->*callback_with_name_)(
          event.getCallerName(), event.getRequest(), event.getResponse());
      } else {
        result = (obj_->*callback_with_event_)(event);
      }
    } catch (...) {
      result = false;
    }
    ros::WallTime finish = ros::WallTime::now();

    processServing(event.getCallerName(), result, finish-start);
    return result;
  }

 public:
  TypedServiceServerImpl()
    :
    obj_(NULL),
    callback_plain_(NULL),
    callback_with_event_(NULL),
    callback_with_name_(NULL)
  {
  }

  TypedServiceServerImpl(ros::NodeHandle &nh,
                         const std::string &service,
                         bool(T::*srv_func)(MReq &, MRes &),
                         T *obj)
  {
    obj_ = obj;
    callback_plain_ = srv_func;
    callback_with_event_ = NULL;
    callback_with_name_ = NULL;
    initialize(nh, service);
  }

  TypedServiceServerImpl(ros::NodeHandle &nh,
                         const std::string &service,
                         bool(T::*srv_func)(ros::ServiceEvent<MReq, MRes> &),
                         T *obj)
  {
    obj_ = obj;
    callback_plain_ = NULL;
    callback_with_event_ = srv_func;
    callback_with_name_ = NULL;
    initialize(nh, service);
  }

  TypedServiceServerImpl(ros::NodeHandle &nh,
                         const std::string &service,
                         bool(T::*srv_func)(const std::string &, const MReq &, MRes &),
                         T *obj)
  {
    obj_ = obj;
    callback_plain_ = NULL;
    callback_with_event_ = NULL;
    callback_with_name_ = srv_func;
    initialize(nh, service);
  }
};  // class TypedServiceServerImpl
}  // namespace swri
#endif  // SWRI_ROSCPP_SERVICE_SERVER_IMPL_H_

