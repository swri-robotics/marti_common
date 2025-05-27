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

#include <rclcpp/service.hpp>

#include <swri_roscpp/service_server_statistics.h>

namespace swri
{
class ServiceServer;
class ServiceServerImpl
{
 protected:
  rclcpp::ServiceBase::SharedPtr server_;
  std::string unmapped_service_;

  ServiceServerStatistics total_stats_;
  bool instrument_per_client_;
  std::map<std::string, ServiceServerStatistics> client_stats_;

  bool log_calls_;
  
  void processServing(const std::string caller_name,
                      bool success,
                      const rclcpp::Duration &runtime)
  {
    total_stats_.merge(success, std::chrono::nanoseconds(runtime.nanoseconds()));
    if (instrument_per_client_) {
      client_stats_[caller_name].merge(success, std::chrono::nanoseconds(runtime.nanoseconds()));
    }
  }

 public:
  ServiceServerImpl()
    :
    unmapped_service_("uninitialized"),
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

template<class S, class MReq, class MRes, class T>
class TypedServiceServerImpl : public ServiceServerImpl
{
  T *obj_;
  bool (T::*callback_plain_)(const MReq &, const MRes &);
  rclcpp::Node* nh_;

  void initialize(rclcpp::Node &nh,
                  const std::string &service)
  {
    nh_ = &nh;
    unmapped_service_ = service;

    RCLCPP_INFO(nh_->get_logger(), "Serving to '%s'.", unmapped_service_.c_str());

    server_ = nh.create_service<S>(unmapped_service_,
        std::bind(&TypedServiceServerImpl::handleService,
        this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
  }

  bool handleService(const std::shared_ptr<rmw_request_id_t> request_header,
                     const MReq req, const MRes res)
  {
    std::string caller = "";
    size_t array_len = sizeof(request_header->writer_guid) / sizeof(*request_header->writer_guid);
    for (size_t idx = 0; idx < array_len; idx++)
    {
      caller += std::to_string(request_header->writer_guid[idx]);
    }
    if (logCalls()) {
      RCLCPP_INFO(nh_->get_logger(), "Service '%s' called by '%s'",
               unmapped_service_.c_str(),
               caller.c_str());
    }
    
    bool result;
    std::chrono::system_clock::time_point start = std::chrono::system_clock::now();

    try {
      result = (obj_->*callback_plain_)(req, res);
    } catch (...) {
      result = false;
    }
    std::chrono::system_clock::time_point finish = std::chrono::system_clock::now();

    processServing(caller, result, finish-start);
    return result;
  }

 public:
  TypedServiceServerImpl()
    :
    obj_(NULL),
    callback_plain_(NULL)
  {
  }

  TypedServiceServerImpl(rclcpp::Node &nh,
                         const std::string &service,
                         bool(T::*srv_func)(const MReq &, const MRes &),
                         T *obj)
  {
    obj_ = obj;
    callback_plain_ = srv_func;
    initialize(nh, service);
  }
};  // class TypedServiceServerImpl
}  // namespace swri
#endif  // SWRI_ROSCPP_SERVICE_SERVER_IMPL_H_

