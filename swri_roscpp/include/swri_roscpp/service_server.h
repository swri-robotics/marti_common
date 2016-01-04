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
#ifndef SWRI_ROSCPP_SERVICE_SERVER_H_
#define SWRI_ROSCPP_SERVICE_SERVER_H_

#include <ros/node_handle.h>
#include <diagnostic_updater/DiagnosticStatusWrapper.h>
#include <swri_roscpp/service_server_impl.h>
#include <swri_roscpp/service_server_statistics.h>

namespace swri
{
class ServiceServer
{
 private:
  boost::shared_ptr<ServiceServerImpl> impl_;

 public:
  ServiceServer();

  template<class MReq, class MRes, class T>
  ServiceServer(ros::NodeHandle &nh,
                const std::string &service,
                bool(T::*srv_func)(MReq &, MRes &),
                T *obj);

  template<class MReq, class MRes, class T>
  ServiceServer(ros::NodeHandle &nh,
                const std::string &service,
                bool(T::*srv_func)(ros::ServiceEvent< MReq, MRes > &),                
                T *obj);

  template<class MReq, class MRes, class T>
  ServiceServer(ros::NodeHandle &nh,
                const std::string &service,
                bool(T::*srv_func)(const std::string &, const MReq &, MRes &),
                T *obj);
  
  ServiceServer& operator=(const ServiceServer &other);

  // Reset all statistics, including message and timeout counts.
  void resetStatistics();
  
  // Returns the unmapped service name that was provided when the
  // service was created.
  const std::string& unmappedService() const;
  // Returns the fully mapped service name that the server is
  // listening on.
  const std::string& mappedService() const;

  // Retrieve the statitics for all service calls handled by the
  // server.
  const ServiceServerStatistics& statistics() const;  

  // The service statistics can be broken down to individual clients
  // if desired.
  void setInstrumentPerClient(bool enable);
  bool instrumentPerClient() const;
  std::vector<std::string> clientNames() const;
  ServiceServerStatistics clientStatistics(const std::string &name) const;

  // The service server can output a console log message when the
  // service is called if desired.
  void setLogCalls(bool enable);
  bool logCalls() const;
  
  // These flags determine which values are added to a diagnostic
  // status by the appendDiagnostics method.
  enum DIAGNOSTIC_FLAGS {
    DIAG_CONNECTION = 1 << 0,  // Include service names
    DIAG_SERVINGS   = 1 << 1,   // Servings counts/success rates
    DIAG_TIMING     = 1 << 2,  // Include service time
    DIAG_CLIENTS    = 1 << 3,  // Include client list, if applicable.
    
    DIAG_ALL        = ~0,       // Abbreviation to include all information
    DIAG_MOST       = DIAG_ALL ^ DIAG_CONNECTION
    // Abbreviation to include everything except connection info.
  };
  
  // This is a convenience method to add information about this
  // server to a diagnostic status in a standard way.
  //
  // The status is merged with a warning if service calls have failed.
  // The status is merged with an error if the most recent service
  // call failed.
  //
  // The flags parameter determines which values are added to the
  // status' key/value pairs.  This should be a bitwise combination of
  // the values defined in DIAGNOSTIC_FLAGS.
  void appendDiagnostics(diagnostic_updater::DiagnosticStatusWrapper &status,
                         const std::string &name,
                         const int flags);
};  // class ServiceServer

inline
ServiceServer::ServiceServer()
{
  // Setup an empty implementation so that we can assume impl_ is
  // non-null and avoid a lot of unnecessary NULL checks.
  impl_ = boost::make_shared<ServiceServerImpl>();
}

template<class MReq, class MRes, class T>
inline
ServiceServer::ServiceServer(ros::NodeHandle &nh,
                             const std::string &service,
                             bool(T::*srv_func)(MReq &, MRes &),
                             T *obj)
{
  impl_ = boost::shared_ptr<ServiceServerImpl>(
    new TypedServiceServerImpl<MReq, MRes, T>(
      nh, service, srv_func, obj));
}

template<class MReq, class MRes, class T>
inline
ServiceServer::ServiceServer(ros::NodeHandle &nh,
                             const std::string &service,
                             bool(T::*srv_func)(ros::ServiceEvent< MReq, MRes > &),                
                             T *obj)
{
  impl_ = boost::shared_ptr<ServiceServerImpl>(
    new TypedServiceServerImpl<MReq, MRes, T>(
      nh, service, srv_func, obj));
}

template<class MReq, class MRes, class T>
inline
ServiceServer::ServiceServer(ros::NodeHandle &nh,
                             const std::string &service,
                             bool(T::*srv_func)(const std::string &, const MReq &, MRes &),
                             T *obj)
{
  impl_ = boost::shared_ptr<ServiceServerImpl>(
    new TypedServiceServerImpl<MReq, MRes, T>(
      nh, service, srv_func, obj));
}

inline
ServiceServer& ServiceServer::operator=(const ServiceServer &other)
{
  bool instrument_per_client = impl_->instrumentPerClient();
  bool log_calls = impl_->logCalls();
  impl_ = other.impl_;
  impl_->setInstrumentPerClient(instrument_per_client);
  impl_->setLogCalls(log_calls);
  return *this;
}

inline
void ServiceServer::resetStatistics()
{
  impl_->resetStatistics();
}

inline
const std::string& ServiceServer::unmappedService() const
{
  return impl_->unmappedService();
}

inline
const std::string& ServiceServer::mappedService() const
{
  return impl_->mappedService();
}

inline
const ServiceServerStatistics& ServiceServer::statistics() const
{
  return impl_->totalStats();
}
   
inline
void ServiceServer::setInstrumentPerClient(bool enable)
{
  impl_->setInstrumentPerClient(enable);
}

inline
bool ServiceServer::instrumentPerClient() const
{
  return impl_->instrumentPerClient();
}

inline
std::vector<std::string> ServiceServer::clientNames() const
{
  return impl_->clientNames();
}

inline
ServiceServerStatistics ServiceServer::clientStatistics(
  const std::string &name) const
{
  return impl_->clientStatistics(name);
}

inline
void ServiceServer::setLogCalls(bool enable)
{
  impl_->setLogCalls(enable);
}

inline
bool ServiceServer::logCalls() const
{
  return impl_->logCalls();
}

inline
void ServiceServer::appendDiagnostics(
  diagnostic_updater::DiagnosticStatusWrapper &status,
  const std::string &name,
  const int flags)
{
  const ServiceServerStatistics& stats = statistics();

  // Alias a type for easier access to DiagnosticStatus enumerations.
  typedef diagnostic_msgs::DiagnosticStatus DS;

  if (stats.lastFailed()) {
    status.mergeSummaryf(DS::ERROR, "Last %s service called failed", name.c_str());
  } else if (stats.failed()) {
    status.mergeSummaryf(DS::ERROR, "%s service calls have failed.", name.c_str());
  }

  if (flags & DIAG_CONNECTION) {
    if (mappedService() == unmappedService()) {
      status.addf(name + "service name", "%s", mappedService().c_str());
    } else {
      status.addf(name + "service name", "%s -> %s",
                  unmappedService().c_str(),
                  mappedService().c_str());
    }
  }

  if (flags & DIAG_SERVINGS) {
    status.addf(name + "service calls", "%d failed / %d calls",
                stats.failed(), stats.servings());                
  }

  if (flags & DIAG_TIMING) {
    status.addf(name + "service call time", "%.2f [s] (%.2f [s] - %.2f [s])",
                stats.meanTime().toSec(),
                stats.minTime().toSec(),
                stats.maxTime().toSec());
  }

  if (flags & DIAG_CLIENTS) {
    std::vector<std::string> names = clientNames();

    for (size_t i = 0; i < names.size(); i++) {
      ServiceServerStatistics client_stats = clientStatistics(names[i]);
      
      if (flags & DIAG_SERVINGS) {
        status.addf(name + " calls from " + names[i], "%d failed / %d calls",
                    client_stats.failed(),
                    client_stats.servings());
      }

      if (flags & DIAG_TIMING) {
        status.addf(name + " calls from " + names[i] + " time", "%.2f [s] (%.2f [s] - %.2f [s])",
                client_stats.meanTime().toSec(),
                client_stats.minTime().toSec(),
                client_stats.maxTime().toSec());
      }
    }
  }
}
}  // namespace swri
#endif  // SWRI_ROSCPP_SERVICE_SERVER_H_
