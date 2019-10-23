// *****************************************************************************
//
// Copyright (c) 2019, Southwest Research Institute® (SwRI®)
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
#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <swri_roscpp/service_server.h>

#include <diagnostic_updater/diagnostic_updater.hpp>

#include <std_srvs/srv/empty.hpp>

namespace du = diagnostic_updater;

// Alias type for easier access to DiagnosticStatus enumerations.
typedef diagnostic_msgs::msg::DiagnosticStatus DS;

class ServiceServerTest : public rclcpp::Node
{
  rclcpp::TimerBase::SharedPtr init_timer_;
  rclcpp::TimerBase::SharedPtr diag_timer_;

  du::Updater diagnostic_updater_;
  
  swri::ServiceServer test1_srv_;
  swri::ServiceServer test2_srv_;
  swri::ServiceServer test3_srv_;

  bool test1_result_;
  
 public:
  ServiceServerTest(const std::string& name)
    :
    rclcpp::Node(name),
    diagnostic_updater_(this),
    test1_result_(true)
  {
    // Setup a one-shot timer to initialize the node after a brief
    // delay so that /rosout is always fully initialized.
    RCLCPP_INFO(this->get_logger(), "Starting initialization timer...");
    init_timer_ = this->create_wall_timer(std::chrono::seconds(1),
                                          std::bind(&ServiceServerTest::initialize,
                                          this));
  }

  void initialize()
  {
    test1_srv_.setInstrumentPerClient(true);
    test1_srv_ = swri::ServiceServer::createService<std_srvs::srv::Empty>(*this,
        "test_service1",
        &ServiceServerTest::handleService1,
        this);
    
    
    diagnostic_updater_.setHardwareID("none");
    diagnostic_updater_.add(
      "swri::ServiceServer test service 1", this,
      &ServiceServerTest::service1Diagnostics);
    
    diag_timer_ = this->create_wall_timer(std::chrono::seconds(1),
                                  std::bind(&ServiceServerTest::handleDiagnosticsTimer,
                                  this));
  }

  bool handleService1(const std::shared_ptr<std_srvs::srv::Empty::Request>& req,
                      const std::shared_ptr<std_srvs::srv::Empty::Response>& res)
  {
    RCLCPP_INFO(this->get_logger(), "test service 1 called. returning %s",
             test1_result_ ? "true" : "false");
    return test1_result_;
  }
  
  void handleDiagnosticsTimer()
  {
    diagnostic_updater_.force_update();
  }

  void service1Diagnostics(du::DiagnosticStatusWrapper& status) // NOLINT
  {
    status.summary(DS::OK, "No errors reported.");
    test1_srv_.appendDiagnostics(status, "Test Service 1",
                                 swri::ServiceServer::DIAG_ALL);
  }

  void service2Diagnostics(du::DiagnosticStatusWrapper& status) // NOLINT
  {
    status.summary(DS::OK, "No errors reported.");
    test2_srv_.appendDiagnostics(status, "Test Service 2",
                                 swri::ServiceServer::DIAG_ALL);
  }  

  void service3Diagnostics(du::DiagnosticStatusWrapper& status) // NOLINT
  {
    status.summary(DS::OK, "No errors reported.");
    test3_srv_.appendDiagnostics(status, "Test Service 3",
                                 swri::ServiceServer::DIAG_ALL);
  }  
};  // class ServiceServerTest

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<ServiceServerTest> node = std::make_shared<ServiceServerTest>("service_server_test");
  rclcpp::spin(node);
  
  return 0;  
}
