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
#include <rclcpp/rclcpp.hpp>
#include <swri_roscpp/subscriber.h>
#include <boost/bind/bind.hpp>

#include <diagnostic_updater/diagnostic_updater.hpp>

#include <std_msgs/msg/float32.hpp>
#include <nav_msgs/msg/odometry.hpp>

namespace du = diagnostic_updater;

// Alias type for easier access to DiagnosticStatus enumerations.
typedef diagnostic_msgs::msg::DiagnosticStatus DS;

class SubscriberTest : public rclcpp::Node
{
  rclcpp::TimerBase::SharedPtr init_timer_;
  rclcpp::TimerBase::SharedPtr diag_timer_;

  du::Updater diagnostic_updater_;
  
  swri::Subscriber sub_;
  
 public:
  SubscriberTest(const std::string& name) :
    rclcpp::Node(name),
    diagnostic_updater_(this)
  {
    // Setup a one-shot timer to initialize the node after a brief
    // delay so that /rosout is always fully initialized.
    RCLCPP_INFO(this->get_logger(), "Starting initialization timer...");
    init_timer_ = this->create_wall_timer(std::chrono::seconds(1),
                                      std::bind(&SubscriberTest::initialize,
                                      this));
  }

  void initialize()
  {
    // Example to subscribe to a class method.
    sub_ = swri::Subscriber(*this, "odom", 100, &SubscriberTest::handleMessage, this);

    // Example to subscribe using a boost function.  I am unable to
    // figure out how to get the compiler to infer the type, so I have
    // to declare the function first and then pass that.
    std::function<void(const std::shared_ptr< nav_msgs::msg::Odometry const> &)> callback =
      boost::bind(&SubscriberTest::handleMessage, this, boost::placeholders::_1);
    sub_ = swri::Subscriber(*this, "odom", 100, callback);

    sub_.setTimeout(rclcpp::Duration(1, 0));

    diagnostic_updater_.setHardwareID("none");
    diagnostic_updater_.add(
      "swri::Subscriber test (manual diagnostics)", this,
      &SubscriberTest::manualDiagnostics);

    diagnostic_updater_.add(
      "swri::Subscriber test (auto diagnostics)", this,
      &SubscriberTest::autoDiagnostics);
    
    diag_timer_ = this->create_wall_timer(std::chrono::seconds(1),
                                  std::bind(&SubscriberTest::handleDiagnosticsTimer,
                                  this));
  }

  void handleMessage(const nav_msgs::msg::Odometry::ConstSharedPtr &msg)
  {
    // Suppresses unused parameter warning
    (void)msg;
    RCLCPP_INFO_ONCE(this->get_logger(), "Receiving messages");
  }

  void handleDiagnosticsTimer()
  {
    diagnostic_updater_.force_update();
  }

  void manualDiagnostics(du::DiagnosticStatusWrapper& status) // NOLINT
  {
    status.summary(DS::OK, "No errors reported.");

    // The swri::Subscriber provides methods to access all of the
    // statistics and status information.  These are suitable for
    // making control decisions or report diagnostics.  In this
    // example, we access the data directly to update a diagnostic
    // status on our own.  This is useful if you want to apply custom
    // summary semantics or change how the key/values are included.
    
    if (sub_.messageCount() == 0) {
      status.mergeSummary(DS::WARN, "No messages received.");
    } else if (sub_.inTimeout()) {
      status.mergeSummary(DS::ERROR, "Topic has timed out.");
    } else if (sub_.timeoutCount() > 0) {
      status.mergeSummary(DS::WARN, "Timeouts have occurred.");
    }

    status.addf("Topic Name", "%s", sub_.unmappedTopic().c_str());
    status.addf("Number of publishers", "%d", sub_.numPublishers());
    
    status.addf("Mean Latency [us]", "%f", sub_.meanLatencyMicroseconds());
    status.addf("Min Latency [us]", "%f", sub_.minLatencyMicroseconds());
    status.addf("Max Latency [us]", "%f", sub_.maxLatencyMicroseconds());
    
    status.addf("Mean Frequency [Hz]", "%f", sub_.meanFrequencyHz());
    status.addf("Mean Period [ms]", "%f", sub_.meanPeriodMilliseconds());
    status.addf("Min Period [ms]", "%f", sub_.minPeriodMilliseconds());
    status.addf("Max Period [ms]", "%f", sub_.maxPeriodMilliseconds());

    if (sub_.timeoutEnabled()) {
      status.addf("Timed Out Count", "%d [> %f ms]",
                  sub_.timeoutCount(),
                  sub_.timeoutMilliseconds());
    } else {
      status.add("Timed Out Count", "N/A");
    }
  }

  void autoDiagnostics(du::DiagnosticStatusWrapper& status) // NOLINT
  {
    status.summary(DS::OK, "No errors reported.");

    // In this example, we use swri::Subscriber's appendDiagnostics
    // method to include diagnostics using a common format and common
    // summary semantics.  If we didn't care about specific bits of
    // information, we can leave them out by removing specific flags.
    sub_.appendDiagnostics(status, "Odometry",
                           swri::Subscriber::DIAG_CONNECTION |
                           swri::Subscriber::DIAG_MSG_COUNT |
                           swri::Subscriber::DIAG_TIMEOUT |
                           swri::Subscriber::DIAG_LATENCY |
                           swri::Subscriber::DIAG_RATE);
  }
};  // class SubscriberTest

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<SubscriberTest> node = std::make_shared<SubscriberTest>("subscriber_test");
  
  rclcpp::spin(node);
  
  return 0;  
}
