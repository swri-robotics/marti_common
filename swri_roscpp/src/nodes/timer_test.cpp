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
#include <swri_roscpp/timer.h>

#include <diagnostic_updater/diagnostic_updater.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>

namespace du = diagnostic_updater;

// Alias type for easier access to DiagnosticStatus enumerations.
typedef diagnostic_msgs::msg::DiagnosticStatus DS;

class TimerTest : public rclcpp::Node
{
  rclcpp::TimerBase::SharedPtr init_timer_;
  swri::Timer update_timer_;
  swri::Timer diag_timer_;

  std::shared_ptr<du::Updater> diagnostic_updater_;
  
 public:
  TimerTest(const std::string& name) :
    rclcpp::Node(name)
  {
    this->declare_parameter("fibonacci_index", 30);
    diagnostic_updater_ = std::make_shared<du::Updater>(this);
    // Setup a one-shot timer to initialize the node after a brief
    // delay so that /rosout is always fully initialized.
    RCLCPP_INFO(this->get_logger(), "Starting initialization timer...");
    auto initialize_callback = [this]() -> void {this->initialize();};
    init_timer_ = this->create_wall_timer(std::chrono::seconds(1),
        initialize_callback);
  }

  void initialize()
  {
    update_timer_ = swri::Timer(*this, std::chrono::milliseconds(20),
                                &TimerTest::handleUpdateTimer,
                                this);

    diagnostic_updater_->setHardwareID("none");
    diagnostic_updater_->add(
      "swri::Timer test", this,
      &TimerTest::timerDiagnostics);
    
    diag_timer_ = swri::Timer(*this, std::chrono::seconds(1),
                              &TimerTest::handleDiagnosticsTimer,
                              this);
  }

  void handleUpdateTimer()
  {
    // Do some work to give us a measurable time.
    int64_t fibonacci_index = this->get_parameter("fibonacci_index").as_int();
    size_t number = super_slow_fibonacci(fibonacci_index);
    RCLCPP_INFO(this->get_logger(), "The %d-th number of the fibonacci sequence is %lu",
             fibonacci_index, number);
  }

  int super_slow_fibonacci(int x)
  {
    if (x <= 0) {
      return 0;
    } else if (x == 1) {
      return 1;
    } else {
      return super_slow_fibonacci(x-1) + super_slow_fibonacci(x-2);
    }
  }

  void handleDiagnosticsTimer()
  {
    diagnostic_updater_->force_update();
  }

  void timerDiagnostics(du::DiagnosticStatusWrapper& status) // NOLINT
  {
    status.summary(DS::OK, "No errors reported.");

    // This example uses swri::Timer's appendDiagnostics method to
    // include diagnostics using a common format and common summary
    // semantics.  If we didn't care about specific bits of
    // information, we use more specific flags.
    update_timer_.appendDiagnostics(status, "Update",
                                    swri::Timer::DIAG_ALL);
  }
  
};  // class TimerTest

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<TimerTest> node = std::make_shared<TimerTest>("timer_test");

  rclcpp::spin(node);

  return 0;
}
