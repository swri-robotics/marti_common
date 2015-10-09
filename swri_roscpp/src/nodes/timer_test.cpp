#include <ros/ros.h>
#include <swri_roscpp/timer.h>
#include <swri_roscpp/parameters.h>

#include <diagnostic_updater/diagnostic_updater.h>

namespace du = diagnostic_updater;

// Alias type for easier access to DiagnosticStatus enumerations.
typedef diagnostic_msgs::DiagnosticStatus DS;

class TimerTest
{
  ros::NodeHandle nh_;
  ros::WallTimer init_timer_;
  swri::Timer update_timer_;
  swri::Timer diag_timer_;

  du::Updater diagnostic_updater_;

  int fibonacci_index_;
  
 public:
  TimerTest()
  {
    // Setup a one-shot timer to initialize the node after a brief
    // delay so that /rosout is always fully initialized.
    ROS_INFO("Starting initialization timer...");
    init_timer_ = nh_.createWallTimer(ros::WallDuration(1.0),
                                      &TimerTest::initialize,
                                      this,
                                      true);
  }

  void initialize(const ros::WallTimerEvent &ignored)
  {
    update_timer_ = swri::Timer(nh_, ros::Duration(1.0/50.0),
                                &TimerTest::handleUpdateTimer,
                                this);

    ros::NodeHandle pnh("~");
    swri::param(pnh, "fibonacci_index", fibonacci_index_, 30);
    
    diagnostic_updater_.setHardwareID("none");
    diagnostic_updater_.add(
      "swri::Timer test", this,
      &TimerTest::timerDiagnostics);
    
    diag_timer_ = swri::Timer(nh_, ros::Duration(1.0),
                              &TimerTest::handleDiagnosticsTimer,
                              this);
  }

  void handleUpdateTimer(const ros::TimerEvent &ignored)
  {
    // Do some work to give us a measurable time.
    size_t number = super_slow_fibonacci(fibonacci_index_);
    ROS_INFO("The %d-th number of the fibonacci sequence is %lu",
             fibonacci_index_, number);
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

  void handleDiagnosticsTimer(const ros::TimerEvent &ignored)
  {
    diagnostic_updater_.update();
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
  ros::init(argc, argv, "timer_test");

  TimerTest node;
  ros::spin();

  return 0;
}
