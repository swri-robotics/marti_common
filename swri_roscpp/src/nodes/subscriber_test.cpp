#include <boost/bind.hpp>
#include <ros/ros.h>
#include <swri_roscpp/subscriber.h>
#include <boost/bind.hpp>

#include <diagnostic_updater/diagnostic_updater.h>

#include <std_msgs/Float32.h>
#include <nav_msgs/Odometry.h>

namespace du = diagnostic_updater;

// Alias type for easier access to DiagnosticStatus enumerations.
typedef diagnostic_msgs::DiagnosticStatus DS;

class SubscriberTest
{
  ros::NodeHandle nh_;
  ros::WallTimer init_timer_;
  ros::Timer diag_timer_;

  du::Updater diagnostic_updater_;
  
  swri::Subscriber sub_;
  
 public:
  SubscriberTest()
  {
    // Setup a one-shot timer to initialize the node after a brief
    // delay so that /rosout is always fully initialized.
    ROS_INFO("Starting initialization timer...");
    init_timer_ = nh_.createWallTimer(ros::WallDuration(1.0),
                                      &SubscriberTest::initialize,
                                      this,
                                      true);
  }

  void initialize(const ros::WallTimerEvent &ignored)
  {
    // Example to subscribe to a class method.
    sub_ = swri::Subscriber(nh_, "odom", 100, &SubscriberTest::handleMessage, this);

    // Example to subscribe using a boost function.  I am unable to
    // figure out how to get the compiler to infer the type, so I have
    // to declare the function first and then pass that.
    boost::function<void(const boost::shared_ptr< nav_msgs::Odometry const> &)> callback = boost::bind(&SubscriberTest::handleMessage, this, _1);
    sub_ = swri::Subscriber(nh_, "odom", 100, callback);

    sub_.setTimeout(ros::Duration(1.0));

    diagnostic_updater_.setHardwareID("none");
    diagnostic_updater_.add(
      "swri::Subscriber test (manual diagnostics)", this,
      &SubscriberTest::manualDiagnostics);

    diagnostic_updater_.add(
      "swri::Subscriber test (auto diagnostics)", this,
      &SubscriberTest::autoDiagnostics);
    
    diag_timer_ = nh_.createTimer(ros::Duration(1.0),
                                  &SubscriberTest::handleDiagnosticsTimer,
                                  this);
  }

  void handleMessage(const nav_msgs::OdometryConstPtr &msg)
  {
    ROS_INFO_THROTTLE(1.0, "Receiving messages.");
  }

  void handleDiagnosticsTimer(const ros::TimerEvent &ignored)
  {
    diagnostic_updater_.update();
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

    if (sub_.mappedTopic() == sub_.unmappedTopic()) {
      status.addf("Topic Name", "%s", sub_.mappedTopic().c_str());
    } else {
      status.addf("Topic Name", "%s -> %s",
                  sub_.unmappedTopic().c_str(),
                  sub_.mappedTopic().c_str());
    }
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
  ros::init(argc, argv, "subscriber_test");

  SubscriberTest node;
  ros::spin();
  
  return 0;  
}
