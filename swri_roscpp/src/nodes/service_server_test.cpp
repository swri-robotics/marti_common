#include <ros/ros.h>
#include <swri_roscpp/service_server.h>

#include <diagnostic_updater/diagnostic_updater.h>

#include <std_srvs/Empty.h>

namespace du = diagnostic_updater;

// Alias type for easier access to DiagnosticStatus enumerations.
typedef diagnostic_msgs::DiagnosticStatus DS;

class ServiceServerTest
{
  ros::NodeHandle nh_;
  ros::WallTimer init_timer_;
  ros::Timer diag_timer_;

  du::Updater diagnostic_updater_;
  
  swri::ServiceServer test1_srv_;
  swri::ServiceServer test2_srv_;
  swri::ServiceServer test3_srv_;

  bool test1_result_;
  
 public:
  ServiceServerTest()
    :
    test1_result_(true)
  {
    // Setup a one-shot timer to initialize the node after a brief
    // delay so that /rosout is always fully initialized.
    ROS_INFO("Starting initialization timer...");
    init_timer_ = nh_.createWallTimer(ros::WallDuration(1.0),
                                      &ServiceServerTest::initialize,
                                      this,
                                      true);
  }

  void initialize(const ros::WallTimerEvent &ignored)
  {
    test1_srv_.setInstrumentPerClient(true);
    test1_srv_ = swri::ServiceServer(nh_, "test_service1",
                                     &ServiceServerTest::handleService1,
                                     this);

    test2_srv_.setInstrumentPerClient(false);
    test2_srv_ = swri::ServiceServer(nh_, "test_service2",
                                     &ServiceServerTest::handleService2,
                                     this);

    test3_srv_.setLogCalls(true);
    test3_srv_.setInstrumentPerClient(false);
    test3_srv_ = swri::ServiceServer(nh_, "test_service3",
                                     &ServiceServerTest::handleService3,
                                     this);
    
    
    diagnostic_updater_.setHardwareID("none");
    diagnostic_updater_.add(
      "swri::ServiceServer test service 1", this,
      &ServiceServerTest::service1Diagnostics);

    diagnostic_updater_.add(
      "swri::ServiceServer test service 2", this,
      &ServiceServerTest::service2Diagnostics);

    diagnostic_updater_.add(
      "swri::ServiceServer test service 3", this,
      &ServiceServerTest::service3Diagnostics);
    
    diag_timer_ = nh_.createTimer(ros::Duration(1.0),
                                  &ServiceServerTest::handleDiagnosticsTimer,
                                  this);
  }

  bool handleService1(std_srvs::Empty::Request &req,
                      std_srvs::Empty::Response &res)
  {
    ROS_INFO("test service 1 called. returning %s",
             test1_result_ ? "true" : "false");
    return test1_result_;
  }

  bool handleService2(ros::ServiceEvent<std_srvs::Empty::Request,
                                        std_srvs::Empty::Response> &event)
  {
    ROS_INFO("test service 2 called");
    test1_result_ = !test1_result_;
    return true;
  }

  bool handleService3(const std::string &name,
                      const std_srvs::Empty::Request &request,
                      std_srvs::Empty::Response &response)
  {
    ROS_INFO("test service 3 called by %s", name.c_str());
    return true;
  }
  
  void handleDiagnosticsTimer(const ros::TimerEvent &ignored)
  {
    diagnostic_updater_.update();
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
  ros::init(argc, argv, "service_server_test");

  ServiceServerTest node;
  ros::spin();
  
  return 0;  
}
