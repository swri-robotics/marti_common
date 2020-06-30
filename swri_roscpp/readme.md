# swri_roscpp

This packages contains a set of wrappers classes and functions for common ROS operations.

## Topic Services

Topic services are a wrappers over ROS topics that provide a service like functionality but in a much more introspectable way. Eg: There is no ros2 service echo and no way to view service clients in a running system.

The usage is very similar to that of standard ROS services and can even make use of the same service definitions. For most usages simply just substitute `ros::ServiceServer` or `ros::ServiceClient` for `swri::TopicServiceServer` or `swri::TopicServiceClient` in the declaration and use it in the same way.

### Generating Topics from a Service

In order to generate the backing topics for a Topic Service from a service message add a `generate_topic_service_files` block to your project's CMakeList.txt and use it almost the same way you would an `rosidl_generate_interfaces` block. This generates the individual topic service messages and returns a list of these to the variable named by the first argument. Make sure to add these to your rosidl_generate_interfaces call along with the marti_common_msgs dependency.

Make sure to add `swri_roscpp` as a dependency to your message project.

For example:

```cmake
generate_topic_service_files(generated_message_list
 DIRECTORY topic_srv
 FILES
  ClearActiveRoute.srv
  DeleteRoute.srv
  GetRoute.srv
  GetRouteNames.srv
  SaveRoute.srv
  SetActiveRoute.srv
  SetRoute.srv
  SetNextCheckpoint.srv
)

rosidl_generate_interfaces(${PROJECT_NAME}
   your_other_messages_here.msg
   ${generated_message_list}
  DEPENDENCIES 
   ${MSG_DEPS}
   marti_common_msgs
)
```

To use the generated topics simply include `your_project/srv/your_service.h`. They have response and request fields just like normal services.

## Dynamic Parameters

**Note:** This package's dynamic parameter implementation has been deprecated and removed; it is unnecessary in ROS 2 Dashing and `rclcpp::Node::declare_parameter` provides an equivalent alternative.

