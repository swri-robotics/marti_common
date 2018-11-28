# swri_roscpp

This packages contains a set of wrappers classes and functions for common ROS operations.


## Topic Services

Topic services are a wrappers over ROS topics that provide a service like functionality but in a much more introspectable way. Eg: There is no rosservice echo and no way to view service clients in a running system.

The usage is very similar to that of standard ROS services and can even make use of the same service definitions. For most usages simply just substitute `ros::ServiceServer` or `ros::ServiceClient` for `swri::TopicServiceServer` or `swri::TopicServiceClient` in the declaration and use it in the same way.

### Generating Topics from a Service

In order to generate the backing topics for a Topic Service from a service message add a `add_topic_service_files` block to your project's CMakeList.txt and use it the same way you would an `add_message_files` or `add_service_files` block.

Make sure to add this `swri_roscpp` as a dependency to your message project.

For example:

```cmake
add_topic_service_files(DIRECTORY topic_srv FILES
  ClearActiveRoute.srv
  DeleteRoute.srv
  GetRoute.srv
  GetRouteNames.srv
  SaveRoute.srv
  SetActiveRoute.srv
  SetRoute.srv
  SetNextCheckpoint.srv
)
```

To use the generated topics simply include `your_project/your_service.h`. They have response and request fields just like normal services.
