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

## Dynamic Parameters

A class that implements the same functionality as Dynamic Reconfigure, but in a more dynamic and easier to use way. Instead of declaring configurable parameters at compile time in a config file they are specified in your C++ code.

To use, add the class to your node and call the initalize function with a NodeHandle in your private namespace.

```cpp
#include <swri_roscpp/dynamic_parameters.h>

void main()
{
  // your node stuff here

  ros::NodeHandle pnh("~");
  swri::DynamicParameters params;
  params.initialize(pnh);
...
```

You can then start declaring and reading in configuration values, giving a long living pointer to a variable for each. This variable is changed on a dynamic reconfigure of that parameter. Be sure to lock the parameter mutex before reading any of these parameters. You can do this by calling the `params.mutex()` function and manually locking/unlocking it, or by using the `.get()` function on each `*Param` to do it automatically.

```cpp
  FloatParam flt;
  params.get("float_value", flt, 
              10.0f /*default*/, "Description...", 
             -10.0f /* min */, 10.0f /* max */);
  DoubleParam dbl;
  params.get("double_value", dbl, 
              10.0 /*default*/, "Description...", 
             -10.0 /* min */, 10.0 /* max */);
  BoolParam bl;
  params.get("bool_value", bl, 
              10.0f /*default*/, "Description...");
  StringParam str;
  params.get("string_value", str, 
              "default" /*default*/, "Description...");
```

You can also load the parameters directly in to normal variables (doubles, ints, strings), but you will have to get the new values in the on change callback function discussed below.

```cpp
  float flt;
  params.get("float_value", flt, 
              10.0f /*default*/, "Description...", 
             -10.0f /* min */, 10.0f /* max */);
  double dbl;
  params.get("double_value", dbl, 
              10.0 /*default*/, "Description...", 
             -10.0 /* min */, 10.0 /* max */);
  bool bl;
  params.get("bool_value", bl, 
              10.0f /*default*/, "Description...");
  std::string str;
  params.get("string_value", str, 
              "default" /*default*/, "Description...");
```


These functions read in the current parameter value to the provided variable, if they haven't been set they are set to the default value. After you have read in all the variables you want to be dynamically reconfigurable, call the `finalize()` function.

```cpp
  params.finalize();
```


This publishes the configuration options to a latched topic so that the dynamic reconfigure tools can read them in. 

After you call finalize, you should be able to access and dynamically configure your node with any dynamic reconfigure compatible tools.

Then to use/read in the parameter values in your code:

```cpp
  float val = flt.get();// or *flt to get it without locking the mutex if you know what you are doing
```

You can also lock the mutex manually for getting the parameters in a block as follows:

```cpp
  params.mutex().lock();

  float a = *flt;
  double b = *dbl;
  bool c = *bl;
  std::string d = *str; 

  params.mutex().unlock();
```

Alternatively you can use the on change callback to be notified when variables change and load the updated values then.

```cpp

void callback(swri::DynamicParameters& params)
{
  double param1 = params.getDouble("double_value");
  float  param2 = params.getFloat("float_value");
  int    param3 = params.getInt("int_value");
  bool   param4 = params.getBool("bool_value");
  std::string param5 = params.getString("string_value");
}

// somewhere in your initialization....
params.setCallback(callback);

```
