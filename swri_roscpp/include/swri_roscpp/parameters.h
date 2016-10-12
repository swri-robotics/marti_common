#ifndef SWRI_ROSCPP_PARAMETERS_H_
#define SWRI_ROSCPP_PARAMETERS_H_

#include <algorithm>
#include <set>
#include <string>
#include <vector>

#include <boost/bind.hpp>

#include <ros/console.h>
#include <ros/node_handle.h>

namespace swri
{

  /// This set stores all of the parameters that have been got with this library
  /// It is used for the getUnusedParamKeys and WarnUnusedParams functions
  static std::set<std::string> _used_params;

  static inline
  bool getParam(const ros::NodeHandle &nh,
      const std::string &name,
      int &variable)
  {
    std::string resolved_name = nh.resolveName(name);
    _used_params.insert(resolved_name);
    if (!nh.getParam(name, variable))
    {
      ROS_ERROR("Required int parameter %s does not exist", name.c_str());
      return false;
    }
    ROS_INFO("Read parameter %s = %d", name.c_str(), variable);
    return true;
  }

  static inline
  bool getParam(const ros::NodeHandle &nh,
      const std::string &name,
      double &variable)
  {
    std::string resolved_name = nh.resolveName(name);
    _used_params.insert(resolved_name);
    if (!nh.getParam(name, variable))
    {
      ROS_ERROR("Required double parameter %s does not exist", name.c_str());
      return false;
    }
    ROS_INFO("Read parameter %s = %lf", name.c_str(), variable);
    return true;
  }

  static inline
  bool getParam(const ros::NodeHandle &nh,
      const std::string &name,
      float &variable)
  {
    double dbl_value;
    if (!nh.getParam(name, dbl_value))
    {
      ROS_ERROR("Required double parameter %s does not exist", name.c_str());
      return false;
    }
    variable = dbl_value;
    ROS_INFO("Read parameter %s = %f", name.c_str(), variable);
    return true;
  }

  static inline
  bool getParam(const ros::NodeHandle &nh,
      const std::string &name,
      std::string &variable)
  {
    std::string resolved_name = nh.resolveName(name);
    _used_params.insert(resolved_name);
    if (!nh.getParam(name, variable))
    {
      ROS_ERROR("Required string parameter %s does not exist", name.c_str());
      return false;
    }
    ROS_INFO("Read parameter %s = %s", name.c_str(), variable.c_str());
    return true;
  }

  static inline
  bool getParam(const ros::NodeHandle &nh,
      const std::string &name,
      bool &variable)
  {
    std::string resolved_name = nh.resolveName(name);
    _used_params.insert(resolved_name);
    if (!nh.getParam(name, variable))
    {
      ROS_ERROR("Required bool parameter %s does not exist", name.c_str());
      return false;
    }
    ROS_INFO("Read parameter %s = %s", name.c_str(), variable ? "true" : "false");
    return true;
  }

  static inline
  void param(const ros::NodeHandle &nh,
      const std::string &name,
      int &variable,
      const int default_value)
  {
    std::string resolved_name = nh.resolveName(name);
    _used_params.insert(resolved_name);
    nh.param(name, variable, default_value);
    ROS_INFO("Read parameter %s = %d", name.c_str(), variable);
  }

  static inline
  void param(const ros::NodeHandle &nh,
      const std::string &name,
      double &variable,
      const double default_value)
  {
    std::string resolved_name = nh.resolveName(name);
    _used_params.insert(resolved_name);
    nh.param(name, variable, default_value);
    ROS_INFO("Read parameter %s = %lf", name.c_str(), variable);
  }

  static inline
  void param(const ros::NodeHandle &nh,
      const std::string &name,
      float &variable,
      const float default_value)
  {
    double dbl_value;
    double dbl_default = default_value;
    nh.param(name, dbl_value, dbl_default);
    variable = dbl_value;
    ROS_INFO("Read parameter %s = %f", name.c_str(), variable);
  }

  static inline
  void param(const ros::NodeHandle &nh,
      const std::string &name,
      std::string &variable,
      const std::string default_value)
  {
    std::string resolved_name = nh.resolveName(name);
    _used_params.insert(resolved_name);
    nh.param(name, variable, default_value);
    ROS_INFO("Read parameter %s = \"%s\"", name.c_str(), variable.c_str());
  }

  static inline
  void param(const ros::NodeHandle &nh,
      const std::string &name,
      bool &variable,
      const bool default_value)
  {
    std::string resolved_name = nh.resolveName(name);
    _used_params.insert(resolved_name);
    nh.param(name, variable, default_value);
    ROS_INFO("Read parameter %s = %s", name.c_str(), variable ? "true" : "false");
  }

  /**
   * Determines whether `prefix` is a prefix of `string`.
   *
   * @param string The longer string to check, which may begin with the
   *               characters of `prefix`.
   * @param prefix The shorter string, which may be a prefix of `string`.
   * @return 0 if the first n characters of `string` are identical to `prefix`,
   *         where n is the length of `prefix`. 1 or -1 otherwise, according to
   *         the conventions of std::string::compare()
   */
  inline int comparePrefix(std::string const& string, std::string const& prefix)
  {
    return string.compare(0, prefix.size(), prefix);
  }

  /**
   * Boolean wrapper around comparePrefix.
   *
   * @param string The longer string to check, which may begin with the
   *               characters of `prefix`.
   * @param prefix The shorter string, which may be a prefix of `string`.
   * @return True if the first n characters of `string` are identical to
   *         `prefix`, where n is the length of `prefix`, false otherwise.
   */
  inline bool isPrefixOf(std::string const& string, std::string const& prefix)
  {
    return comparePrefix(string, prefix) == 0;
  }

  /**
   * Less-than wrapper around comparePrefix, for algorithms that require a
   * less-than comparator, such as sorting.
   *
   * @param string The longer string to check, which may begin with the
   *               characters of `prefix`.
   * @param prefix The shorter string, which may be a prefix of `string`.
   * @return True if the first n characters of `string` are "less than" the
   *         first n to `prefix`, where n is the length of `prefix`, according
   *         to the conventions of std::string::compare(). False otherwise.
   */
  inline int prefixLessThan(std::string const& string, std::string const& prefix)
  {
    return comparePrefix(string, prefix) < 0;
  }

  /**
   * Gets the list of parameters on the parameter server in the namespace of
   * NodeHandle n that have not yet been got using any of the swri::param() or
   * swri::getParam() functions.
   *
   * The parameter keys are filtered by prefix, so if there exists a parameter
   * /foo/bar/baz and parameter /foo/bar has been got, then /foo/bar/baz will
   * not be returned as an unused parameter.
   *
   * Note that this function has no way of knowing about parameters got
   * using the ros::param() and ros::getParam() functions.
   *
   * @param n  Nodehandle defining the namespace to restrict the list
   * @return   A vector of fully-qualified parameter names in n's
   *           namespace that haven't been got.
   */
  static inline
  std::vector<std::string> getUnusedParamKeys(ros::NodeHandle const& n)
  {
    // Get a list of every parameter on the parameter server
    std::string const& n_namespace = n.getNamespace();
    std::vector<std::string> all_params;
    n.getParamNames(all_params);

    // Sort the list of parameters to permit partitioning it by namespace
    std::sort(all_params.begin(), all_params.end());

    // Partition the list of parameters to only the namespace of interest
    // Create a functor that tells whether a parameter is in a namespace
    boost::function<bool(std::string)> inNamespace = boost::bind(isPrefixOf, _1, n_namespace);
    // Find the beginning of the namespace in the sorted list
    std::vector<std::string>::const_iterator first_param_in_namespace = find_if(all_params.begin(), all_params.end(), inNamespace);
    // Find the end of the namespace in the sorted list
    std::vector<std::string>::const_iterator after_last_param_in_namespace = find_if(all_params.begin(), all_params.end(), std::not1(inNamespace));
    // At this point, all parameters on the server that are in the namespace
    // are in range [`first_param_in_namespace`,`after_last_param_in_namespace`)

    // Preallocate a vector to put unused params in.
    std::vector<std::string> unused_params(all_params.size());

    // Fill unused_params with the set difference (params in namespace) - (used params)
    std::vector<std::string>::iterator last_unused_param = std::set_difference(
        first_param_in_namespace, after_last_param_in_namespace,  // Set 1
        _used_params.begin(), _used_params.end(),                 // Set 2
        unused_params.begin(),  // Destination iterator
        prefixLessThan);        // Prefix comparator so that all sub-keys of a
                                // used parameter are also considered used

    // Shrink the unused_params vector to fit contents
    unused_params.resize(last_unused_param - unused_params.begin());

    return unused_params;
  }

  /**
   * Wrapper around getUnusedParamKeys that emits a warning for every unused
   * parameter
   * @param n  Nodehandle defining the namespace to restrict the list
   */
  static inline
  void warnUnusedParams(ros::NodeHandle const& n)
  {
    std::vector<std::string> unused_params = getUnusedParamKeys(n);
    for (std::vector<std::string>::const_iterator itr = unused_params.begin(); itr != unused_params.end(); ++itr)
    {
      ROS_WARN("Parameter %s was set, but not used by this node", itr->c_str());
    }
  }
}  // namespace swri_param
#endif  // SWRI_ROSCPP_PARAMETERS_H_
