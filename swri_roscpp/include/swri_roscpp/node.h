#ifndef SWRI_ROSCPP_NODE_H_
#define SWRI_ROSCPP_NODE_H_

#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rclcpp/macros.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>

namespace swri
{
class Node : rclcpp::Node
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS(Node);
  RCLCPP_PUBLIC
  explicit Node(
    const std::string& node_name,
    const std::string& doc_string,
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions()) :
      rclcpp::Node(node_name, options)
    {
      add_doc_string(doc_string);
    }
    
  RCLCPP_PUBLIC
  explicit Node(
    const std::string& node_name,
    const std::string& doc_string,
    const std::string& namespace_,
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions()) :
      rclcpp::Node(node_name, namespace_, options)
    {
      add_doc_string(doc_string);
    }

private:
  void add_doc_string(const std::string& doc_string)
  {
    auto param_desc = rcl_interfaces::msg::ParameterDescriptor();

    param_desc.description = "Documents the node's purpose.";
    param_desc.read_only = true;

    this->declare_parameter("doc_string", doc_string, param_desc);
  }
}; // class SwriNode

} // namespace swri

#endif // include guard
