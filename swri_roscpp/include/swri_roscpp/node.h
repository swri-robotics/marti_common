// *****************************************************************************
//
// Copyright (c) 2023, Southwest Research Institute® (SwRI®)
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
// ARE DISCLAIMED. IN NO EVENT SHALL SOUTHWEST RESEARCH INSTITUTE BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// *****************************************************************************

#ifndef SWRI_ROSCPP_NODE_H_
#define SWRI_ROSCPP_NODE_H_

#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rclcpp/macros.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <rcpputils/asserts.hpp>

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
    rcpputils::check_true(doc_string.length() > 0);

    auto param_desc = rcl_interfaces::msg::ParameterDescriptor();

    param_desc.description = "Documents the node's purpose.";
    param_desc.read_only = true;

    this->declare_parameter("node_documentation", doc_string, param_desc);
  }
}; // class Node

} // namespace swri

#endif // include guard
