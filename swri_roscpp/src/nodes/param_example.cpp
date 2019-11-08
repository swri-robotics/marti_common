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
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <swri_roscpp/parameters.h>

int main(int argc, char **argv)
{
  rclcpp::Node pnh("param_test");

  int int_var;
  swri::getParam(pnh, "used_int_var_1", int_var);
  swri::param(pnh, "used_int_var_2", int_var, 2);
  swri::param(pnh, "unset_int_var", int_var, 2);
  
  std::string str_var;
  swri::getParam(pnh, "used_str_var_1", str_var);
  swri::param(pnh, "used_str_var_2", str_var, "foo");
  swri::param(pnh, "unset_str_var", str_var, "foo");
  
  double dbl_var;
  swri::getParam(pnh, "used_dbl_var_1", dbl_var);
  swri::param(pnh, "used_dbl_var_2", dbl_var, 2.0);
  swri::param(pnh, "unset_dbl_var", dbl_var, 2.0);
  
  bool bool_var;
  swri::getParam(pnh, "used_bool_var_1", bool_var);
  swri::param(pnh, "used_bool_var_2", bool_var, true);
  swri::param(pnh, "unset_bool_var", bool_var, true);
  
  return 0;
}
