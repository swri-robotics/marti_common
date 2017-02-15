// *****************************************************************************
//
// Copyright (c) 2016, Southwest Research Institute® (SwRI®)
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

#ifndef SWRI_NODELET_CLASS_LIST_MACROS_H_
#define SWRI_NODELET_CLASS_LIST_MACROS_H_

#include <boost/make_shared.hpp>
#include <pluginlib/class_list_macros.h>

/*
 Macro to define a nodelet with a factory function, so that it can be used in
 boilerplate wrapper nodes that do not rely on dynamic class loading.
 
 The macro calls PLUGINLIB_EXPORT_CLASS with plugin type nodelet::Nodelet and
 creates a factory function NS::createCLASS() that returns a 
 boost::shared_ptr<nodelet::Nodelet> to NS::CLASS
 
 @param NS The namespace of the class to be used for the nodelet
 @param CLASS The classname of the class to be used for the nodelet
*/
#define SWRI_NODELET_EXPORT_CLASS(NS, CLASS) PLUGINLIB_EXPORT_CLASS(NS::CLASS, nodelet::Nodelet);\
namespace NS\
{\
  boost::shared_ptr<nodelet::Nodelet> create ## CLASS()\
  {\
    return boost::make_shared<CLASS>();\
  }\
}

#endif  // SWRI_NODELET_CLASS_LIST_MACROS_H_

