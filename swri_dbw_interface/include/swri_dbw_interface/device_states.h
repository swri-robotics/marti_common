// *****************************************************************************
//
// Copyright (c) 2017, Southwest Research Institute® (SwRI®)
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

#ifndef SWRI_DBW_INTERFACE_DEVICE_STATES_H_
#define SWRI_DBW_INTERFACE_DEVICE_STATES_H_

#include <string>

namespace swri_dbw_interface {

///////////////////////////////////////////////////////////////////////////////
// The follow define common states used for transmission devices in the
// swri_dbw_interface.

// These values are valid command and feedback states.
static const std::string TRANSMISSION_PARK = "park";
static const std::string TRANSMISSION_DRIVE = "drive";
static const std::string TRANSMISSION_DRIVE_LOW = "drive_low";
static const std::string TRANSMISSION_REVERSE = "reverse";
static const std::string TRANSMISSION_NEUTRAL = "neutral";

// The following states are only valid for feedback states.
static const std::string TRANSMISSION_SHIFTING = "shifting";
static const std::string TRANSMISSION_UNKNOWN = "unknown";

///////////////////////////////////////////////////////////////////////////////
// This following define common states used for turn signal devices in the
// swri_dbw_interface.

// These values are valid command and feedback states.
static const std::string TURN_SIGNAL_NONE = "none";
static const std::string TURN_SIGNAL_LEFT = "left";
static const std::string TURN_SIGNAL_RIGHT = "right";
static const std::string TURN_SIGNAL_HAZARD = "hazard";

}  // namespace swri_dbw_interface

#endif  // SWRI_DBW_INTERFACE_DEVICE_STATES_H_

