# swri_dbw_interface

This package provides documentation and tools to define and specify common 
interface conventions for various drive-by-wire systems and the SwRI autonomy
system.

More specifically, the practical purpose here is to define and specify the ROS
interface between the SwRI autonomy system and a generic/standardized DBW system, 
where the generic/standardized messages would then be translated to and from the 
native DBW system. This type standardized interface would enable and encourage 
consistency across vehicle platforms by ensuring that the autonomy software can
be decoupled from vehicle-specific implementations.

## Contents
 - [Overview](#overview-)
 - [Conventions](#conventions-)
 - [Namespace](#namespace-)
 - [Status Feedback](#status-feedback-)
 - [Steering Interface](#steering-interface-)
 - [Throttle Interface](#throttle-interface-)
 - [Speed Interface](#speed-interface-)
 - [Robotic Mode Interface](#robotic-mode-interface-)
 - [Transmission Interface](#transmission-interface-)
 - [Ignition/Engine Interface](#ignitionengine-interface-)
 - [Turn Signal Interface](#turn-signal-interface-)
 - [E-Stop Interface](#e-stop-interface-)
 - [Button Interfaces](#button-interfaces-)

## Overview [⬆](#swri_dbw_interface)

The swri_dbw_interface defines an abstract interface to a native
vehicle drive-by-wire system. In practice, this is most commonly
implemented as a translation layer by wrapping the native interface
and translating between the swri_dbw_interface messages and the 
native drive-by-wire system messages and vice versa.  This layer may
be anything from simple translations between message types to complex
closed loop controllers.  It may consist of one node or many nodes.
The important aspect is a consistent interface presented to the higher
level autonomy system, not how it is implemented.

This interface was designed for actuated gas/brake
Ackermann-steered vehicles (as opposed to tracked vehicles or vehicles
that provide intrinsic speed control).  It is expected that the
interface will be extended to handle other types of vehicles as they
are encountered.

Due to the combined unique needs of use cases and variations in
drive-by-wire capabilities and implementations, it is expected that
variations and adaptations may be required for different
circumastances.  In those cases, any variations should be documented
along with the implementation.

As new features are required, it is recommended that they be developed
for the specific use case according to the conventions below, tested
and refined in an actual use case, and then, if they are generic
enough for other applications, merged into these specifications once
they are mature.


## Conventions [⬆](#swri_dbw_interface)

We have developed many conventions that are followed to have as much
internal consistency as possible.

1. Every axis of control should have a command topic, used by the
   higher autonomy to control the hardware, and a feedback topic, used
   by the higher autonomy to determine the current state of the axis
   (or the predicted state if no actual feedback is available).

1. Command topics should have topic names with the postfix "_command".

1. Feedback topics should have topic names with the postfix "_feedback".

1. Other topics should be given names that make it obvious whether
   they are an input to or output from the DBW system.

1. Prefer to use strings (with constants) over integer enumerations so
   they are easy to introspect using "rostopic echo".  Constants should be
   defined in [device_states.h](../include/swri_dbw_interface/device_states.h)

1. For axes that change infrequently (e.g, transmission), make the
   feedback topic latched.  Publish the state when it changes, and at
   a fixed slow rate (e.g. 1Hz) for easier usage with bag files.  This
   rate can be a configurable parameter.  This specification will
   label topics that follow this convention as SLOW/ONCHANGE.

1. Prefer topics over services for easier usage with bag files.

1. If a DBW implementation provides feedback but no control of an
   axis, it is acceptable to either subscribe but ignore message on
   tha command topic, or not subscribe to the topic at all.

1. Use the ROS console macros to provide useful feedback for
   developers and troubleshooters, but be careful not spam the output.


## Namespace [⬆](#swri_dbw_interface)

By convention, all topics related to the drive-by-wire system are in
the "/vehicle_interface" namespace.

## Status Feedback [⬆](#swri_dbw_interface)

Most axes/devices have a status feedback topic that is published at
SLOW/ONCHANGE.  In all cases, the status field should follow the
convention described in the HealthStatus message definition, and the
message field should be filled out with an informative message
describing the most critical issue.

Some systems provide specific health feedback for critical devices
like steering, throttle, or brake, which can be communicated using the
status feedback.  Otherwise, the status feedback will typically be
determined by whether or not the interface node is communicating with
the DBW hardware.

## Steering Interface [⬆](#swri_dbw_interface)

The steering interface has two main topics for control:

- **/vehicle_interface/steering_command** -
*(marti_common_msgs/Float64Stamped)* - *subscription* - Desired
position of the steering axis normalized between [0.0, 1.0].  Expected
at a constant rate from the high level ROS system (typically 50Hz).

- **/vehicle_interface/steering_feedback** -
*(marti_common_msgs/Float64Stamped)* - *publication* - Measured
position of the steering axis normalized between [0.0, 1.0].  Expected
to be published at a constant rate to the rest of the system
(typically 50Hz).

The steering axis is typically normalized between the documented range
of the underlying DBW system (e.g. [0,1023]).  The direction does not
matter (0.0 may map to full left or full right) and is accounted for
by the steering calibration.

In some cases, we have encountered platforms that only use a tiny
portion of the documented range (e.g, a system with a documented range
of [0, 65535] with a useful travel only over [4000, 8000].  In those
cases, it is handy to only map slightly more than than the used range
(e.g. [3500 - 8500]) instead of the documented range for convenient
use with the steering calibration tool.

It is extremely helpful for the steering feedback to be provided even
in manual mode for calibration and continuity when switching into
robotic mode.  Every effort should be made to get a DBW supplier to
support this.  If not, the steering has to be calibrated under robotic
control using joystick for control which is much less convenient.

- **/vehicle_interface/steering_status** -
*(marti_common_msgs/HealthStatus)* - *publication* - Summarized health status of the
steering system.  The status field should follow the guidelines in the
HealthStatus mesage definition, and the message field should be filled
out with an informative value. SLOW/ONCHANGE.


## Throttle Interface [⬆](#swri_dbw_interface)

The throttle interface has two main topics for control:

- **/vehicle_interface/throttle_command** -
*(marti_common_msgs/Float64Stamped)* - *subscription* - Desired
position of the throttle axis normalized between [0.0, 1.0].  Expected
at a constant rate from the high level ROS system (typically 50Hz).

- **/vehicle_interface/throttle_feedback** -
*(marti_common_msgs/Float64Stamped)* - *publication* - Measured
position of the throttle axis normalized between [0.0, 1.0].  Expected
to be published at a constant rate to the rest of the system
(typically 50Hz).

The throttle axis is typically normalized between the documented range
of the underlying DBW system (e.g. [0,1023]).

- **/vehicle_interface/throttle_status** -
*(marti_common_msgs/HealthStatus)* - *publication* - Summarized health
status of the throttle control system.  The status field should follow the
guidelines in the HealthStatus mesage definition, and the message
field should be filled out with an informative value. SLOW/ONCHANGE.


## Brake Interface [⬆](#swri_dbw_interface)

The service brake interface has two main topics for control:

- **/vehicle_interface/brake_command** -
*(marti_common_msgs/Float64Stamped)* - *subscription* - Desired
position of the brake axis normalized between [0.0, 1.0].  Expected
at a constant rate from the high level ROS system (typically 50Hz).

- **/vehicle_interface/brake_feedback** -
*(marti_common_msgs/Float64Stamped)* - *publication* - Measured
position of the brake axis normalized between [0.0, 1.0].  Expected
to be published at a constant rate to the rest of the system
(typically 50Hz).

The brake axis is typically normalized between the documented range
of the underlying DBW system (e.g. [0,1023]).

- **/vehicle_interface/brake_status** -
*(marti_common_msgs/HealthStatus)* - *publication* - Summarized health
status of the brake control system.  The status field should follow the
guidelines in the HealthStatus mesage definition, and the message
field should be filled out with an informative value. SLOW/ONCHANGE.

## Speed Interface [⬆](#swri_dbw_interface)

On some platforms, throttle/brake control may be handled by the DBW system and
a speed interface is provided.  In some cases DBW systems with throttle/brake
interfaces will also provide speed feedback.

The speed interface has two main topics for control:

- **/vehicle_interface/speed_command** -
*(marti_common_msgs/Float64Stamped)* - *subscription* - Desired
positive speed in m/s.  Expected at a constant rate from the high level ROS
system (typically 50Hz).

- **/vehicle_interface/speed_feedback** -
*(marti_common_msgs/Float64Stamped)* - *publication* - Measured
speed in m/s.  Expected to be published at a constant rate to the rest of the
system (typically 50Hz).

- **/vehicle_interface/speed_status** -
*(marti_common_msgs/HealthStatus)* - *publication* - Summarized health
status of the control control system.  The status field should follow the
guidelines in the HealthStatus mesage definition, and the message
field should be filled out with an informative value. SLOW/ONCHANGE.

## Robotic Mode Interface [⬆](#swri_dbw_interface)

The robotic mode interface is used to control and monitor whether the
DBW system has control of the vehicle.

- **/vehicle_interface/robotic_mode_command** -
   *(marti_common_msgs/BoolStamped)* - *subscription* - Receives
   messages to put the vehicle in or out of robotic mode.  A true
   value is request to put the vehicle in robotic mode, while a false
   value takes the vehicle out of robotic mode.  Messages may be
   ignored if suitable conditions are not met, but these should be
   logged to the console.  Messages are expected to be sent when a
   change is required.

- **/vehicle_interface/robotic_mode_feedback** -
   *(marti_common_msgs/BoolStamped)* - *publication* - Publishes the
   whether or not the vehicle is currently in robotic mode.  A true
   value indicates that the DBW system in control of the vehicle, and
   a false value indicates the vehicle is in manual control.

Most vehicles that we have worked with only allow robotic mode to be
enabled externally by a user.  We typically do not add a subscription
to the robotic_mode_command topic for those systems.

- **/vehicle_interface/robotic_mode_status** -
   *(marti_common_msgs/HealthStatus)* - *publication* - Summarized health status of
   the robotic mode control/feedback.


## Transmission Interface [⬆](#swri_dbw_interface)

The transmission interface has two main topics for control:

- **/vehicle_interface/transmission_command** -
   *(marti_common_msgs/StringStamped)* - *subscription* - Desired
   range of the transmission.  If the vehicle is not in a state where
   the transmission can be changed, the command should be discarded
   rather than queued (Occurrances should be communicated over ROS
   console).  Messages are expected only when a shift is requested.
   Requests for unknown ranges should be logged and ignored.

- **/vehicle_interface/transmission_feedback** -
   *(marti_common_msgs/StringStamped)* - *publication* - Current state
   of the transmission.  SLOW/ONCHANGE.

Common values for the transmission ranges are defined as constants in
[swri_dbw_interface/device_states.h](../include/swri_dbw_interface/device_states.h):
  - *TRANSMISSION_PARK* = **"park"**
  - *TRANSMISSION_REVERSE* = **"reverse"**
  - *TRANSMISSION_DRIVE* = **"drive"**
  - *TRANSMISSION_NEUTRAL* = **"neutral"**
  - *TRANSMISSION_SHIFTING* = **"shifting"** *(feedback only)*
  - *TRANSMISSION_UNKNOWN* = **"unknown"** *(feedback only)*

Note that some applications require additional gears (e.g. specfically
requesting high and low gear for tele-op).  These are not typically
not requested by any of our autonomous systems, but unknown values
should be expected and treated appropriately.

- **/vehicle_interface/transmission_status** -
   *(marti_common_msgs/HealthStatus)* - *publication* - The current
   *health status of the transmission interface.


## Ignition/Engine Interface [⬆](#swri_dbw_interface)

The ignition/engine device has two main topics for interacting with
the engine and a health status topic.

- **/vehicle_interface/engine_command** -
   *(marti_common_msgs/BoolStamped)* - *subscription* - This topic is
   used to start and stop the engine.  A true value is sent to start
   the engine and a false value to stop the engine.  If the vehicle
   is not in a valid state to start or stop the engine, a message
   should be sent to the console and the command should be discarded.
   Messages are expected only when a change is requested.

- **/vehicle_interface/engine_feedback** -
   *(marti_common_msgs/BoolStamped)* - *publication* - This topic
   *communicates the current state of the engine, with true indicating
   *a running engine and false indicating a stopped engine.  SLOW/ONCHANGE.

- **/vehicle_interface/engine_status** -
   *(marti_common_msgs/HealthStatus)* - *publication* - This topic
   communicates the current status of the engine system.


## Turn Signal Interface [⬆](#swri_dbw_interface)

The turn signal interface has two main topics for managing the turn
signals and one topic to indicate the health of the device.

- **/vehicle_interface/turn_signal_command** -
   *(marti_common_msgs/StringStamped)* - *subscription* - Receives
   messages to control the turn signals.  A new state is latched
   indefinitely until a new command is received.

- **/vehicle_interface/turn_signal_feedback** -
   *(marti_common_msgs/StringStamped)* - *publication* - Current state
   of the turn signal.  SLOW/ONCHANGE.

Common values for the turn signals are defined as constants in
[swri_dbw_interface/device_states.h](../include/swri_dbw_interface/device_states.h):
  - *TURN_SIGNAL_NONE* = **"none"**
  - *TURN_SIGNAL_LEFT* = **"left"**
  - *TURN_SIGNAL_RIGHT* = **"right"**
  - *TURN_SIGNAL_HAZARD* = **"hazard"** *(feedback only)*

- **/vehicle_interface/turn_signal_status** -
   *(marti_common_msgs/HealthStatus)* - *publication* - The current
   health status of the turn signal device.


## E-Stop Interface [⬆](#swri_dbw_interface)

The e-stop interface is used to assert an emergency stop (when
supported) and report the current e-stop condition (when available).

- **/vehicle_interface/estop_command** -
   *(marti_common_msgs/BoolStamped)* - *subscription* - Receives
   messages to assert or de-assert an e-stop condition.  This message
   should be sent when a change is required.  The state will be
   latched and held until changed by a subsequent message.  An e-stop
   message should not be ignored.

- **/vehicle_interface/estop_feedback** -
   *(marti_common_msgs/BoolStamped)* - *publication* - Publishes the
   current state of the e-stop condition, whether it was asserted by
   a command or external hardware.

- **/vehicle_interface/estop_status** -
   *(marti_common_msgs/HealthStatus)* - *publication* - Publishes the
   *current status of the e-stop device interface.

## Button Interfaces [⬆](#swri_dbw_interface)

Button interfaces are used to provide feedback from physical buttons accessible
through a DBW system for user input (such as on a steering wheel or console).

- **/vehicle_interface/buttons/\<NAME\>_feedback** -
   *(marti_common_msgs/BoolStamped)* - *publication* - Publishes the
   current state of the button.  SLOW/ONCHANGE
