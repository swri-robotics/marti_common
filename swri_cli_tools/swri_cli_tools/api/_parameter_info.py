# Copyright (c) 2024, Southwest Research Institute® (SwRI®)
# All rights reserved.
# 
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of Southwest Research Institute® (SwRI®) nor the
#       names of its contributors may be used to endorse or promote products
#       derived from this software without specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
# DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

from rcl_interfaces.msg import ParameterType


class ParameterInfo:
    """Hold information about node parameter."""
    def __init__(self,
                 param_name: str = None,
                 param_type: ParameterType = None,
                 param_value: str = None) -> None:

        self._param_name = param_name
        self._param_type = "Unknown"
        if param_type == ParameterType.PARAMETER_BOOL:
            self._param_type = "boolean"
        elif param_type == ParameterType.PARAMETER_INTEGER:
            self._param_type = "integer"
        elif param_type == ParameterType.PARAMETER_DOUBLE:
            self._param_type = "double"
        elif param_type == ParameterType.PARAMETER_STRING:
            self._param_type = "string"
        elif param_type == ParameterType.PARAMETER_BYTE_ARRAY:
            self._param_type = "byte array"
        elif param_type == ParameterType.PARAMETER_BOOL_ARRAY:
            self._param_type = "bool array"
        elif param_type == ParameterType.PARAMETER_INTEGER_ARRAY:
            self._param_type = "integer array"
        elif param_type == ParameterType.PARAMETER_DOUBLE_ARRAY:
            self._param_type = "double array"
        elif param_type == ParameterType.PARAMETER_STRING_ARRAY:
            self._param_type = "string array"
        elif param_type == ParameterType.PARAMETER_NOT_SET:
            self._param_type = "Not set"

        self._param_value = param_value

    @property
    def param_name(self):
        """Get parameter name."""
        return self._param_name

    @property
    def param_type(self):
        """Get parameter type."""
        return self._param_type

    @property
    def param_value(self):
        """Get parameter value."""
        return self._param_value
