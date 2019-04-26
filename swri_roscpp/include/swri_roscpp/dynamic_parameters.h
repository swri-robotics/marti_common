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
// ARE DISCLAIMED. IN NO EVENT SHALL SOUTHWEST RESEARCH INSTITUTE BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// *****************************************************************************

#ifndef SWRI_ROSCPP_DYNAMIC_PARAMETERS_H_
#define SWRI_ROSCPP_DYNAMIC_PARAMETERS_H_

#include <map>
#include <string>

#include <boost/thread/mutex.hpp>

#include <ros/console.h>
#include <ros/node_handle.h>

#include <dynamic_reconfigure/Config.h>
#include <dynamic_reconfigure/ConfigDescription.h>
#include <dynamic_reconfigure/GroupState.h>
#include <dynamic_reconfigure/Reconfigure.h>

namespace swri
{
  struct DynamicValue
  {
    enum Type
    {
      Bool = 0,
      Float = 1,
      Double = 2,
      Int = 3,
      String = 4
    };

    Type type;
    std::string name;
    std::string description;

    //pointer to the parameter to update on change
    boost::shared_ptr<float> flt;
    boost::shared_ptr<double> dbl;
    boost::shared_ptr<std::string> str;
    boost::shared_ptr<int> integer;
    boost::shared_ptr<bool> boolean;

    // Defaults, maximum and minimum values for this parameter
    union
    {
      double d;
      bool b;
      int i;
    } Default;
    union
    {
      double d;
      int i;
    } Min;
    union
    {
      double d;
      int i;
    } Max;
    std::string default_string;// to get around union issues with strings
  };

  template <class T>
  struct TypedParam
  {
    boost::shared_ptr<T> data;
    boost::shared_ptr<boost::mutex> mutex;

    inline
    T operator* ()
    {
      return *data;
    }

    T get()
    {
      mutex->lock();
      T d = *data;
      mutex->unlock();

      return d;
    }
  };

  typedef TypedParam<double> DoubleParam;
  typedef TypedParam<float> FloatParam;
  typedef TypedParam<int> IntParam;
  typedef TypedParam<bool> BoolParam;
  typedef TypedParam<std::string> StringParam;


  class DynamicParameters
  {
    ros::Publisher descr_pub_;
    ros::Publisher update_pub_;
    ros::ServiceServer set_service_;
    ros::NodeHandle nh_;

    std::map<std::string, DynamicValue> values_;

    boost::function<void(DynamicParameters&)> on_change_;


    boost::shared_ptr<boost::mutex> mutex_;

    bool setConfigCallback(dynamic_reconfigure::Reconfigure::Request &req,
          dynamic_reconfigure::Reconfigure::Response &rsp)
    {
      ROS_DEBUG("Got configuration change request");

      boost::mutex::scoped_lock lock(*mutex_);
      
      // update the parameters
      for (int i = 0; i < req.config.doubles.size(); i++)
      {
        dynamic_reconfigure::DoubleParameter param = req.config.doubles[i];
        std::map<std::string, DynamicValue>::iterator iter = values_.find(param.name);
        if (iter == values_.end())
        {
          ROS_ERROR("Could not find parameter '%s'", param.name.c_str());
          continue;
        }

        if (iter->second.type != DynamicValue::Double && iter->second.type != DynamicValue::Float)
        {
          ROS_ERROR("Value '%s' was not a double type.", param.name.c_str());
          continue;
        }

        if (iter->second.type == DynamicValue::Double)
        {
          *iter->second.dbl = param.value;
        }

        if (iter->second.type == DynamicValue::Float)
        {
          *iter->second.flt = (float)param.value;
        }
      }

      for (int i = 0; i < req.config.ints.size(); i++)
      {
        dynamic_reconfigure::IntParameter param = req.config.ints[i];
        std::map<std::string, DynamicValue>::iterator iter = values_.find(param.name);
        if (iter == values_.end())
        {
          ROS_ERROR("Could not find parameter '%s'", param.name.c_str());
          continue;
        }

        if (iter->second.type != DynamicValue::Int)
        {
          ROS_ERROR("Value '%s' was not a int type.", param.name.c_str());
          continue;
        }

        *iter->second.integer = param.value;
      }

      for (int i = 0; i < req.config.bools.size(); i++)
      {
        dynamic_reconfigure::BoolParameter param = req.config.bools[i];
        std::map<std::string, DynamicValue>::iterator iter = values_.find(param.name);
        if (iter == values_.end())
        {
          ROS_ERROR("Could not find parameter '%s'", param.name.c_str());
          continue;
        }

        if (iter->second.type != DynamicValue::Bool)
        {
          ROS_ERROR("Value '%s' was not a bool type.", param.name.c_str());
          continue;
        }

        *iter->second.boolean = param.value;
      }

      for (int i = 0; i < req.config.strs.size(); i++)
      {
        dynamic_reconfigure::StrParameter param = req.config.strs[i];
        std::map<std::string, DynamicValue>::iterator iter = values_.find(param.name);
        if (iter == values_.end())
        {
          ROS_ERROR("Could not find parameter '%s'", param.name.c_str());
          continue;
        }

        if (iter->second.type != DynamicValue::String)
        {
          ROS_ERROR("Value '%s' was not a string type.", param.name.c_str());
          continue;
        }

        *iter->second.str = param.value;
      }

      updateCurrent(rsp.config);

      if (on_change_)
      {
        on_change_(*this);
      }

      return true;
    }

    // Updates a config with the current parameter values
    void updateCurrent(dynamic_reconfigure::Config& config)
    {
      for (std::map<std::string, DynamicValue>::iterator value = values_.begin(); value != values_.end(); value++)
      {
        if (value->second.type == DynamicValue::Double)
        {
          dynamic_reconfigure::DoubleParameter param;
          param.name = value->first;
          param.value = *value->second.dbl;
          config.doubles.push_back(param);
        }
        else if (value->second.type == DynamicValue::Float)
        {
          dynamic_reconfigure::DoubleParameter param;
          param.name = value->first;
          param.value = *value->second.flt;
          config.doubles.push_back(param);
        }
        else if (value->second.type == DynamicValue::Int)
        {
          dynamic_reconfigure::IntParameter param;
          param.name = value->first;
          param.value = *value->second.integer;
          config.ints.push_back(param);
        }
        else if (value->second.type == DynamicValue::Bool)
        {
          dynamic_reconfigure::BoolParameter param;
          param.name = value->first;
          param.value = *value->second.boolean;
          config.bools.push_back(param);
        }
        else if (value->second.type == DynamicValue::String)
        {
          dynamic_reconfigure::StrParameter param;
          param.name = value->first;
          param.value = *value->second.str;
          config.strs.push_back(param);
        }
      }

      dynamic_reconfigure::GroupState gs;
      gs.name = "Default";
      gs.state = true;
      gs.id = 0;
      gs.parent = 0;
      config.groups.push_back(gs);
      update_pub_.publish(config);
    }

  public:

    DynamicParameters() : mutex_(new boost::mutex)
    {

    }
    
    // Sets up the node handle and publishers. Be sure to call this before finalize or any of the 'get*' calls.
    void initialize(ros::NodeHandle& pnh)
    {
      boost::mutex::scoped_lock lock(*mutex_);
      nh_ = pnh;

      descr_pub_ = nh_.advertise<dynamic_reconfigure::ConfigDescription>("parameter_descriptions", 1, true);
      update_pub_ = nh_.advertise<dynamic_reconfigure::Config>("parameter_updates", 1, true);
    }

    // Publishes the configuration parameters that have been added
    void finalize()
    {
      boost::mutex::scoped_lock lock(*mutex_);

      // publish the configs as one group
      dynamic_reconfigure::ConfigDescription rdesc;
      dynamic_reconfigure::Group group;
      group.name = "Default";
      group.type = "";
      group.id = 0;
      group.parent = 0;

      dynamic_reconfigure::GroupState gs;
      gs.name = "Default";
      gs.state = true;
      gs.id = 0;
      gs.parent = 0;
      for (std::map<std::string, DynamicValue>::iterator param = values_.begin(); param != values_.end(); param++)
      {
        std::string type;
        if (param->second.type == DynamicValue::Bool)
        {
          type = "bool";

          dynamic_reconfigure::BoolParameter desc;
          desc.name = param->first;
          desc.value = true;
          rdesc.max.bools.push_back(desc);
          desc.value = false;
          rdesc.min.bools.push_back(desc);
          desc.value = param->second.Default.b;
          rdesc.dflt.bools.push_back(desc);
        }
        else if (param->second.type == DynamicValue::Float)
        {
          type = "double";

          dynamic_reconfigure::DoubleParameter desc;
          desc.name = param->first;
          desc.value = param->second.Max.d;
          rdesc.max.doubles.push_back(desc);
          desc.value = param->second.Min.d;
          rdesc.min.doubles.push_back(desc);
          desc.value = param->second.Default.d;
          rdesc.dflt.doubles.push_back(desc);
        }
        else if (param->second.type == DynamicValue::Double)
        {
          type = "double";

          dynamic_reconfigure::DoubleParameter desc;
          desc.name = param->first;
          desc.value = param->second.Max.d;
          rdesc.max.doubles.push_back(desc);
          desc.value = param->second.Min.d;
          rdesc.min.doubles.push_back(desc);
          desc.value = param->second.Default.d;
          rdesc.dflt.doubles.push_back(desc);
        }
        else if (param->second.type == DynamicValue::Int)
        {
          type = "int";

          dynamic_reconfigure::IntParameter desc;
          desc.name = param->first;
          desc.value = param->second.Max.i;
          rdesc.max.ints.push_back(desc);
          desc.value = param->second.Min.i;
          rdesc.min.ints.push_back(desc);
          desc.value = param->second.Default.i;
          rdesc.dflt.ints.push_back(desc);
        }
        else if (param->second.type == DynamicValue::String)
        {
          type = "string";
          dynamic_reconfigure::StrParameter desc;
          desc.name = param->first;
          desc.value = "";
          rdesc.max.strs.push_back(desc);
          desc.value = "";
          rdesc.min.strs.push_back(desc);
          desc.value = param->second.default_string;
          rdesc.dflt.strs.push_back(desc);
        }

        dynamic_reconfigure::ParamDescription desc;
        desc.name = param->first;
        desc.type = type;
        desc.level = 0;
        desc.description = param->second.description;
        desc.edit_method = "";
        group.parameters.push_back(desc);
      }
      rdesc.max.groups.push_back(gs);
      rdesc.min.groups.push_back(gs);
      rdesc.dflt.groups.push_back(gs);
      rdesc.groups.push_back(group);
      descr_pub_.publish(rdesc);

      dynamic_reconfigure::Config config;
      updateCurrent(config);

      set_service_ = nh_.advertiseService("set_parameters",
            &DynamicParameters::setConfigCallback, this);
    }

    void setCallback(boost::function<void(DynamicParameters&)> fun)
    {
      on_change_ = fun;
    }

    //for use in the on change callback
    double getDouble(const std::string& name)
    {
      std::map<std::string, DynamicValue>::iterator iter = values_.find(name);
      if (iter == values_.end())
      {
        ROS_ERROR("Tried to get nonexistant param %s", name.c_str());
        return 0.0;
      }
      if (iter->second.type != DynamicValue::Double)
      {
        ROS_ERROR("Tried to load parameter %s with the wrong type: double.", name.c_str());
        return 0.0;
      }

      return *iter->second.dbl;
    }

    //for use in the on change callback
    float getFloat(const std::string& name)
    {
      std::map<std::string, DynamicValue>::iterator iter = values_.find(name);
      if (iter == values_.end())
      {
        ROS_ERROR("Tried to get nonexistant param %s", name.c_str());
        return 0.0f;
      }
      if (iter->second.type != DynamicValue::Float)
      {
        ROS_ERROR("Tried to load parameter %s with the wrong type: float.", name.c_str());
        return 0.0f;
      }

      return *iter->second.flt;
    }

    int getInt(const std::string& name)
    {
      std::map<std::string, DynamicValue>::iterator iter = values_.find(name);
      if (iter == values_.end())
      {
        ROS_ERROR("Tried to get nonexistant param %s", name.c_str());
        return 0.0f;
      }
      if (iter->second.type != DynamicValue::Int)
      {
        ROS_ERROR("Tried to load parameter %s with the wrong type: int.", name.c_str());
        return 0.0f;
      }

      return *iter->second.integer;
    }

    //for use in the on change callback
    bool getBool(const std::string& name)
    {
      std::map<std::string, DynamicValue>::iterator iter = values_.find(name);
      if (iter == values_.end())
      {
        ROS_ERROR("Tried to get nonexistant param %s", name.c_str());
        return false;
      }
      if (iter->second.type != DynamicValue::Bool)
      {
        ROS_ERROR("Tried to load parameter %s with the wrong type: bool.", name.c_str());
        return false;
      }

      return *iter->second.boolean;
    }

    //for use in the on change callback
    std::string getString(const std::string& name)
    {
      std::map<std::string, DynamicValue>::iterator iter = values_.find(name);
      if (iter == values_.end())
      {
        ROS_ERROR("Tried to get nonexistant param %s", name.c_str());
        return "";
      }
      if (iter->second.type != DynamicValue::String)
      {
        ROS_ERROR("Tried to load parameter %s with the wrong type: string.", name.c_str());
        return "";
      }

      return *iter->second.str;
    }


    inline
    boost::mutex& mutex()
    {
      return *mutex_;
    }

    inline
    boost::mutex::scoped_lock lock_guard()
    {
      return boost::mutex::scoped_lock(*mutex_);
    }

    // for use with on change callbacks
    inline
    void get(const std::string &name,
      float &variable,
      const float default_value,
      const std::string description = "None.",
      const float min = -100,
      const float max = 100)
    {
      DynamicValue value;
      value.type = DynamicValue::Float;
      value.description = description;
      value.Min.d = min;
      value.Max.d = max;
      value.Default.d = default_value;
      value.flt = boost::shared_ptr<float>(new float);
      values_[name] = value;

      std::string resolved_name = nh_.resolveName(name);
      //_used_params.insert(resolved_name);
      nh_.param(name, *value.flt, default_value);
      variable = *value.flt;
      ROS_INFO("Read dynamic parameter %s = %f", name.c_str(), variable);
    }

    inline
    void get(const std::string &name,
      FloatParam &variable,
      const float default_value,
      const std::string description = "None.",
      const float min = -100,
      const float max = 100)
    {
      DynamicValue value;
      value.type = DynamicValue::Float;
      value.description = description;
      value.Min.d = min;
      value.Max.d = max;
      value.Default.d = default_value;
      value.flt = boost::shared_ptr<float>(new float);
      values_[name] = value;

      variable.data = value.flt;
      variable.mutex = mutex_;

      std::string resolved_name = nh_.resolveName(name);
      //_used_params.insert(resolved_name);
      nh_.param(name, *value.flt, default_value);
      ROS_INFO("Read dynamic parameter %s = %f", name.c_str(), *variable);
    }

    // for use with on change callbacks
    inline
    void get(const std::string &name,
      double &variable,
      const double default_value,
      const std::string description = "None.",
      const double min = -100,
      const double max = 100)
    {
      DynamicValue value;
      value.type = DynamicValue::Double;
      value.description = description;
      value.Min.d = min;
      value.Max.d = max;
      value.Default.d = default_value;
      value.dbl = boost::shared_ptr<double>(new double);
      values_[name] = value;

      std::string resolved_name = nh_.resolveName(name);
      //_used_params.insert(resolved_name);
      nh_.param(name, *value.dbl, default_value);
      variable = *value.dbl;
      ROS_INFO("Read dynamic parameter %s = %lf", name.c_str(), variable);
    }
    
    inline
    void get(const std::string &name,
      DoubleParam &variable,
      const double default_value,
      const std::string description = "None.",
      const double min = -100,
      const double max = 100)
    {
      DynamicValue value;
      value.type = DynamicValue::Double;
      value.description = description;
      value.Min.d = min;
      value.Max.d = max;
      value.Default.d = default_value;
      value.dbl = boost::shared_ptr<double>(new double);
      values_[name] = value;

      variable.data = value.dbl;
      variable.mutex = mutex_;

      std::string resolved_name = nh_.resolveName(name);
      //_used_params.insert(resolved_name);
      nh_.param(name, *value.dbl, default_value);
      ROS_INFO("Read dynamic parameter %s = %lf", name.c_str(), *variable);
    }

    inline
    void get(const std::string &name,
      int &variable,
      const int default_value,
      const std::string description = "None.",
      const int min = -100,
      const int max = 100)
    {
      DynamicValue value;
      value.type = DynamicValue::Int;
      value.description = description;
      value.Min.i = min;
      value.Max.i = max;
      value.Default.i = default_value;
      value.integer = boost::shared_ptr<int>(new int);
      values_[name] = value;

      std::string resolved_name = nh_.resolveName(name);
      //_used_params.insert(resolved_name);
      nh_.param(name, *value.integer, default_value);
      variable = *value.integer;
      ROS_INFO("Read dynamic parameter %s = %i", name.c_str(), variable);
    }

    inline
    void get(const std::string &name,
      IntParam &variable,
      const int default_value,
      const std::string description = "None.",
      const int min = -100,
      const int max = 100)
    {
      DynamicValue value;
      value.type = DynamicValue::Int;
      value.description = description;
      value.Min.i = min;
      value.Max.i = max;
      value.Default.i = default_value;
      value.integer = boost::shared_ptr<int>(new int);
      values_[name] = value;

      variable.data = value.integer;
      variable.mutex = mutex_;
 
      std::string resolved_name = nh_.resolveName(name);
      //_used_params.insert(resolved_name);
      nh_.param(name, *value.integer, default_value);
      ROS_INFO("Read dynamic parameter %s = %i", name.c_str(), *variable);
    }

    // for use with on change callbacks
    inline
    void get(const std::string &name,
      bool &variable,
      const bool default_value,
      const std::string description = "None.")
    {
      DynamicValue value;
      value.type = DynamicValue::Bool;
      value.description = description;
      value.Default.b = default_value;
      value.boolean = boost::shared_ptr<bool>(new bool);
      values_[name] = value;

      std::string resolved_name = nh_.resolveName(name);
      //_used_params.insert(resolved_name);
      nh_.param(name, *value.boolean, default_value);
      variable = *value.boolean;
      ROS_INFO("Read dynamic parameter %s = %s", name.c_str(), variable ? "true" : "false");
    }

    inline
    void get(const std::string &name,
      BoolParam &variable,
      const bool default_value,
      const std::string description = "None.")
    {
      DynamicValue value;
      value.type = DynamicValue::Bool;
      value.description = description;
      value.Default.b = default_value;
      value.boolean = boost::shared_ptr<bool>(new bool);
      values_[name] = value;

      variable.data = value.boolean;
      variable.mutex = mutex_;
 
      std::string resolved_name = nh_.resolveName(name);
      //_used_params.insert(resolved_name);
      nh_.param(name, *value.boolean, default_value);
      ROS_INFO("Read dynamic parameter %s = %s", name.c_str(), *variable ? "true" : "false");
    }

    // for use with on change callbacks
    inline
    void get(const std::string &name,
      std::string &variable,
      const std::string default_value,
      const std::string description = "None.")
    {
      DynamicValue value;
      value.type = DynamicValue::Bool;
      value.description = description;
      value.default_string = default_value;
      value.str = boost::shared_ptr<std::string>(new std::string());
      values_[name] = value;

      std::string resolved_name = nh_.resolveName(name);
      //_used_params.insert(resolved_name);
      nh_.param(name, *value.str, default_value);
      variable = *value.str;
      ROS_INFO("Read dynamic parameter %s = %s", name.c_str(), variable.c_str());
    }

    inline
    void get(const std::string &name,
      StringParam &variable,
      const std::string default_value,
      const std::string description = "None.")
    {
      DynamicValue value;
      value.type = DynamicValue::Bool;
      value.description = description;
      value.default_string = default_value;
      value.str = boost::shared_ptr<std::string>(new std::string());
      values_[name] = value;
 
      variable.data = value.str;
      variable.mutex = mutex_;

      std::string resolved_name = nh_.resolveName(name);
      //_used_params.insert(resolved_name);
      nh_.param(name, *value.str, default_value);
      ROS_INFO("Read dynamic parameter %s = %s", name.c_str(), (*variable).c_str());
    }
  };
}  // namespace swri_param
#endif  // SWRI_ROSCPP_DYNAMIC_PARAMETERS_H_
