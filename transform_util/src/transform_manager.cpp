// *****************************************************************************
//
// Copyright (C) 2013 All Right Reserved, Southwest Research Institute® (SwRI®)
//
// Contract No.  10-62987
// Contractor    Southwest Research Institute® (SwRI®)
// Address       6220 Culebra Road, San Antonio, Texas 78228-0510
// Contact       Kris Kozak <kkozak@swri.org> (210) 522-3854
//
// This code was developed as part of an internal research project fully funded
// by Southwest Research Institute®.
//
// THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
// KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
// PARTICULAR PURPOSE.
//
// *****************************************************************************

#include <transform_util/transform_manager.h>

#include <vector>

#include <pluginlib/class_loader.h>

namespace transform_util
{
  TransformManager::TransformManager() :
      initialized_(false)
  {
    pluginlib::ClassLoader<transform_util::Transformer> loader(
        "transform_util", "transform_util::Transformer");

    std::vector<std::string> class_names = loader.getDeclaredClasses();

    for (uint32_t i = 0; i < class_names.size(); i++)
    {
      try
      {
        boost::shared_ptr<Transformer> transformer = loader.createInstance(class_names[i]);

        std::map<std::string, std::string> supports = transformer->Supports();

        std::map<std::string, std::string>::iterator iter;
        for (iter = supports.begin(); iter != supports.end(); ++iter)
        {
          if (transformers_[iter->first].count(iter->second) > 0)
          {
            ROS_WARN("[transform_manager]: Transformer conflict for %s to %s",
                iter->first.c_str(), iter->second.c_str());
          }

          transformers_[iter->first][iter->second] = transformer;
        }
      }
      catch (pluginlib::CreateClassException& e)
      {
        ROS_FATAL("[transform_manager]: Failed to load transformer plugin '%s': %s",
            class_names[i].c_str(), e.what());
      }
    }
  }

  void TransformManager::Initialize(boost::shared_ptr<tf::TransformListener> tf)
  {
    std::map<std::string, std::map<std::string, boost::shared_ptr<Transformer> > >::iterator iter1;
    for (iter1 = transformers_.begin(); iter1 != transformers_.end(); ++iter1)
    {
      std::map<std::string, boost::shared_ptr<Transformer> >::iterator iter2;
      for (iter2 = iter1->second.begin(); iter2 != iter1->second.end(); ++iter2)
      {
        iter2->second->Initialize(tf);
      }
    }

    initialized_ = true;
  }
}
