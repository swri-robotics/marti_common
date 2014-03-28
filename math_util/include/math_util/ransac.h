// *****************************************************************************
//
// Copyright (C) 2014 All Right Reserved, Southwest Research Institute® (SwRI®)
//
// Contract No.  10-R8248
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

#ifndef MATH_UTIL_RANSAC_H_
#define MATH_UTIL_RANSAC_H_

#include <cmath>
#include <vector>

#include <boost/make_shared.hpp>

#include <math_util/random.h>

namespace math_util
{
  template <class Model>
  class Ransac
  {
  public:
    typedef typename Model::M ModelType;
    typedef typename Model::T DataType;
  
    Ransac(RandomGeneratorPtr rng = RandomGeneratorPtr()) : rng_(rng) {}

    ModelType FitModel(
      const std::vector<DataType>& data,
      double max_error,
      double confidence,
      int32_t max_iterations,
      std::vector<uint32_t>& inliers)
    {
      ModelType best_fit;
      inliers.clear();
      int32_t breakout = 1;
      
      if (!rng_)
      {
        rng_ = boost::make_shared<RandomGenerator>();
      }
      
      for (int32_t i = 0; i < max_iterations && i < breakout; i++)
      {
        std::vector<int32_t> indices;
        rng_->GetUniformRandomSample(0, data.size(), Model::MIN_SIZE, indices);
        
        std::vector<DataType> sample(indices.size());
        for (size_t j = 0; j < indices.size(); j++)
        {
          sample[j] = data[indices[j]];
        }
        
        ModelType hypothesis;
        if (Model::GetModel(sample, hypothesis))
        {
          std::vector<uint32_t> consensus_set;
          for (size_t j = 0; j < data.size(); j++)
          {
            if (Model::GetError(data[j], hypothesis) < max_error)
            {
              consensus_set.push_back(j);
            }
          }
          
          if (consensus_set.size() > inliers.size())
          {
            inliers = consensus_set;
            best_fit = hypothesis;
            
            // Recalculate breakout threshold to see if the fit is good enough.
            double ratio = inliers.size() / static_cast<double>(data.size());
            double p_no_outliers = 1.0 - std::pow(ratio, Model::MIN_SIZE);
            if (p_no_outliers == 0)
            {
              breakout = 0;
            }
            else
            {
              breakout = std::log(1 - confidence) / std::log(p_no_outliers);
            }
          }
        }
        else
        {
          // Since this iteration was degenerate, increase the breakout 
          // threshold.
          breakout++;
        }
      }
      
      // Return a least-squares fit of the inliers.      
      return best_fit;
    }

  private:
    RandomGeneratorPtr rng_;
  };
}

#endif  // MATH_UTIL_RANSAC_H_
