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
#include <limits>
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
      
      if (data.size() < Model::MIN_SIZE)
      {
        return best_fit;
      }
      
      int32_t breakout = std::numeric_limits<int32_t>::max();
      
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
        
        // Generate a hypothesis model from the random sample.
        // If the sample is not degenerate, calculate the number of inliers.
        ModelType hypothesis;
        if (Model::GetModel(sample, hypothesis))
        {
          std::vector<uint32_t> consensus_set;
          
          // Check that the hypothesis is even valid for the sample set used
          // to generate it before testing the full data set.
          if (Model::GetError(sample, hypothesis) < max_error)
          {
            // Find all the inliers in the full data set.
            for (size_t j = 0; j < data.size(); j++)
            {
              if (Model::GetError(data[j], hypothesis) < max_error)
              {
                consensus_set.push_back(j);
              }
            }
          }
          
          // Update the best fit hypothesis and inliers if this hypothesis has
          // the most inliers so far.
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
      }
      
      // Return a least-squares fit of the inliers.      
      return best_fit;
    }

  private:
    RandomGeneratorPtr rng_;
  };
}

#endif  // MATH_UTIL_RANSAC_H_
