// *****************************************************************************
//
// Copyright (c) 2014, Southwest Research Institute® (SwRI®)
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

#ifndef MATH_UTIL_RANSAC_H_
#define MATH_UTIL_RANSAC_H_

#include <cmath>
#include <limits>
#include <vector>

#include <boost/make_shared.hpp>

#include <swri_math_util/random.h>

namespace swri_math_util
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
      std::vector<uint32_t>& inliers, 
      int32_t& iterations)
    {
      iterations = 0;
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
        iterations++;
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
          // Check that the hypothesis is even valid for the sample set used
          // to generate it before testing the full data set.
          double max_sample_error = 0;
          for (size_t j = 0; j < sample.size(); j++)
          {
            double sample_error = Model::GetError(sample[j], hypothesis);
            max_sample_error = std::max(sample_error, max_sample_error);
          }
          
          std::vector<uint32_t> consensus_set;
          if (max_sample_error < max_error)
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
