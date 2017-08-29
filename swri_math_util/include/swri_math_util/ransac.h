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
  
    explicit Ransac(RandomGeneratorPtr rng = RandomGeneratorPtr()) : rng_(rng) {}

    ModelType FitModel(
      Model& model,
      double max_error,
      double confidence,
      int32_t min_iterations,
      int32_t max_iterations,
      std::vector<uint32_t>& inliers, 
      int32_t& iterations)
    {
      int32_t breakout = std::numeric_limits<int32_t>::max();
      ModelType best_fit;
      inliers.clear();
      int32_t max_inliers = 0;
      
      if (!model.ValidData())
      {
        return best_fit;
      }
      
      if (!rng_)
      {
        rng_ = boost::make_shared<RandomGenerator>();
      }
      
      std::vector<int32_t> indices;

      ModelType hypothesis;
      for (iterations = 0; (iterations < max_iterations && iterations < breakout) || iterations < min_iterations; iterations++)
      {
        indices.clear();
        rng_->GetUniformRandomSample(0, model.Size() - 1, Model::MIN_SIZE, indices);

        // Generate a hypothesis model from the random sample.
        // If the sample is not degenerate, calculate the number of inliers.
        if (model.GetModel(indices, hypothesis, max_error))
        {
          int32_t inlier_count = model.GetInlierCount(hypothesis, max_error);
          
          // Update the best fit hypothesis and inliers if this hypothesis has
          // the most inliers so far.
          if (inlier_count > max_inliers)
          {
            max_inliers = inlier_count;
            Model::CopyTo(hypothesis, best_fit);
            
            // Recalculate breakout threshold to see if the fit is good enough.
            double ratio = inlier_count / static_cast<double>(model.Size());
            double p_no_outliers = 1.0 - std::pow(ratio, Model::MIN_SIZE);
            if (p_no_outliers == 0)
            {
              breakout = 0;
            }
            else if (p_no_outliers < .9999)
            {
              breakout = std::log(1 - confidence) / std::log(p_no_outliers);
            }
          }
        }
      }
      
      if (max_inliers > 0)
      {
        model.GetInliers(best_fit, max_error, inliers);
      }
      return best_fit;
    }

  private:
    RandomGeneratorPtr rng_;
  };

  template <class Model>
  class RansacBatch
  {
  public:
    typedef typename Model::M ModelType;
    typedef typename Model::T DataType;

    explicit RansacBatch(RandomGeneratorPtr rng = RandomGeneratorPtr()) : rng_(rng) {}

    ModelType FitModel(
      const DataType& data,
      double max_error,
      double confidence,
      int32_t max_iterations,
      int32_t batch_size,
      std::vector<uint32_t>& inliers,
      int32_t& iterations)
    {
      Model model(data, batch_size);
      iterations = 0;
      int32_t breakout = std::numeric_limits<int32_t>::max();
      ModelType best_fit;
      inliers.clear();
      int32_t max_inliers = 0;

      if (!model.ValidData())
      {
        return best_fit;
      }

      if (!rng_)
      {
        rng_ = boost::make_shared<RandomGenerator>();
      }

      std::vector<int32_t> indices;

      ModelType hypothesis;
      while (iterations < max_iterations && iterations < breakout)
      {
        int32_t valid = 0;
        model.ClearSamples();
        while (iterations < max_iterations && iterations < breakout && model.Samples() < batch_size)
        {
          iterations++;
          indices.clear();
          rng_->GetUniformRandomSample(0, model.Size() - 1, Model::MIN_SIZE, indices);
          model.AddSample(indices, max_error);
        }

        if (model.Samples() > 0)
        {
          int32_t inlier_count = model.ProcessSamples(hypothesis, max_error);
          if (inlier_count > 0 && inlier_count > max_inliers)
          {
            max_inliers = inlier_count;
            Model::CopyTo(hypothesis, best_fit);

            // Recalculate breakout threshold to see if the fit is good enough.
            double ratio = inlier_count / static_cast<double>(model.Size());
            double p_no_outliers = 1.0 - std::pow(ratio, Model::MIN_SIZE);
            if (p_no_outliers == 0)
            {
              breakout = 0;
            }
            else if (p_no_outliers < .9999)
            {
              breakout = std::log(1 - confidence) / std::log(p_no_outliers);
            }
          }
        }
      }

      if (max_inliers > 0)
      {
        model.GetInliers(best_fit, max_error, inliers);
      }
      return best_fit;
    }

  private:
    RandomGeneratorPtr rng_;
  };
}

#endif  // MATH_UTIL_RANSAC_H_
