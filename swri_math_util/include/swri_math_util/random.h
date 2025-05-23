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

#ifndef MATH_UTIL_RANDOM_H_
#define MATH_UTIL_RANDOM_H_

#include <memory>
#include <mutex>
#include <random>
#include <vector>

namespace swri_math_util
{
  class RandomGenerator
  {
    public:
      explicit RandomGenerator(int32_t seed = -1);
      
      void GetUniformRandomSample(
        int32_t min, 
        int32_t max, 
        int32_t count,
        std::vector<int32_t>& sample);
      
    private:
      std::random_device seed_;
      std::mt19937 rng_;
      std::mutex mutex_;
  };
  typedef std::shared_ptr<RandomGenerator> RandomGeneratorPtr;

  /**
   * Gets a uniform random sample of integers without repeats for a given range.
   *
   * The number of samples should be much smaller than the number of possible
   * values in the range for optimal performance.
   *
   * This function depends on a random number generator (RNG) being provided, 
   * which generally aren't thread safe.  It is recommended that a 
   * multi-threaded application create a seperate RNGs for each thread. 
   *
   * param[in]   rng     The random number generator.
   * param[in]   min     The minimum of the range (inclusive).
   * param[in]   max     The maximum of the range (inclusive).
   * param[in]   count   The sample size.
   * param[out]  sample  The sample.
   */
  template <class RNG>
  void GetUniformRandomSample(
    RNG& rng,
    int32_t min, 
    int32_t max, 
    int32_t count,
    std::vector<int32_t>& sample)
  {
    sample.clear();
    if (count < 0)
    {
      return;
    }
    
    if (min > max)
    {
      int32_t tmp = min;
      min = max;
      max = tmp;
    }
    
    int32_t range = (max - min) + 1;
    if (count > range)
    {
      count = range;
    }

    sample.resize(count);
      
    std::uniform_int_distribution<> dist(min, max);
    for (int32_t i = 0; i < count; i++)
    {
      bool has_sample = false;
      while (!has_sample)
      {
        sample[i] = dist(rng);
        int32_t j;
        for (j = 0; j < i; j++)
        {
          if (sample[i] == sample[j]) break;
        }
        has_sample = j == i;
      }
    }
  }
}

#endif  // MATH_UTIL_RANDOM_H_
