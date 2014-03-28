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

#ifndef MATH_UTIL_RANDOM_H_
#define MATH_UTIL_RANDOM_H_

#include <vector>

#include <boost/random/mersenne_twister.hpp>
#include <boost/random/uniform_int.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/thread/mutex.hpp>

#ifdef BOOST_1_46
  #include <boost/nondet_random.hpp>
#else
  #include <boost/random/random_device.hpp>
#endif

namespace math_util
{
  #ifdef BOOST_1_46
  namespace boost_random = boost;
  #else
  namespace boost_random = boost::random;
  #endif

  class RandomGenerator
  {
    public:
      RandomGenerator(int32_t seed = -1);
      
      void GetUniformRandomSample(
        int32_t min, 
        int32_t max, 
        int32_t count,
        std::vector<int32_t>& sample);
      
    private:
      boost_random::random_device seed_;
      boost_random::mt19937 rng_;
      boost::mutex mutex_;
  };
  typedef boost::shared_ptr<RandomGenerator> RandomGeneratorPtr;

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
      
    boost::uniform_int<> dist(min, max);
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
