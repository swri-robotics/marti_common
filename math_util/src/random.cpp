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

#include <math_util/random.h>

namespace math_util
{
  RandomGenerator::RandomGenerator(int32_t seed) :
    rng_(seed == -1 ? seed_() : seed)
  {
  }
  
  void RandomGenerator::GetUniformRandomSample(
    int32_t min, 
    int32_t max, 
    int32_t count,
    std::vector<int32_t>& sample)
  {
    boost::unique_lock<boost::mutex> lock(mutex_);
    math_util::GetUniformRandomSample<boost::random::mt19937>(rng_, min, max, count, sample);
  }
}
