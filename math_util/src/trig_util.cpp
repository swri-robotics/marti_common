// *****************************************************************************
//
// Copyright (C) 2013 All Right Reserved, Southwest Research Institute® (SwRI®)
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

#include <math_util/trig_util.h>

#include <math_util/constants.h>

namespace math_util
{
  double WrapRadians(double angle, double center)
  {
    double wrapped = angle;
    while (wrapped < center && center - wrapped > _pi)
    {
      wrapped += _2pi;
    }

    while (wrapped > center && wrapped - center > _pi)
    {
      wrapped -= _2pi;
    }

    return wrapped;
  }
}
