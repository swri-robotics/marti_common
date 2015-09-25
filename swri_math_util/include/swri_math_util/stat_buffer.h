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

#ifndef MATH_UTIL_STAT_BUFFER_H_
#define MATH_UTIL_STAT_BUFFER_H_

#ifndef PI
#define PI 3.14159265358979
#endif

#include <cmath>
#include <algorithm>

#include <swri_math_util/generic_ring_buffer.h>

namespace swri_math_util
{
  template <class T>
  class StatBuffer: public GenRingBuffer<T>
  {
  public:
    void modifyBufferSize(int NumElements)  // moved from private 04/02/2008 JJC
    {
      this->realloc_mem(NumElements);
    }

    StatBuffer()
    {
      this->modifyBufferSize(30);
    }

    explicit StatBuffer(int NumElements)
    {
      this->modifyBufferSize(NumElements);
    }

    ~StatBuffer()
    {
      this->modifyBufferSize(0);
    }

    bool UpdateStats()
    {
      return this->computeStats();
    }

    bool UpdateDiffStats()
    {
      return this->computeDiffStats();
    }

    T reportDiffMean()
    {
      return RetainedDiffStats.mean;
    }

    T reportDiffMedian()
    {
      return RetainedDiffStats.median;
    }

    T reportDiffMin()
    {
      return RetainedDiffStats.min;
    }

    T reportDiffMax()
    {
      return RetainedDiffStats.max;
    }

    T reportMean()
    {
      return RetainedStats.mean;
    }

    // Computes the mean of the last NumToAvg items in the buffer
    T reportPartialMean(int NumToAvg)
    {
      return this->computeMean(NumToAvg);
    }

    T reportMedian()
    {
      return RetainedStats.median;
    }

    T reportMin()
    {
      return RetainedStats.min;
    }

    T reportMax()
    {
      return RetainedStats.max;
    }
    T reportStd()
    {
      return RetainedStats.std;
    }

    T reportVar()
    {
      return RetainedStats.variance;
    }
    T reportRetainedStats()
    {
      return RetainedStats;
    }

  private:
    typedef struct
    {
      T mean;
      T min;
      T max;
      T median;
      T std;
      T variance;
    } StatPack;

    StatPack RetainedStats;
    StatPack RetainedDiffStats;

    T computeMean(int NumToAvg)
    {
      int NumElems = this->size();
      if (NumElems <= 0) return (T)(0.0);
      T CurVal = *this->getTail(0);
      T sum = 0;
      NumToAvg = std::min(NumToAvg, NumElems);
      for (int i = 0; i < NumToAvg; ++i)
      {
        CurVal = *this->getTail(i);
        sum += CurVal;
      }
      T mean = sum/((T)NumToAvg);
      return mean;
    }

    bool computeStats()
    {
      int NumElems = this->size();
      if (NumElems <= 0) return false;

      T sum = 0;
      T &min = RetainedStats.min;
      T &max = RetainedStats.max;
      T &mean = RetainedStats.mean;
      T &median = RetainedStats.median;
      T &std = RetainedStats.std;
      T &var = RetainedStats.variance;

      T CurVal = *this->get(0);
      sum += CurVal;
      min = CurVal;
      max = CurVal;
      mean = CurVal;
      median = CurVal;
      std = 0;
      var = std*std;

      // compute mean, min and max
      for (int i = 1; i < NumElems; i++)
      {
        CurVal = *this->get(i);
        sum += CurVal;
        if (CurVal > max) max = CurVal;
        else if (CurVal < min) min = CurVal;
      }
      mean = sum/((T)NumElems);
      sum = 0;

      // compute
      if (NumElems > 1)
      {
        T *vec1 = new T[NumElems];  // for median calculation
        for (int i = 0; i < NumElems; i++)
        {
          CurVal = *this->get(i);
          sum += (CurVal-mean)*(CurVal-mean);
          vec1[i] = CurVal;  // for median calculation
        }
        std=(T)sqrt(static_cast<double>(sum/(NumElems-1)));
        var = std*std;

        // Compute Median
        std::sort(vec1, vec1+NumElems);  // first sort the data
        if (NumElems % 2 == 0)
        {
          median = (vec1[NumElems/2-1] + vec1[NumElems/2])/2;
        }
        else
        {
          median = vec1[NumElems/2];
        }
        if (NumElems <= 1)
        {
          delete vec1;
        }
        else
        {
          delete [] vec1;
        }
      }

      return true;
    }

    bool computeDiffStats()
    {
      int NumElems = this->size();
      if (NumElems <= 1) return false;

      T sum    = 0;
      T &min    = RetainedDiffStats.min;
      T &max    = RetainedDiffStats.max;
      T &mean    = RetainedDiffStats.mean;
      T &median  = RetainedDiffStats.median;

      T *vec1 = new T[NumElems];

      T CurVal1 = *this->get(0);
      T CurVal2 = *this->get(1);
      T CVDiff = CurVal2-CurVal1;

      vec1[0] = CVDiff;

      sum += CVDiff;
      min = CVDiff;
      max = CVDiff;
      mean = CVDiff;
      median = CVDiff;
      for (int i = 1; i < NumElems-1; i++)
      {
        CurVal1 = *this->get(i);
        CurVal2 = *this->get(i+1);
        CVDiff = CurVal2-CurVal1;
        vec1[i] = CVDiff;
        sum += CVDiff;
        if (CVDiff > max) max = CVDiff;
        else if (CVDiff < min) min = CVDiff;
      }
      mean = sum/((T)NumElems);

      NumElems--;  // we put in one fewer than NumElems into the vector
      std::sort(vec1, vec1+NumElems);  // first sort the data
      if (NumElems % 2 == 0)
      {
        median = (vec1[NumElems/2-1] + vec1[NumElems/2])/2;
      }
      else
      {
        median = vec1[NumElems/2];
      }

      delete [] vec1;

      return true;
    }
  };
}


#endif  // MATH_UTIL_STAT_BUFFER_H_
