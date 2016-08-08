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

#ifndef MATH_UTIL_GENERIC_RING_BUFFER_H_
#define MATH_UTIL_GENERIC_RING_BUFFER_H_
#ifndef NULL
#define NULL 0
#endif
#define GEN_RING_BUFF_DEFAULT_NUM_ELEMENTS 16

#include <cassert>
#include <algorithm>

namespace swri_math_util
{
  template <class T>
  class GenRingBuffer
  {
  public:
    // Default constructor
    GenRingBuffer()
    {
      NumElements = 0;
      this->alloc_mem(GEN_RING_BUFF_DEFAULT_NUM_ELEMENTS);
      this->init_array();
    }

    explicit GenRingBuffer(int NumElements2Alloc)
    {
      NumElements = 0;
      this->alloc_mem(NumElements2Alloc);
      this->init_array();
    }

    GenRingBuffer(const GenRingBuffer<T>& src)
    {
      this->alloc_mem(src.MaxNumElements);
      this->init_array();
      this->copyRB(src);
    }

    // Copy Assignment
    GenRingBuffer<T>& operator=(const GenRingBuffer<T>& src)
    {
      this->copyRB(src);
      return *this;
    }

    virtual ~GenRingBuffer()
    {
      delete [] HEAD;
    }

    void ResizeBuffer(int newSize)
    {
      this->realloc_mem(newSize);
    }

    int size() const
    {
      return NumElements;
    }

    int MaxSize() const
    {
      return MaxNumElements;
    }

    virtual T* operator[](int i)
    {
      return this->get(i);
    }

    virtual T* get(int i = 0) const
    {
      if (i >= NumElements) return NULL;
      int j = ((consumePtr-HEAD)+i)%MaxNumElements;
      return (&HEAD[j].Data);
    }

    T* getRaw(int i) const
    {
      if (i >= MaxNumElements)
      {
        return NULL;
      }
      return (&HEAD[i].Data);
    }

    T* getLoad() const
    {
      return &loadPtr->Data;
    }

    // getTail searches backward from index i
    T* getTail(int i = 0) const
    {
      if (i >= NumElements) return NULL;
      int j = ((loadPtr-1-HEAD)-i);
      if (j < 0)
      {
        j += MaxNumElements;
      }
      return (&HEAD[j].Data);
    }

    void load(const T &newElem)
    {
      this->push(newElem);
    }

    void load1(T newElem)
    {
      this->push(newElem);
    }

    T* pop()
    {
      T* temp;
      if (this->size() == 0)
      {
        return 0;
      }
      else
      {
        temp = this->get();
        this->incConsumePtr();
        NumElements--;
      }
      return temp;
    }

    bool indexValid(int i)
    {
      return (i >= 0 && i <this->NumElements);
    }

    void clear()
    {
      while (this->pop());
    }

  protected:
    void realloc_mem(int NumElements2Alloc)
    {
      temp = new ctr[std::max(2, NumElements2Alloc)];
      this->copy_elems(temp, NumElements2Alloc);
      delete [] HEAD;
      HEAD = temp;
      TAIL = HEAD+NumElements2Alloc;
      loadPtr = HEAD;
      consumePtr = HEAD;
      MaxNumElements = NumElements2Alloc;
      if (MaxNumElements < NumElements)
      {
        NumElements = MaxNumElements;
      }
      this->init_array();
    }

  private:
    struct ctr
    {
      T Data;
      ctr *next;
      ctr *prev;
    };
    ctr *temp;
    ctr *HEAD;
    ctr *TAIL;
    ctr *loadPtr;
    ctr *consumePtr;

    int NumElements;
    int MaxNumElements;


    bool copy_elems(ctr* dest, int MaxNum2Copy)
    {
      bool success;
      int Num2Copy = std::min(NumElements, MaxNum2Copy);
      if (Num2Copy !=NumElements)
      {
        success = false;
      }
      else
      {
        success = true;
      }
      for (int i = 0; i < Num2Copy; i++)
      {
        dest[i].Data = *(this->get(i));
      }
      return success;
    }

    void push(const T &newElem)
    {
      assert(NumElements <= MaxNumElements);
      loadPtr->Data = newElem;
      if (NumElements >= MaxNumElements)
      {
        this->incLoadPtr();
        this->incConsumePtr();
      }
      else
      {
        this->incLoadPtr();
        NumElements++;
      }
    }

    void incLoadPtr()
    {
      loadPtr = loadPtr->next;
    }

    void incConsumePtr()
    {
      consumePtr = consumePtr->next;
    }

    ctr* alloc_mem(int NumElems)
    {
      HEAD = new ctr[std::max(NumElems, 2)];
      TAIL = HEAD+NumElems;
      loadPtr = HEAD;
      consumePtr = HEAD;
      MaxNumElements = NumElems;
      return HEAD;
    }

    void init_array()
    {
      int j, k;
      for (int i = 0; i < MaxNumElements; i++)
      {
        j = (i+1)%MaxNumElements;
        k = (i-1)%MaxNumElements;
        HEAD[i].next = &HEAD[j];
        HEAD[i].prev = &HEAD[k];
      }
    }

    void copyRB(GenRingBuffer<T>& src)
    {
      this->clear();
      if (src.MaxNumElements != this->MaxNumElements)
      {
        this->realloc_mem(src.MaxNumElements);
      }

      // Initialize the new buffer's number of elements to zero. The for loop
      // below will take care of incrementing NumElements to the correct value.
      this->NumElements = 0;
      // This was the previously-used line of code, which resulted in the
      // copied-to buffer incorrectly having a NumElements greater than the
      // copy-from buffer by 1:
      // this->NumElements = src.NumElements;
      for (int i = 0; i < src.NumElements; ++i)
      {
        this->load(*(src[i]));
      }
    }
  };
}

#endif  // MATH_UTIL_GENERIC_RING_BUFFER_H_
