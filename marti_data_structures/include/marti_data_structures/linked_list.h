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

#ifndef MARTI_DATA_STRUCTURES_LINKED_LIST_H_
#define MARTI_DATA_STRUCTURES_LINKED_LIST_H_

#ifndef NULL
#define NULL 0
#endif

namespace marti_data_structures
{
  template<class T>
  class LinkedList
  {
  public:
    // Default constructor
    LinkedList()
    {
      NumElements = 0;
    }

    // Copy Constructor
    LinkedList(const LinkedList<T> &src)
    {
      NumElements = 0;
      this->CopyList(src, *this);
    }

    ~LinkedList()
    {
      ctr *tptr;
      while (this->size() > 0)
      {
        tptr = HEAD;
        HEAD = HEAD->next;
        this->remove(tptr);
      }
    }

    // Copy Assignment
    LinkedList<T>& operator=(const LinkedList<T>& src)
    {
      this->CopyList(src, *this);
      return *this;
    }

    void initialize()
    {
      ctr *tptr;
      while (this->size() > 0)
      {
        tptr = HEAD;
        HEAD = HEAD->next;
        this->remove(tptr);
      }
    }

    int size()
    {
      return NumElements;
    }

    void add(T &newElem)
    {
      if (this->size() == 0)
      {
        this->CreateNewLinkedList(&newElem);
      }
      else
      {
        this->addToTail(&newElem);
      }
    }

    void addCopy(const T &newElem)
    {
      T* copyElem = new T;
      *copyElem = newElem;
      this->add(*copyElem);
    }

    // Adds a new element at location i, and moves previous i (and all above) to
    // i+1 (and so on).  If i >= Size the new element will be added to the end
    // of the list
    void insertAt(T &newElem, int i)
    {
      this->addInPosition(&newElem, i);
    }

    void insertCopyAt(const T &newElem, int i)
    {
      T* copyElem = new T;
      *copyElem = newElem;
      this->addInPosition(copyElem, i);
    }

    void remove(int i)
    {
      ctr *tptr;
      tptr = this->get(i);
      if (tptr)  // if tptr!=NULL
      {
        this->remove(tptr);
      }
    }

    T* ReturnElement(int i)
    {
      // returns NULL if index out of bounds
      ctr* temp = this->get(i);
      if (NULL == temp)
      {
        return NULL;
      }
      else
      {
        return (temp->Data);
      }
    }

    // Crops list by removing all elements from i to the end (i inclusive)
    void CropList(int i)
    {
      while (this->ReturnElement(i))
      {
        this->remove(i);
      }
    }

  private:
    struct ctr
    {
      T *Data;
      ctr *next;
      ctr *prev;
    };
    ctr *temp;
    ctr *HEAD;
    ctr *TAIL;
    int NumElements;

    // void UpdatePointers()
    void CreateNewLinkedList(T *firstElement)
    {
      temp = this->alloc_elem(firstElement);
      HEAD = temp;
      TAIL = temp;
      temp->next = NULL;
      temp->prev = NULL;
      NumElements++;
    }

    void addToTail(T *newElem)
    {
      temp = this->alloc_elem(newElem);
      temp->prev = TAIL;
      TAIL = temp;
      temp->next = NULL;
      temp->prev->next = temp;
      NumElements++;
    }

    void addInPosition(T *elem, int i)
    {
      ctr *itemToMove = this->get(i);
      if (itemToMove == NULL)
      {
        this->add(*elem);  // KCK modified
      }
      else
      {
        temp = this->alloc_elem(elem);
        temp->next = itemToMove;
        temp->prev = itemToMove->prev;
        if (temp->prev != NULL)
        {
          itemToMove->prev->next = temp;
        }
        else
        {
          this->HEAD = temp;
        }
        itemToMove->prev = temp;
        NumElements++;
      }
    }

    ctr* alloc_elem(T *elem)
    {
      ctr *tptr = new ctr;
      tptr->Data = elem;
      return tptr;
    }

    void release_elem(ctr *elem)
    {
      delete elem->Data;
      delete elem;
    }

    ctr* get(int i)
    {
      if (i >= this->size() || i < 0)
      {
        return NULL;
      }
      ctr* temp = HEAD;
      for (int j = 0; j < i; j++)
      {
        temp = temp->next;
      }
      return temp;
    }

    void remove(ctr *node)
    {
      if (node->prev != NULL)
      {
        node->prev->next = node->next;
      }
      else
      {
        HEAD = node->next;
        if (HEAD != NULL)
        {
          HEAD->prev = NULL;  // KCK -- added (2008/05/23)
        }
      }
      if (node->next != NULL)
      {
        node->next->prev = node->prev;
      }
      else
      {
        TAIL = node->prev;
        if (TAIL != NULL)
        {
          TAIL->next = NULL;  // KCK -- added (2008/05/23)
        }
      }
      this->release_elem(node);
      NumElements--;
    }

    // Copy function
    void CopyList(const LinkedList<T> &src, LinkedList<T> &dest)
    {
      dest.initialize();
      int N = src.size();
      for (int i = 0; i < N; ++i)
      {
        dest.addCopy(*(src.ReturnElement(i)));
      }
    }
  };

  template<class T>
  class LinkedList_NoDealloc
  {
   public:
    // Default constructor
    LinkedList_NoDealloc()
    {
      NumElements = 0;
    }

    // Copy Constructor
    LinkedList_NoDealloc(const LinkedList_NoDealloc<T> &src)
    {
      this->CopyList(src, *this);
    }

    ~LinkedList_NoDealloc()
    {
      ctr *tptr;
      while (this->size() > 0)
      {
        tptr = HEAD;
        HEAD = HEAD->next;
        this->remove(tptr);
      }
    }

    // Copy Assignment
    LinkedList_NoDealloc<T>& operator=(const LinkedList_NoDealloc<T>& src)
    {
      this->CopyList(src, *this);
      return *this;
    }

    void initialize()
    {
      ctr *tptr;
      while (this->size() > 0)
      {
        tptr = HEAD;
        HEAD = HEAD->next;
        this->remove(tptr);
      }
    }

    int size()
    {
      return NumElements;
    }

    void add(T &newElem)
    {
      if (this->size() == 0)
      {
        this->CreateNewLinkedList(&newElem);
      }
      else
      {
        this->addToTail(&newElem);
      }
    }

    void addCopy(const T &newElem)
    {
      T* copyElem = new T;
      *copyElem = newElem;
      this->add(*copyElem);
    }

    // Adds a new element at location i, and moves previous i (and all above) to
    // i+1 (and so on).  If i >= Size the new element will be added to the end
    // of the list
    void insertAt(T &newElem, int i)
    {
      this->addInPosition(&newElem, i);
    }

    void insertCopyAt(const T &newElem, int i)
    {
      T* copyElem = new T;
      *copyElem = newElem;
      this->addInPosition(copyElem, i);
    }

    void remove(int i)
    {
      ctr *tptr;
      tptr = this->get(i);
      if (tptr)  // if tptr!=NULL
      {
        this->remove(tptr);
      }
    }

    T* ReturnElement(int i)
    {
      // returns NULL if index out of bounds
      ctr* temp = this->get(i);
      if (NULL == temp)
      {
        return NULL;
      }
      else
      {
        return (temp->Data);
      }
    }

    // Crops list by removing all elements from i to the end (i inclusive)
    void CropList(int i)
    {
      while (this->ReturnElement(i))
      {
        this->remove(i);
      }
    }

   private:
    struct ctr
    {
      T *Data;
      ctr *next;
      ctr *prev;
    };
    ctr *temp;
    ctr *HEAD;
    ctr *TAIL;
    int NumElements;

    // void UpdatePointers()
    void CreateNewLinkedList(T *firstElement)
    {
      temp = this->alloc_elem(firstElement);
      HEAD = temp;
      TAIL = temp;
      temp->next = NULL;
      temp->prev = NULL;
      NumElements++;
    }

    void addToTail(T *newElem)
    {
      temp = this->alloc_elem(newElem);
      temp->prev = TAIL;
      TAIL = temp;
      temp->next = NULL;
      temp->prev->next = temp;
      NumElements++;
    }

    void addInPosition(T *elem, int i)
    {
      ctr *itemToMove = this->get(i);
      if (itemToMove == NULL)
      {
        this->add(*elem);  // KCK modified
      }
      else
      {
        temp = this->alloc_elem(elem);
        temp->next = itemToMove;
        temp->prev = itemToMove->prev;
        if (temp->prev != NULL)
        {
          itemToMove->prev->next = temp;
        }
        else
        {
          this->HEAD = temp;
        }
        itemToMove->prev = temp;
        NumElements++;
      }
    }

    ctr* alloc_elem(T *elem)
    {
      ctr *tptr = new ctr;
      tptr->Data = elem;
      return tptr;
    }

    void release_elem(ctr *elem)
    {
      delete elem;
    }

    ctr* get(int i)
    {
      if (i >= this->size() || i < 0)
      {
        return NULL;
      }
      ctr* temp = HEAD;
      for (int j = 0; j < i; j++)
      {
        temp = temp->next;
      }
      return temp;
    }

    void remove(ctr *node)
    {
      if (node->prev != NULL)
      {
        node->prev->next = node->next;
      }
      else
      {
        HEAD = node->next;
        if (HEAD != NULL)
        {
          HEAD->prev = NULL;  // KCK -- added (2008/05/23)
        }
      }
      if (node->next != NULL)
      {
        node->next->prev = node->prev;
      }
      else
      {
        TAIL = node->prev;
        if (TAIL != NULL)
        {
          TAIL->next = NULL;  // KCK -- added (2008/05/23)
        }
      }
      this->release_elem(node);
      NumElements--;
    }

    // Copy function
    void CopyList(
      const LinkedList_NoDealloc<T> &src,
      LinkedList_NoDealloc<T> &dest)
    {
      dest.initialize();
      int N = src.size();
      for (int i = 0; i < N; ++i)
      {
        dest.addCopy(*(src.ReturnElement(i)));
      }
    }
  };
}

#endif  // MARTI_DATA_STRUCTURES_LINKED_LIST_H_
