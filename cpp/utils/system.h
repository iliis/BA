#ifndef SYSTEM_H_INCLUDED
#define SYSTEM_H_INCLUDED

// detect if pointers are 32 or 64 bit
// source: https://stackoverflow.com/a/1505664

template<int> struct PaddingTo64bit {};

template<> struct PaddingTo64bit<4>
{
  // do 32-bits operations
  char __padding[8];
};

template<> struct PaddingTo64bit<8>
{
  // do 64-bits operations
  // we can't just use an empty class here, as the C++ standard requires empty
  // classes to have non-zero size so that they won't get mapped to the same
  // address
  // http://www.geeksforgeeks.org/why-is-the-size-of-an-empty-class-not-zero-in-c/
  char __padding[4];
};

#define PADDING_TO_64BIT_T  PaddingTo64bit<sizeof(void*)>


#endif /* end of include guard: SYSTEM_H_INCLUDED */
