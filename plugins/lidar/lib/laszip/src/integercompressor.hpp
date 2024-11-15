/*
===============================================================================

  FILE:  integercompressor.hpp
  
  CONTENTS:
 
    This compressor provides three different contexts for encoding integer
    numbers whose range may lie anywhere between 1 and 31 bits, which is
    specified with the SetPrecision function.

    The compressor encodes two things:

      (1) the number k of miss-predicted low-order bits and
      (2) the k-bit number that corrects the missprediction

    The k-bit number is usually coded broken in two chunks. The highest
    bits are compressed using an arithmetic range table. The lower bits
    are stored raw without predicive coding. How many of the higher bits
    are compressed can be specified with BITS_HIGH. The default is 8.

  PROGRAMMERS:
  
    martin.isenburg@gmail.com
  
  COPYRIGHT:

    (c) 2005-2011, Martin Isenburg, LASSO - tools to catch reality

    This is free software; you can redistribute and/or modify it under the
    terms of the GNU Lesser General Licence as published by the Free Software
    Foundation. See the COPYING file for more information.

    This software is distributed WITHOUT ANY WARRANTY and without even the
    implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
  
  CHANGE HISTORY:
  
    10 January 2011 -- licensing change for LGPL release and liblas integration
    10 December 2010 -- unified for all entropy coders at Baeckerei Schaefer
    31 October 2009 -- switched from the Rangecoder to the Entropycoder
    30 September 2005 -- now splitting the corrector into raw and compressed bits
    13 July 2005 -- created after returning with many mosquito bites from OBX
  
===============================================================================
*/
#ifndef INTEGER_COMPRESSOR_H
#define INTEGER_COMPRESSOR_H

#include "entropyencoder.hpp"
#include "entropydecoder.hpp"

class IntegerCompressor
{
public:

  // Constructor & Deconstructor
  IntegerCompressor(EntropyEncoder* enc, U32 bits=16, U32 contexts=1, U32 bits_high=8, U32 range=0);
  IntegerCompressor(EntropyDecoder* dec, U32 bits=16, U32 contexts=1, U32 bits_high=8, U32 range=0);
  ~IntegerCompressor();

  // Manage Compressor
  void initCompressor();
  void compress(I32 iPred, I32 iReal, U32 context=0);

  // Manage Decompressor
  void initDecompressor();
  I32 decompress(I32 iPred, U32 context=0);

  // Get the k corrector bits from the last compress/decompress call
  U32 getK() const {return k;};

private:
  void writeCorrector(I32 c, EntropyModel* model);
  I32 readCorrector(EntropyModel* model);

  U32 k;

  U32 contexts;
  U32 bits_high;

  U32 bits;
  U32 range;

  U32 corr_bits;
  U32 corr_range;
  I32 corr_min;
  I32 corr_max;

  EntropyEncoder* enc;
  EntropyDecoder* dec;

  EntropyModel** mBits;

  EntropyModel** mCorrector;

  int** corr_histogram;
};

#endif
