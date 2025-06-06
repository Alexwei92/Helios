/*
===============================================================================

  FILE:  lasreaditemcompressed_v2.hpp
  
  CONTENTS:
  
    Implementation of LASitemReadCompressed for *all* items (version 2).

  PROGRAMMERS:
  
    martin.isenburg@gmail.com
  
  COPYRIGHT:

    (c) 2010-2011, Martin Isenburg, LASSO - tools to catch reality

    This is free software; you can redistribute and/or modify it under the
    terms of the GNU Lesser General Licence as published by the Free Software
    Foundation. See the COPYING file for more information.

    This software is distributed WITHOUT ANY WARRANTY and without even the
    implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
  
  CHANGE HISTORY:
  
    5 March 2011 -- created first night in ibiza to improve the RGB compressor
  
===============================================================================
*/
#ifndef LAS_READ_ITEM_COMPRESSED_V2_HPP
#define LAS_READ_ITEM_COMPRESSED_V2_HPP

#include "lasreaditem.hpp"
#include "entropydecoder.hpp"
#include "integercompressor.hpp"

#include "laszip_common_v2.hpp"

class LASreadItemCompressed_POINT10_v2 : public LASreadItemCompressed
{
public:

  LASreadItemCompressed_POINT10_v2(EntropyDecoder* dec);

  BOOL init(const U8* item);
  BOOL read(U8* item);

  ~LASreadItemCompressed_POINT10_v2();

private:
  EntropyDecoder* dec;
  U8 last_item[20];
  U16 last_intensity[16];
  StreamingMedian5 last_x_diff_median5[16];
  StreamingMedian5 last_y_diff_median5[16];
  I32 last_height[8];

  EntropyModel* m_changed_values;
  IntegerCompressor* ic_intensity;
  EntropyModel* m_scan_angle_rank[2];
  IntegerCompressor* ic_point_source_ID;
  EntropyModel* m_bit_byte[256];
  EntropyModel* m_classification[256];
  EntropyModel* m_user_data[256];
  IntegerCompressor* ic_dx;
  IntegerCompressor* ic_dy;
  IntegerCompressor* ic_z;
};

class LASreadItemCompressed_GPSTIME11_v2 : public LASreadItemCompressed
{
public:

  LASreadItemCompressed_GPSTIME11_v2(EntropyDecoder* dec);

  BOOL init(const U8* item);
  BOOL read(U8* item);

  ~LASreadItemCompressed_GPSTIME11_v2();

private:
  EntropyDecoder* dec;
  U32 last, next;
  U64I64F64 last_gpstime[4];
  I32 last_gpstime_diff[4];
  I32 multi_extreme_counter[4];

  EntropyModel* m_gpstime_multi;
  EntropyModel* m_gpstime_0diff;
  IntegerCompressor* ic_gpstime;
};

class LASreadItemCompressed_RGB12_v2 : public LASreadItemCompressed
{
public:

  LASreadItemCompressed_RGB12_v2(EntropyDecoder* dec);

  BOOL init(const U8* item);
  BOOL read(U8* item);

  ~LASreadItemCompressed_RGB12_v2();

private:
  EntropyDecoder* dec;
  U16 last_item[3];

  EntropyModel* m_byte_used;
  EntropyModel* m_rgb_diff_0;
  EntropyModel* m_rgb_diff_1;
  EntropyModel* m_rgb_diff_2;
  EntropyModel* m_rgb_diff_3;
  EntropyModel* m_rgb_diff_4;
  EntropyModel* m_rgb_diff_5;
};

class LASreadItemCompressed_BYTE_v2 : public LASreadItemCompressed
{
public:

  LASreadItemCompressed_BYTE_v2(EntropyDecoder* dec, U32 number);

  BOOL init(const U8* item);
  BOOL read(U8* item);

  ~LASreadItemCompressed_BYTE_v2();

private:
  EntropyDecoder* dec;
  U32 number;
  U8* last_item;

  EntropyModel** m_byte;
};

#endif
