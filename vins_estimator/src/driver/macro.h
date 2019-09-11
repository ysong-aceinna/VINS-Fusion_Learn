/*******************************************************
 * Copyright (C) 2019
 *
 * Author: SongYang (ysong@aceinna.com)
 *******************************************************/
#pragma once

#if !defined (MAX)
#define MAX(x,y) ((x)>(y)?(x):(y))
#endif

#if !defined (MIN)
#define MIN(x,y) ((x)<(y)?(x):(y))
#endif

#if !defined (SWAPTWO)
#define SWAPTWO( a, b, temp ) \
{                                       \
  (temp) = (a);                         \
  (a) = (b);                            \
  (b) = (temp);                         \
}
#endif

#if !defined (WHITESPOT)
#define WHITESPOT 1    //pixel value for white
#endif

#if !defined (BLACKSPOT)
#define BLACKSPOT 0    //pixel value for black
#endif

// common macros
#if !defined (SAFEDELETE)
#define SAFEDELETE(p) if( (p) ){ delete (p); (p) = NULL; }
#endif

#if !defined (SAFEDELETEARRAY)
#define SAFEDELETEARRAY(p) if( (p) ){ delete [] (p); (p) = NULL; }
#endif
