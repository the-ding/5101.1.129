/* zutil.h -- internal interface and configuration of the compression library
 * Copyright (C) 1995-2012 Jean-loup Gailly.
 * For conditions of distribution and use, see copyright notice in zlib.h
 */

/* WARNING: this file should *not* be used by applications. It is
   part of the implementation of the compression library and is
   subject to change. Applications should only use zlib.h.
 */

/* @(#) $Id$ */

#ifndef ZUTIL_H
#define ZUTIL_H

#define ZLIB_INTERNAL

#include "zlib.h"

#include <stddef.h>
#include <string.h>
#include <stdlib.h>

#ifdef Z_SOLO
   typedef long ptrdiff_t;  /* guess -- will be caught if guess is wrong */
#endif

#ifndef local
#  define local static
#endif
/* compile with -Dlocal if your debugger can't find static symbols */

typedef unsigned char  uch;
typedef unsigned short ush;
typedef unsigned long  ulg;

extern const char * const z_errmsg[10]; /* indexed by 2-zlib_error */
/* (size given to avoid silly warnings with Visual C++) */

#define ERR_MSG(err) z_errmsg[Z_NEED_DICT-(err)]

#define ERR_RETURN(strm,err) \
  return (strm->msg = (char*)ERR_MSG(err), (err))
/* To be used only when the state is known to be valid */

        /* common constants */

#ifndef DEF_WBITS
#  define DEF_WBITS MAX_WBITS
#endif
/* default windowBits for decompression. MAX_WBITS is for compression only */

#if MAX_MEM_LEVEL >= 8
#  define DEF_MEM_LEVEL 8
#else
#  define DEF_MEM_LEVEL  MAX_MEM_LEVEL
#endif
/* default memLevel */

#define STORED_BLOCK 0
#define STATIC_TREES 1
#define DYN_TREES    2
/* The three kinds of block type */

#define MIN_MATCH  3
#define MAX_MATCH  258
/* The minimum and maximum match lengths */

#define PRESET_DICT 0x20 /* preset dictionary flag in zlib header */

        /* target dependencies */

#define OS_CODE  0x03  /* assume Unix */

         /* functions */

   void ZLIB_INTERNAL zmemcpy OF((Byte *far dest, const Byte *far  source, uInt len));
   int ZLIB_INTERNAL zmemcmp OF((const Byte *far s1, const Byte *far s2, uInt len));
   void ZLIB_INTERNAL zmemzero OF((Byte *far dest, uInt len));

/* Diagnostic functions */
#  define Assert(cond,msg)
#  define Trace(x)
#  define Tracev(x)
#  define Tracevv(x)
#  define Tracec(c,x)
#  define Tracecv(c,x)

#ifndef Z_SOLO
   void *far  ZLIB_INTERNAL zcalloc OF((void *far  opaque, unsigned items,
                                    unsigned size));
   void ZLIB_INTERNAL zcfree  OF((void *far  opaque, void *far  ptr));
#endif


/* Reverse the bytes in a 32-bit value */
#define ZSWAP32(q) ((((q) >> 24) & 0xff) + (((q) >> 8) & 0xff00) + \
                    (((q) & 0xff00) << 8) + (((q) & 0xff) << 24))

#endif /* ZUTIL_H */
