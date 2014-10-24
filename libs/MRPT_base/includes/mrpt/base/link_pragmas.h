/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2014, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#ifndef base_link_pragmas_H
#define base_link_pragmas_H

#include <mrpt/config.h>
#include <mrpt/utils/boost_join.h>

// ** Important! **
// In each mrpt library, search and replace:
//  MRPT_XXX_EXPORT, MRPT_XXX_IMPORT
//  BASE_IMPEXP, mrpt_xxx_EXPORTS

// If we are building the DLL (_EXPORTS), do not link against the .lib files:
#if !defined(mrpt_base_EXPORTS) && (defined(_MSC_VER) || defined(__BORLANDC__))
#	if defined(_DEBUG)
#		pragma comment (lib, BOOST_JOIN( BOOST_JOIN("libmrpt-base",MRPT_VERSION_POSTFIX),"-dbg.lib"))
#	else
#		pragma comment (lib, BOOST_JOIN( BOOST_JOIN("libmrpt-base",MRPT_VERSION_POSTFIX),".lib"))
#	endif
#endif




/*   The macros below for DLL import/export are required for Windows only.
    Mostly all the definitions in this file are copied or at least based
     on the file wx/dlimpexp.h, written by Vadim Zeitlin and published
	 under the wxWindows licence.
*/
#if defined(MRPT_OS_WINDOWS)
    /*
       __declspec works in BC++ 5 and later, Watcom C++ 11.0 and later as well
       as VC++ and gcc
     */
#    if defined(_MSC_VER) || defined(__BORLANDC__) || defined(__GNUC__) || defined(__WATCOMC__)
#        define MRPT_BASE_EXPORT __declspec(dllexport)
#        define MRPT_BASE_IMPORT __declspec(dllimport)
#    else /* compiler doesn't support __declspec() */
#        define MRPT_BASE_EXPORT
#        define MRPT_BASE_IMPORT
#    endif
#elif defined(MRPT_OS_OS2)		/* was __WXPM__ */
#    if defined (__WATCOMC__)
#        define MRPT_BASE_EXPORT __declspec(dllexport)
        /*
           __declspec(dllimport) prepends __imp to imported symbols. We do NOT
           want that!
         */
#        define MRPT_BASE_IMPORT
#    elif defined(__EMX__)
#        define MRPT_BASE_EXPORT
#        define MRPT_BASE_IMPORT
#    elif (!(defined(__VISAGECPP__) && (__IBMCPP__ < 400 || __IBMC__ < 400 )))
#        define MRPT_BASE_EXPORT _Export
#        define MRPT_BASE_IMPORT _Export
#    endif
#elif defined(MRPT_OS_APPLE)
#    ifdef __MWERKS__
#        define MRPT_BASE_EXPORT __declspec(export)
#        define MRPT_BASE_IMPORT __declspec(import)
#    endif
#elif defined(__CYGWIN__)
#    define MRPT_BASE_EXPORT __declspec(dllexport)
#    define MRPT_BASE_IMPORT __declspec(dllimport)
#endif

/* for other platforms/compilers we don't anything */
#ifndef MRPT_BASE_EXPORT
#    define MRPT_BASE_EXPORT
#    define MRPT_BASE_IMPORT
#endif

/*  Macros that map to export declaration when building the DLL, to import
	declaration if using it or to nothing at all if we are not compiling as DLL */
#if defined(MRPT_BUILT_AS_DLL)
#	if defined(mrpt_base_EXPORTS)  /* Building the DLL */
#		define BASE_IMPEXP MRPT_BASE_EXPORT
#	else  /* Using the DLL */
#		define BASE_IMPEXP MRPT_BASE_IMPORT
#	endif
#else /* not making nor using DLL */
#    define BASE_IMPEXP
#endif

// Finally this one allows exporting a class that inherits from a
// template in MS DLLs with both MSVC and GCC. To see how nasty is
// this, Google "class derived from template DLL export" and suffer...
#if defined(_MSC_VER)
#   define BASE_IMPEXP_TEMPL
#else
    // Mostly for Mingw:
#   define BASE_IMPEXP_TEMPL BASE_IMPEXP
#endif


#endif
